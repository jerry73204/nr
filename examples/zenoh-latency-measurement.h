// Copyright (c) 2024
// SPDX-License-Identifier: GPL-2.0-only

/**
 * @file zenoh-latency-measurement.h
 * @brief Zenoh end-to-end latency measurement for ns-3 simulation
 *
 * Measures pure network latency through NS-3 simulation by tracking packets
 * using payload sequence numbers. Since Zenoh routers repackage data (changing
 * outer frame headers), we use the application-layer payload pattern "[XXXX]"
 * from z_pub.rs to correlate the same logical packet across network hops.
 *
 * Approach: Extract sequence number from payload pattern "[   X]" or "[XXXX]"
 * which is preserved end-to-end regardless of Zenoh router processing.
 *
 * Network topology and measurement points:
 *
 *   External Publisher (Docker)
 *         |
 *   [tap_remote] ─── Ghost Node
 *         |
 *   Remote Host NS-3 Node  ◄── START: RecordSend() - packet enters NS-3
 *         |
 *      WAN P2P (configurable delay)
 *         |
 *   Edge Server (remoteHostEdgeId)
 *         |
 *   [tap_edge] ─── Docker Zenoh routing ─── [tap_edge]
 *         |
 *   Edge Server (serving UE)  ◄── RecordHopIngress() + RecordHopEgress()
 *         |
 *      5G NR tunnel
 *         |
 *        UE  ◄── END: RecordReceive() - packet arrives at UE
 *         |
 *   [tap_ue] ─── Ghost Node
 *         |
 *   External Subscriber (Docker)
 *
 * Currently recorded timestamps:
 * - send_time: When packet enters NS-3 at Remote Host (from tap_remote)
 * - recv_time: When packet arrives at UE (from 5G tunnel)
 * - hops[]: Ingress/egress at serving edge only (Edge Server -> 5G tunnel)
 *
 * NOT recorded (future work - requires additional hooks):
 * - Intermediate edge hops (Edge 5 -> Docker -> Edge X)
 * - These pass through normal IP routing without EdgeTunnelApp interception
 *
 * This measures pure NS-3 network latency, excluding:
 * - External Zenoh processing time (Docker routing)
 * - Docker/host network stack delays
 */

#ifndef ZENOH_LATENCY_MEASUREMENT_H
#define ZENOH_LATENCY_MEASUREMENT_H

#include "ns3/packet.h"
#include "ns3/simulator.h"
#include "ns3/nstime.h"

#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <sstream>
#include <vector>

namespace ns3
{

//==============================================================================
// Zenoh Protocol Constants
//==============================================================================

namespace ZenohMsgId
{
    // Transport layer (lower 5 bits of header)
    constexpr uint8_t FRAME = 0x05;
    constexpr uint8_t FRAGMENT = 0x06;

    // Network layer
    constexpr uint8_t PUSH = 0x1D;

    // Zenoh layer (PushBody)
    constexpr uint8_t PUT = 0x01;
    constexpr uint8_t DEL = 0x02;
}

namespace ZenohFlag
{
    constexpr uint8_t Z = 0x80;  // Has extensions
    constexpr uint8_t R = 0x20;  // Reliable (Frame)
    constexpr uint8_t N = 0x20;  // Named - has key suffix (Push)
    constexpr uint8_t T = 0x20;  // Has timestamp (Put)
    constexpr uint8_t E = 0x40;  // Has encoding (Put)
}

namespace ZenohExtId
{
    constexpr uint8_t SOURCE_INFO = 0x01;  // Put extension ID for SourceInfo
}

//==============================================================================
// VLE (Variable Length Encoding) Decoder
//==============================================================================

inline uint64_t DecodeVLE(const uint8_t* data, size_t maxLen, size_t& bytesRead)
{
    uint64_t value = 0;
    int shift = 0;
    bytesRead = 0;

    for (size_t i = 0; i < maxLen && i < 9; i++)
    {
        uint8_t b = data[i];
        value |= ((uint64_t)(b & 0x7F)) << shift;
        bytesRead++;

        if ((b & 0x80) == 0)
        {
            return value;
        }
        shift += 7;
    }

    bytesRead = 0;
    return 0;
}

//==============================================================================
// Zenoh Source Info - End-to-End Message Identifier
//==============================================================================

/**
 * @brief Source Info extracted from Zenoh Put message
 *
 * This uniquely identifies a message from a specific publisher:
 * - sourceId: ZenohID (16 bytes) + EntityID = unique publisher identifier
 * - sourceSn: Sequence number from that publisher
 *
 * Together they form a unique end-to-end message identifier.
 */
struct ZenohSourceInfo
{
    std::string sourceIdHex;  // ZenohID + EntityID as hex string
    uint32_t sourceSn = 0;    // Source sequence number

    // Transport SN (for debugging only - changes at each hop!)
    uint32_t transportSn = 0;
    bool isReliable = false;

    /**
     * @brief Get unique key for this message
     */
    std::string GetKey() const
    {
        std::ostringstream oss;
        oss << sourceIdHex << ":" << sourceSn;
        return oss.str();
    }

    bool IsValid() const
    {
        return !sourceIdHex.empty();
    }
};

//==============================================================================
// Zenoh Message Parser
//==============================================================================

/**
 * @brief Parse Zenoh message to extract Source Info
 *
 * Parses through protocol layers:
 * - IP header (skip)
 * - TCP header (skip)
 * - 2-byte length prefix (skip)
 * - Frame (Transport) -> extract transport SN
 * - Push (Network) -> skip wire_expr and extensions
 * - Put (Zenoh) -> extract ext_sinfo (SourceInfo)
 *
 * @param packet The packet to parse
 * @param debug Enable debug output
 * @return ZenohSourceInfo if successful
 */
inline std::optional<ZenohSourceInfo> ParseZenohSourceInfo(Ptr<const Packet> packet, bool debug = false)
{
    constexpr size_t MIN_PACKET_SIZE = 50;

    if (packet->GetSize() < MIN_PACKET_SIZE)
    {
        return std::nullopt;
    }

    uint8_t buffer[512];
    uint32_t copied = packet->CopyData(buffer, sizeof(buffer));

    if (copied < MIN_PACKET_SIZE)
    {
        return std::nullopt;
    }

    //--------------------------------------------------------------------------
    // Parse IP header
    //--------------------------------------------------------------------------
    uint8_t ipVersion = (buffer[0] >> 4) & 0x0F;
    if (ipVersion != 4)
    {
        return std::nullopt;
    }

    uint8_t ipHeaderLen = (buffer[0] & 0x0F) * 4;
    if (ipHeaderLen < 20)
    {
        return std::nullopt;
    }

    uint8_t protocol = buffer[9];
    if (protocol != 6)  // TCP only
    {
        return std::nullopt;
    }

    //--------------------------------------------------------------------------
    // Parse TCP header
    //--------------------------------------------------------------------------
    size_t tcpOffset = ipHeaderLen;
    if (copied < tcpOffset + 20)
    {
        return std::nullopt;
    }

    uint8_t tcpDataOffset = (buffer[tcpOffset + 12] >> 4) * 4;
    if (tcpDataOffset < 20)
    {
        return std::nullopt;
    }

    uint16_t srcPort = (buffer[tcpOffset] << 8) | buffer[tcpOffset + 1];
    uint16_t dstPort = (buffer[tcpOffset + 2] << 8) | buffer[tcpOffset + 3];

    //--------------------------------------------------------------------------
    // Skip to Zenoh payload (after 2-byte length prefix)
    //--------------------------------------------------------------------------
    size_t zenohOffset = ipHeaderLen + tcpDataOffset + 2;  // +2 for length prefix

    if (copied < zenohOffset + 5)
    {
        return std::nullopt;
    }

    const uint8_t* zenoh = buffer + zenohOffset;
    size_t zenohLen = copied - zenohOffset;
    size_t pos = 0;

    if (debug)
    {
        std::cout << "[ZENOH] TCP " << srcPort << "->" << dstPort
                  << " zenohLen=" << zenohLen << std::endl;
    }

    //--------------------------------------------------------------------------
    // Parse Frame header (Transport layer)
    //--------------------------------------------------------------------------
    uint8_t frameHeader = zenoh[pos++];
    uint8_t frameMsgId = frameHeader & 0x1F;

    if (frameMsgId != ZenohMsgId::FRAME && frameMsgId != ZenohMsgId::FRAGMENT)
    {
        return std::nullopt;
    }

    ZenohSourceInfo result;
    result.isReliable = (frameHeader & ZenohFlag::R) != 0;

    // Decode transport SN (VLE) - for debugging only
    size_t snBytes;
    result.transportSn = static_cast<uint32_t>(DecodeVLE(zenoh + pos, zenohLen - pos, snBytes));
    if (snBytes == 0)
    {
        return std::nullopt;
    }
    pos += snBytes;

    if (debug)
    {
        std::cout << "[ZENOH] Frame: transportSn=" << result.transportSn << std::endl;
    }

    // Skip Frame extensions if Z flag set
    if (frameHeader & ZenohFlag::Z)
    {
        while (pos < zenohLen)
        {
            uint8_t extByte = zenoh[pos++];
            if ((extByte & 0x80) == 0) break;  // Last extension
        }
    }

    //--------------------------------------------------------------------------
    // Parse Push header (Network layer)
    //--------------------------------------------------------------------------
    if (pos >= zenohLen)
    {
        return std::nullopt;
    }

    uint8_t pushHeader = zenoh[pos++];
    uint8_t pushMsgId = pushHeader & 0x1F;

    if (pushMsgId != ZenohMsgId::PUSH)
    {
        return std::nullopt;
    }

    // Skip key_scope (VLE)
    size_t keyScopeBytes;
    DecodeVLE(zenoh + pos, zenohLen - pos, keyScopeBytes);
    pos += keyScopeBytes;

    // Skip key_suffix if N flag set
    if (pushHeader & ZenohFlag::N)
    {
        size_t suffixLenBytes;
        uint64_t suffixLen = DecodeVLE(zenoh + pos, zenohLen - pos, suffixLenBytes);
        pos += suffixLenBytes + suffixLen;
    }

    // Skip Push extensions if Z flag set
    if (pushHeader & ZenohFlag::Z)
    {
        while (pos < zenohLen)
        {
            uint8_t extHeader = zenoh[pos++];
            size_t vleBytes;
            DecodeVLE(zenoh + pos, zenohLen - pos, vleBytes);
            pos += vleBytes;
            if ((extHeader & 0x80) == 0) break;
        }
    }

    //--------------------------------------------------------------------------
    // Parse Put header (Zenoh layer)
    //--------------------------------------------------------------------------
    if (pos >= zenohLen)
    {
        return std::nullopt;
    }

    uint8_t putHeader = zenoh[pos++];
    uint8_t putMsgId = putHeader & 0x1F;

    if (putMsgId != ZenohMsgId::PUT && putMsgId != ZenohMsgId::DEL)
    {
        return std::nullopt;
    }

    // Skip timestamp if T flag set
    if (putHeader & ZenohFlag::T)
    {
        // Timestamp: 8 bytes NTP64 + variable ZenohID
        pos += 8;  // NTP64
        if (pos >= zenohLen) return std::nullopt;

        uint8_t idLenByte = zenoh[pos++];
        uint8_t idLen = (idLenByte >> 4) + 1;
        pos += idLen - 1;  // -1 because first byte already counted
    }

    // Skip encoding if E flag set
    if (putHeader & ZenohFlag::E)
    {
        size_t encBytes;
        DecodeVLE(zenoh + pos, zenohLen - pos, encBytes);
        pos += encBytes;
    }

    //--------------------------------------------------------------------------
    // Parse Put extensions - looking for SourceInfo (ext ID = 0x01)
    //--------------------------------------------------------------------------
    if (!(putHeader & ZenohFlag::Z))
    {
        if (debug)
        {
            std::cout << "[ZENOH] Put has no extensions (no SourceInfo)" << std::endl;
        }
        return std::nullopt;  // No extensions, no SourceInfo
    }

    while (pos < zenohLen)
    {
        uint8_t extHeader = zenoh[pos++];
        uint8_t extId = extHeader & 0x0F;
        bool hasMore = (extHeader & 0x80) != 0;

        if (extId == ZenohExtId::SOURCE_INFO)
        {
            // Parse SourceInfo extension:
            //   7 6 5 4 3 2 1 0
            //  +-+-+-+-+-+-+-+-+
            //  |zid_len|  zid  |  <- first byte: upper 4 bits = len-1, lower 4 = start of zid
            //  +-------+-------+
            //  ~      zid      ~  <- remaining zid bytes
            //  +---------------+
            //  %      eid      %  <- VLE entity ID
            //  +---------------+
            //  %      sn       %  <- VLE source sequence number
            //  +---------------+

            if (pos >= zenohLen) return std::nullopt;

            uint8_t zidLenByte = zenoh[pos++];
            uint8_t zidLen = (zidLenByte >> 4) + 1;  // Length encoded as len-1

            if (pos + zidLen - 1 > zenohLen) return std::nullopt;

            // Build source ID string: ZenohID + EntityID
            std::ostringstream oss;
            oss << std::hex << std::setfill('0');

            // First nibble of ZenohID is in lower 4 bits of length byte
            oss << std::setw(1) << (int)(zidLenByte & 0x0F);

            // Remaining ZenohID bytes
            for (uint8_t i = 0; i < zidLen - 1 && pos < zenohLen; i++)
            {
                oss << std::setw(2) << (int)zenoh[pos++];
            }

            // Entity ID (VLE)
            size_t eidBytes;
            uint64_t eid = DecodeVLE(zenoh + pos, zenohLen - pos, eidBytes);
            pos += eidBytes;
            oss << ":" << std::dec << eid;

            result.sourceIdHex = oss.str();

            // Source SN (VLE)
            size_t snBytes;
            result.sourceSn = static_cast<uint32_t>(
                DecodeVLE(zenoh + pos, zenohLen - pos, snBytes));
            pos += snBytes;

            if (debug)
            {
                std::cout << "[ZENOH] SourceInfo: id=" << result.sourceIdHex
                          << " sn=" << result.sourceSn << std::endl;
            }

            return result;  // Found SourceInfo, we're done
        }
        else
        {
            // Skip other extensions (assume ZBuf type with VLE length)
            size_t extLenBytes;
            uint64_t extLen = DecodeVLE(zenoh + pos, zenohLen - pos, extLenBytes);
            pos += extLenBytes + extLen;
        }

        if (!hasMore) break;
    }

    if (debug)
    {
        std::cout << "[ZENOH] SourceInfo extension not found" << std::endl;
    }

    return std::nullopt;  // SourceInfo not found
}

//==============================================================================
// Latency Measurement Tracker
//==============================================================================

struct LatencyRecord
{
    std::string messageKey;
    int64_t sendTimeNs;
    int64_t recvTimeNs;
    double latencyMs;
    uint16_t edgeNodeId;
    bool slaViolation;
};

/**
 * @brief Buffered CSV row for deferred file I/O
 *
 * Stores all fields needed to write one latency CSV row.
 * Rows are accumulated in memory during simulation and flushed
 * to disk in Close() to avoid blocking the real-time scheduler.
 */
struct CsvRow
{
    std::string sourceId;
    uint32_t sourceSn;
    int64_t sendTimeMs;
    int64_t recvTimeMs;
    double latencyMs;
    uint16_t edgeNodeId;
    bool slaViolation;
    bool handoverActive;
    std::string hopsStr;  // pre-formatted hop string (empty for non-hop API)
};

/**
 * @brief Singleton class to track Zenoh packet latencies
 *
 * Uses Source Info (source_id + source_sn) to track packets end-to-end,
 * even through external Zenoh routers.
 */
class ZenohLatencyTracker
{
public:
    static ZenohLatencyTracker& GetInstance()
    {
        static ZenohLatencyTracker instance;
        return instance;
    }

    void Initialize(const std::string& outputPath = "zenoh_latency.csv",
                    double slaThresholdMs = 50.0,
                    uint16_t sourceEdgeNodeId = 5)
    {
        m_outputPath = outputPath;
        m_slaThresholdMs = slaThresholdMs;
        m_sourceEdgeNodeId = sourceEdgeNodeId;
        m_initialized = true;
        m_debug = false;

        // CSV rows are buffered in memory and written in Close()
        m_csvBuffer.clear();

        m_totalPackets = 0;
        m_slaViolations = 0;
        m_sumLatency = 0.0;
        m_maxLatency = 0.0;
        m_minLatency = 1e9;
    }

    void SetDebug(bool debug) { m_debug = debug; }

    /**
     * @brief Record when a packet is sent into the 5G network
     * @param packet The packet being sent
     * @param edgeNodeId Edge node sending the packet
     * @return true if packet was tracked (had valid SourceInfo)
     */
    bool RecordSend(Ptr<const Packet> packet, uint16_t edgeNodeId)
    {
        if (!m_initialized) return false;

        auto srcInfo = ParseZenohSourceInfo(packet, m_debug);
        if (!srcInfo || !srcInfo->IsValid())
        {
            return false;
        }

        std::lock_guard<std::mutex> lock(m_mutex);

        std::string key = srcInfo->GetKey();
        SendRecord record;
        record.sendTimeNs = Simulator::Now().GetNanoSeconds();
        record.lastEdgeNodeId = edgeNodeId;
        record.sourceId = srcInfo->sourceIdHex;
        record.sourceSn = srcInfo->sourceSn;

        m_pendingSends[key] = record;

        if (m_debug)
        {
            std::cout << "[LATENCY] RecordSend: " << key
                      << " edge=" << edgeNodeId << std::endl;
        }

        return true;
    }

    /**
     * @brief Record when a packet is received from the 5G network
     * @param packet The packet received
     * @param handoverActive Whether handover is in progress
     * @return Latency in milliseconds, or -1 if not found
     */
    double RecordReceive(Ptr<const Packet> packet, bool handoverActive = false)
    {
        if (!m_initialized) return -1.0;

        auto srcInfo = ParseZenohSourceInfo(packet, m_debug);
        if (!srcInfo || !srcInfo->IsValid())
        {
            return -1.0;
        }

        std::lock_guard<std::mutex> lock(m_mutex);

        std::string key = srcInfo->GetKey();
        auto it = m_pendingSends.find(key);
        if (it == m_pendingSends.end())
        {
            if (m_debug)
            {
                std::cout << "[LATENCY] RecordReceive: " << key
                          << " NOT FOUND (pending=" << m_pendingSends.size() << ")" << std::endl;
            }
            return -1.0;
        }

        int64_t sendTimeNs = it->second.sendTimeNs;
        uint16_t edgeNodeId = it->second.lastEdgeNodeId;
        std::string sourceId = it->second.sourceId;
        uint32_t sourceSn = it->second.sourceSn;
        int64_t recvTimeNs = Simulator::Now().GetNanoSeconds();
        double latencyMs = (recvTimeNs - sendTimeNs) / 1e6;
        bool slaViolation = latencyMs > m_slaThresholdMs;

        m_totalPackets++;
        m_sumLatency += latencyMs;
        if (latencyMs > m_maxLatency) m_maxLatency = latencyMs;
        if (latencyMs < m_minLatency) m_minLatency = latencyMs;
        if (slaViolation) m_slaViolations++;

        m_csvBuffer.push_back({sourceId, sourceSn,
                               sendTimeNs / 1000000, recvTimeNs / 1000000,
                               latencyMs, edgeNodeId, slaViolation, handoverActive, ""});

        if (m_debug)
        {
            std::cout << "[LATENCY] RecordReceive: " << key
                      << " latency=" << latencyMs << "ms" << std::endl;
        }

        m_pendingSends.erase(it);
        return latencyMs;
    }

    // Sequence number based API (for payload pattern "[XXX]" extraction)
    // Special node IDs: 0xFFFF = Remote Host, 0xFFFE = UE
    static constexpr uint16_t NODE_REMOTE_HOST = 0xFFFF;
    static constexpr uint16_t NODE_UE = 0xFFFE;

    /**
     * @brief Record when a packet enters NS-3 at Remote Host (START point)
     * Only records the start time - no hop entry for Remote Host itself
     */
    void RecordSend(uint32_t zenohSn, uint16_t edgeNodeId)
    {
        if (!m_initialized) return;
        std::lock_guard<std::mutex> lock(m_mutex);
        std::string key = "seq:" + std::to_string(zenohSn);

        SendRecord record;
        record.sendTimeNs = Simulator::Now().GetNanoSeconds();
        record.lastEdgeNodeId = edgeNodeId;
        record.sourceId = "payload";
        record.sourceSn = zenohSn;
        // No hop entry for Remote Host - we only care about edge-to-edge timestamps

        m_pendingSends[key] = record;
    }

    /**
     * @brief Record when a packet arrives at an edge node (ingress)
     */
    void RecordHopIngress(uint32_t zenohSn, uint16_t nodeId)
    {
        if (!m_initialized) return;
        std::lock_guard<std::mutex> lock(m_mutex);
        std::string key = "seq:" + std::to_string(zenohSn);
        auto it = m_pendingSends.find(key);
        if (it == m_pendingSends.end()) return;

        int64_t now = Simulator::Now().GetNanoSeconds();
        it->second.hops.emplace_back(nodeId, now, 0);
    }

    /**
     * @brief Record when a packet leaves an edge node (egress to 5G/WAN)
     * If no ingress entry exists for this node, creates one with ingress=egress
     */
    void RecordHopEgress(uint32_t zenohSn, uint16_t nodeId)
    {
        if (!m_initialized) return;
        std::lock_guard<std::mutex> lock(m_mutex);
        std::string key = "seq:" + std::to_string(zenohSn);
        auto it = m_pendingSends.find(key);
        if (it == m_pendingSends.end()) return;

        int64_t now = Simulator::Now().GetNanoSeconds();
        // Find the hop for this node and update egress time
        for (auto& hop : it->second.hops)
        {
            if (hop.nodeId == nodeId && hop.egressTimeNs == 0)
            {
                hop.egressTimeNs = now;
                it->second.lastEdgeNodeId = nodeId;
                break;
            }
        }
    }

    /**
     * @brief Record when a packet arrives at UE (END point)
     * Only records the receive time - no hop entry for UE itself
     *
     * @param zenohSn Zenoh sequence number from payload
     * @param handoverActive Whether handover is in progress
     * @param bufferingDelayMs Additional buffering delay from handover blackout model (default 0)
     * @return Total latency in milliseconds including buffering delay
     */
    double RecordReceive(uint32_t zenohSn, bool handoverActive = false, double bufferingDelayMs = 0.0)
    {
        if (!m_initialized) return -1.0;
        std::lock_guard<std::mutex> lock(m_mutex);
        std::string key = "seq:" + std::to_string(zenohSn);
        auto it = m_pendingSends.find(key);
        if (it == m_pendingSends.end()) return -1.0;

        int64_t sendTimeNs = it->second.sendTimeNs;
        uint16_t edgeNodeId = it->second.lastEdgeNodeId;
        int64_t recvTimeNs = Simulator::Now().GetNanoSeconds();
        double baseLatencyMs = (recvTimeNs - sendTimeNs) / 1e6;

        // Total latency = base latency + handover buffering delay
        double latencyMs = baseLatencyMs + bufferingDelayMs;
        bool slaViolation = latencyMs > m_slaThresholdMs;

        // No hop entry for UE - we only care about edge-to-edge timestamps

        m_totalPackets++;
        m_sumLatency += latencyMs;
        if (latencyMs > m_maxLatency) m_maxLatency = latencyMs;
        if (latencyMs < m_minLatency) m_minLatency = latencyMs;
        if (slaViolation) m_slaViolations++;

        // Buffer CSV row - include hop details
        {
            // Build hop string: "nodeId:ingress:egress;nodeId:ingress:egress;..."
            std::ostringstream hopStr;
            for (size_t i = 0; i < it->second.hops.size(); ++i)
            {
                const auto& hop = it->second.hops[i];
                if (i > 0) hopStr << ";";
                hopStr << hop.nodeId << ":"
                       << (hop.ingressTimeNs / 1000000) << ":"
                       << (hop.egressTimeNs / 1000000);
            }

            m_csvBuffer.push_back({"payload", zenohSn,
                                   sendTimeNs / 1000000, recvTimeNs / 1000000,
                                   latencyMs, edgeNodeId, slaViolation, handoverActive,
                                   hopStr.str()});
        }

        m_pendingSends.erase(it);
        return latencyMs;
    }

    /**
     * @brief Get send time for a pending packet (for buffering delay calculation)
     * @param zenohSn Zenoh sequence number
     * @return Send time in nanoseconds, or -1 if not found
     */
    int64_t GetSendTimeNs(uint32_t zenohSn) const
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        std::string key = "seq:" + std::to_string(zenohSn);
        auto it = m_pendingSends.find(key);
        if (it == m_pendingSends.end()) return -1;
        return it->second.sendTimeNs;
    }

    bool IsFromSourceEdge(uint16_t edgeNodeId) const { return edgeNodeId == m_sourceEdgeNodeId; }
    uint16_t GetSourceEdgeNodeId() const { return m_sourceEdgeNodeId; }

    void PrintSummary()
    {
        if (!m_initialized || m_totalPackets == 0)
        {
            std::cout << "\n[ZENOH LATENCY] No packets measured\n";
            std::cout << "[ZENOH LATENCY] Pending (unmatched): " << m_pendingSends.size() << "\n";
            return;
        }

        double avgLatency = m_sumLatency / m_totalPackets;
        double violationRate = (100.0 * m_slaViolations) / m_totalPackets;

        // std::cout << "\n";
        // std::cout << "═══════════════════════════════════════════════════════════\n";
        // std::cout << "  ZENOH E2E LATENCY (Source Info based)\n";
        // std::cout << "═══════════════════════════════════════════════════════════\n";
        // std::cout << "  Source Edge Node:    " << m_sourceEdgeNodeId << "\n";
        // std::cout << "  SLA Threshold:       " << m_slaThresholdMs << " ms\n";
        // std::cout << "───────────────────────────────────────────────────────────\n";
        // std::cout << "  Total Packets:       " << m_totalPackets << "\n";
        // std::cout << "  SLA Violations:      " << m_slaViolations
        //           << " (" << std::fixed << std::setprecision(1) << violationRate << "%)\n";
        // std::cout << "  Pending (unmatched): " << m_pendingSends.size() << "\n";
        // std::cout << "───────────────────────────────────────────────────────────\n";
        // std::cout << "  Min Latency:         " << std::fixed << std::setprecision(3)
        //           << m_minLatency << " ms\n";
        // std::cout << "  Avg Latency:         " << avgLatency << " ms\n";
        // std::cout << "  Max Latency:         " << m_maxLatency << " ms\n";
        // std::cout << "═══════════════════════════════════════════════════════════\n";
        // std::cout << "  Output File:         " << m_outputPath << "\n";
        // std::cout << "═══════════════════════════════════════════════════════════\n\n";
    }

    void Close()
    {
        if (!m_initialized || m_outputPath.empty()) return;

        std::ofstream csvFile(m_outputPath);
        if (!csvFile.is_open()) return;

        csvFile << "source_id,source_sn,send_time_ms,recv_time_ms,"
                << "latency_ms,edge_node_id,sla_violation,handover_active,hops\n";

        for (const auto& row : m_csvBuffer)
        {
            csvFile << "\"" << row.sourceId << "\","
                    << row.sourceSn << ","
                    << row.sendTimeMs << ","
                    << row.recvTimeMs << ","
                    << row.latencyMs << ","
                    << row.edgeNodeId << ","
                    << (row.slaViolation ? 1 : 0) << ","
                    << (row.handoverActive ? 1 : 0);
            if (!row.hopsStr.empty())
            {
                csvFile << ",\"" << row.hopsStr << "\"";
            }
            csvFile << "\n";
        }

        csvFile.close();
    }

    uint64_t GetTotalPackets() const { return m_totalPackets; }
    uint64_t GetSlaViolations() const { return m_slaViolations; }
    double GetMaxLatency() const { return m_maxLatency; }
    double GetMinLatency() const { return m_minLatency < 1e9 ? m_minLatency : 0; }
    double GetAvgLatency() const { return m_totalPackets > 0 ? m_sumLatency / m_totalPackets : 0; }
    uint64_t GetPendingPackets() const { return m_pendingSends.size(); }

    /**
     * @brief Get details of lost packets (packets still pending at end of simulation)
     * @return Vector of (sendTimeMs, sourceSn) pairs sorted by send time
     */
    std::vector<std::pair<double, uint32_t>> GetLostPacketDetails() const
    {
        std::vector<std::pair<double, uint32_t>> lostPackets;
        std::lock_guard<std::mutex> lock(m_mutex);
        for (const auto& entry : m_pendingSends)
        {
            double sendTimeMs = entry.second.sendTimeNs / 1e6;
            lostPackets.emplace_back(sendTimeMs, entry.second.sourceSn);
        }
        std::sort(lostPackets.begin(), lostPackets.end());
        return lostPackets;
    }

private:
    ZenohLatencyTracker() = default;
    ~ZenohLatencyTracker() { Close(); }
    ZenohLatencyTracker(const ZenohLatencyTracker&) = delete;
    ZenohLatencyTracker& operator=(const ZenohLatencyTracker&) = delete;

    // Per-hop timestamp record (NDN-like structure)
    struct HopTimestamp
    {
        uint16_t nodeId;       // Node ID (edge server ID or special: 0xFFFF=remote, 0xFFFE=ue)
        int64_t ingressTimeNs; // When packet arrived at this node
        int64_t egressTimeNs;  // When packet left this node (0 if not yet)

        HopTimestamp(uint16_t id = 0, int64_t ingress = 0, int64_t egress = 0)
            : nodeId(id), ingressTimeNs(ingress), egressTimeNs(egress) {}
    };

    struct SendRecord
    {
        int64_t sendTimeNs;           // START: packet enters NS-3 (Remote Host)
        std::vector<HopTimestamp> hops;  // Timestamps at each hop along the path
        uint16_t lastEdgeNodeId;      // Edge node that sent to UE (for backward compat)
        std::string sourceId;
        uint32_t sourceSn;

        SendRecord() : sendTimeNs(0), lastEdgeNodeId(0), sourceSn(0) {}
    };

    bool m_initialized = false;
    bool m_debug = false;
    std::string m_outputPath;
    double m_slaThresholdMs = 50.0;
    uint16_t m_sourceEdgeNodeId = 5;

    std::map<std::string, SendRecord> m_pendingSends;
    mutable std::mutex m_mutex;
    std::vector<CsvRow> m_csvBuffer;

    uint64_t m_totalPackets = 0;
    uint64_t m_slaViolations = 0;
    double m_sumLatency = 0.0;
    double m_maxLatency = 0.0;
    double m_minLatency = 1e9;
};

//==============================================================================
// Backward Compatibility
//==============================================================================

struct ZenohFrameInfo
{
    uint32_t sequenceNumber;
    bool isReliable;
    size_t headerSize;
};

inline std::optional<ZenohFrameInfo> ExtractZenohFrameInfo(Ptr<const Packet> packet, bool debug = false)
{
    auto srcInfo = ParseZenohSourceInfo(packet, debug);
    if (!srcInfo) return std::nullopt;

    return ZenohFrameInfo{
        .sequenceNumber = srcInfo->transportSn,
        .isReliable = srcInfo->isReliable,
        .headerSize = 0
    };
}

//==============================================================================
// Simple Payload-Based Sequence Extractor
//==============================================================================

/**
 * @brief Extract sequence number from Zenoh payload pattern "[XXXX]"
 *
 * This is a simple, robust alternative to protocol-level parsing.
 * It scans the raw packet bytes for the pattern produced by z_pub.rs:
 *   format!("[{idx:4}] {payload}")  ->  "[   0] Pub from Rust!"
 *
 * Advantages:
 * - Works regardless of Zenoh protocol version
 * - Survives any number of router hops (payload is preserved)
 * - Minimal parsing overhead
 * - No dependency on SourceInfo extension being present
 *
 * @param packet The packet to scan
 * @return Sequence number if pattern found, nullopt otherwise
 */
inline std::optional<uint32_t> ExtractZenohPayloadSeq(Ptr<const Packet> packet, bool debug = false)
{
    uint8_t buffer[512];
    uint32_t copied = packet->CopyData(buffer, sizeof(buffer));

    if (debug && copied > 100)  // Only debug larger packets (likely contain payload)
    {
        NS_LOG_UNCOND("[ZENOH_SEQ] Scanning " << copied << " bytes, last 50: ");
        for (uint32_t i = (copied > 50 ? copied - 50 : 0); i < copied; i++)
        {
            if (buffer[i] >= 32 && buffer[i] < 127)
                std::cout << (char)buffer[i];
            else
                std::cout << ".";
        }
        std::cout << std::endl;
    }

    // Scan for '[' followed by optional spaces, digits, and ']'
    for (uint32_t i = 0; i + 6 < copied; i++)
    {
        if (buffer[i] == '[')
        {
            uint32_t seq = 0;
            uint32_t j = i + 1;
            bool hasDigits = false;

            // Skip spaces, parse digits until ']'
            while (j < copied && j < i + 12)
            {
                if (buffer[j] >= '0' && buffer[j] <= '9')
                {
                    seq = seq * 10 + (buffer[j] - '0');
                    hasDigits = true;
                }
                else if (buffer[j] == ']' && hasDigits)
                {
                    if (debug)
                    {
                        NS_LOG_UNCOND("[ZENOH_SEQ] Found seq=" << seq << " at offset " << i);
                    }
                    return seq;  // Found valid "[XXX]" pattern
                }
                else if (buffer[j] != ' ')
                {
                    break;  // Invalid character, try next position
                }
                j++;
            }
        }
    }
    return std::nullopt;
}

}  // namespace ns3

#endif  // ZENOH_LATENCY_MEASUREMENT_H
