// Copyright (c) 2024
// SPDX-License-Identifier: GPL-2.0-only

/**
 * @ingroup examples
 * @file nr-baseline-latency.cc
 * @brief Baseline 5G NR latency measurement for remote driving scenario
 *
 * This example measures end-to-end downlink latency for remote driving:
 * - Downlink: Control commands from Remote Host (cloud) to moving UE (vehicle)
 *
 * Features:
 * - Per-packet latency measurement with CSV output
 * - Hexagonal grid topology with handover support
 * - Configurable mobility (linear or random waypoint)
 * - REALISTIC HANDOVER INTERRUPTION MODEL
 * - FlowMonitor for aggregate statistics
 * - No external dependencies (pure NS-3 simulation)
 *
 * Handover Interruption Model:
 *   Real 5G handover causes 30-100ms interruption where packets are buffered.
 *   NS-3's ideal handover completes in ~2ms. This example adds configurable
 *   artificial interruption (--handoverInterruptMs) to model realistic behavior.
 *
 *   During handover window:
 *   - Packets are "buffered" at source gNB
 *   - After handover, buffered packets are delivered with additional delay
 *   - Latency = normal_latency + time_spent_in_buffer
 *
 * Network Topology:
 *
 *   Remote Host (Cloud Controller)
 *          |
 *       P2P Link (configurable WAN delay)
 *          |
 *         PGW
 *          |
 *       S1-U Link
 *          |
 *        gNB(s) ─── Hexagonal Grid
 *          |
 *         UE (Vehicle, moving)
 */

#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/nr-module.h"
#include "ns3/ns2-mobility-helper.h"
#include "ns3/point-to-point-module.h"

#include "builtin-mobility-paths.h"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <map>
#include <sys/stat.h>
#include <cerrno>
#include <ctime>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NrBaselineLatency");

//==============================================================================
// Global State for Latency Tracking
//==============================================================================

// CSV output stream
std::ofstream g_latencyCsv;

// Current serving cell per UE (IMSI -> CellId)
std::map<uint64_t, uint16_t> g_ueServingCell;

// Handover active flag per UE (IMSI -> bool)
std::map<uint64_t, bool> g_ueHandoverActive;

// UE IMSI (for single-UE scenario)
uint64_t g_ueImsi = 0;

// Statistics counters
uint64_t g_packetCount = 0;
double g_latencySum = 0.0;
double g_minLatency = std::numeric_limits<double>::max();
double g_maxLatency = 0.0;
uint64_t g_handoverPacketCount = 0;

// SLA tracking
double g_slaThresholdMs = 50.0;    // SLA threshold in ms (default: 50ms for remote driving)
uint64_t g_slaViolationCount = 0;  // Count of packets exceeding SLA threshold

// Handover interruption model parameters
double g_handoverInterruptMs = 50.0;  // Total handover interruption time (ms)
double g_networkDelayMs = 21.0;       // WAN + S1U delay (for calculating arrival at gNB)
std::map<uint64_t, Time> g_handoverStartTime;  // IMSI -> actual handover start time
std::map<uint64_t, Time> g_handoverEndTime;    // IMSI -> handover end time (start + interrupt)

// Trajectory logging
std::ofstream g_trajectoryFile;
bool g_trajectoryLoggingEnabled = false;

// Per-packet send tracking for packet loss detection
std::map<uint32_t, int64_t> g_pendingSendPackets;  // seq -> sendTimeNs
uint64_t g_totalPacketsSent = 0;

// Handover event tracking
struct HandoverEvent
{
    Time startTime;
    Time endTime;
    uint64_t imsi;
    uint16_t sourceCell;
    uint16_t targetCell;
    bool success;
};

std::vector<HandoverEvent> g_handoverEvents;

//==============================================================================
// Callback Functions
//==============================================================================

/**
 * @brief Callback when UE receives a downlink packet
 *
 * This is the main latency measurement callback. It extracts the timestamp
 * from SeqTsSizeHeader (set at Remote Host) and calculates one-way latency.
 *
 * For packets sent during the handover window, we add artificial buffering
 * delay to model realistic behavior where packets queue at source gNB.
 */
void
DlRxCallback(Ptr<const Packet> packet,
             const Address& from,
             const Address& to,
             const SeqTsSizeHeader& header)
{
    Time now = Simulator::Now();
    Time sendTime = header.GetTs();
    Time baseLatency = now - sendTime;
    double bufferingDelayMs = 0.0;
    bool duringHandover = false;

    // Calculate when packet arrives at source gNB (sendTime + network delay)
    Time arrivalAtGnb = sendTime + MilliSeconds(g_networkDelayMs);

    // Check if packet ARRIVES at gNB during handover window
    // This correctly handles in-flight packets sent before handover started
    if (g_handoverStartTime.count(g_ueImsi) > 0 && g_handoverEndTime.count(g_ueImsi) > 0)
    {
        Time hoStart = g_handoverStartTime[g_ueImsi];
        Time hoEnd = g_handoverEndTime[g_ueImsi];

        if (arrivalAtGnb >= hoStart && arrivalAtGnb <= hoEnd)
        {
            // Packet arrives at gNB during handover -> buffered
            duringHandover = true;

            // Buffering delay = time from arrival at gNB until handover completes
            // (packet waits at source gNB until handover is done, then forwarded)
            Time timeInBuffer = hoEnd - arrivalAtGnb;
            bufferingDelayMs = timeInBuffer.GetSeconds() * 1000.0;
        }
    }

    // Also check if handover is currently active (for real-time tracking)
    if (!duringHandover && g_ueHandoverActive.count(g_ueImsi) && g_ueHandoverActive[g_ueImsi])
    {
        duringHandover = true;
    }

    // Total latency = base latency + buffering delay
    double totalLatencyMs = baseLatency.GetSeconds() * 1000.0 + bufferingDelayMs;

    uint16_t cellId = g_ueServingCell.count(g_ueImsi) ? g_ueServingCell[g_ueImsi] : 0;

    // Check SLA violation (needed for CSV before statistics update)
    bool slaViolation = totalLatencyMs > g_slaThresholdMs;

    // Write to CSV
    g_latencyCsv << std::fixed << std::setprecision(6)
                 << now.GetSeconds() << ","
                 << header.GetSeq() << ","
                 << totalLatencyMs << ","
                 << header.GetSize() << ","
                 << cellId << ","
                 << (duringHandover ? 1 : 0) << ","
                 << bufferingDelayMs << ","
                 << (slaViolation ? 1 : 0) << "\n";

    // Update statistics
    g_packetCount++;
    g_pendingSendPackets.erase(header.GetSeq());
    g_latencySum += totalLatencyMs;
    g_minLatency = std::min(g_minLatency, totalLatencyMs);
    g_maxLatency = std::max(g_maxLatency, totalLatencyMs);
    if (slaViolation)
    {
        g_slaViolationCount++;
    }

    if (duringHandover)
    {
        g_handoverPacketCount++;
    }

    // Debug output for first few packets, periodic, handover, and SLA violations
    if (g_packetCount <= 5 || g_packetCount % 100 == 0 || duringHandover || slaViolation)
    {
        NS_LOG_INFO(now.GetSeconds() << "s [RX] seq=" << header.GetSeq()
                    << " latency=" << totalLatencyMs << "ms"
                    << (bufferingDelayMs > 0 ? " (buffered=" + std::to_string(bufferingDelayMs) + "ms)" : "")
                    << " cell=" << cellId
                    << (duringHandover ? " [HANDOVER]" : "")
                    << (slaViolation ? " [SLA VIOLATION]" : ""));
    }
}

/**
 * @brief Callback when a downlink packet is sent from Remote Host
 *
 * Records each packet's sequence number and send timestamp into
 * g_pendingSendPackets for later reconciliation with received packets.
 */
void
DlTxCallback(Ptr<const Packet> packet,
             const Address& from,
             const Address& to,
             const SeqTsSizeHeader& header)
{
    uint32_t seq = header.GetSeq();
    int64_t sendTimeNs = header.GetTs().GetNanoSeconds();
    g_pendingSendPackets[seq] = sendTimeNs;
    g_totalPacketsSent++;
}

/**
 * @brief Callback when handover starts at the UE
 *
 * Sets up the handover window for realistic interruption modeling.
 *
 * The window accounts for:
 * 1. In-flight packets: sent before handover but still in transit (WAN + radio delay)
 * 2. Buffered packets: sent during handover, queued at source gNB
 *
 * Window: [now - inFlightTime, now + handoverInterruptMs]
 * where inFlightTime ≈ WAN delay (packets already at gNB when handover starts)
 */
void
HandoverStartCallback(std::string path,
                      uint64_t imsi,
                      uint16_t sourceCellId,
                      uint16_t rnti,
                      uint16_t targetCellId)
{
    Time now = Simulator::Now();
    g_ueHandoverActive[imsi] = true;

    // Set handover window: [now, now + handoverInterruptMs]
    // Packets that ARRIVE at gNB during this window are buffered
    // (arrival time = sendTime + networkDelay, checked in DlRxCallback)
    g_handoverStartTime[imsi] = now;
    g_handoverEndTime[imsi] = now + MilliSeconds(g_handoverInterruptMs);

    NS_LOG_UNCOND(now.GetSeconds()
                  << "s [HANDOVER START] IMSI=" << imsi
                  << " Cell " << sourceCellId << " -> " << targetCellId
                  << " (interrupt: " << g_handoverInterruptMs << "ms)");

    // Record handover event
    HandoverEvent event;
    event.startTime = now;
    event.endTime = Time(0);
    event.imsi = imsi;
    event.sourceCell = sourceCellId;
    event.targetCell = targetCellId;
    event.success = false;
    g_handoverEvents.push_back(event);
}

/**
 * @brief Callback when handover completes successfully
 */
void
HandoverEndOkCallback(std::string path,
                      uint64_t imsi,
                      uint16_t cellId,
                      uint16_t rnti)
{
    g_ueHandoverActive[imsi] = false;
    g_ueServingCell[imsi] = cellId;

    // Complete the handover event
    for (auto it = g_handoverEvents.rbegin(); it != g_handoverEvents.rend(); ++it)
    {
        if (it->imsi == imsi && !it->success && it->endTime == Time(0))
        {
            it->endTime = Simulator::Now();
            it->success = true;
            break;
        }
    }

    NS_LOG_UNCOND(Simulator::Now().GetSeconds()
                  << "s [HANDOVER SUCCESS] IMSI=" << imsi
                  << " NewCell=" << cellId);
}

/**
 * @brief Callback when handover fails
 */
void
HandoverEndErrorCallback(std::string path,
                         uint64_t imsi,
                         uint16_t cellId,
                         uint16_t rnti)
{
    g_ueHandoverActive[imsi] = false;

    // Mark the handover event as failed
    for (auto it = g_handoverEvents.rbegin(); it != g_handoverEvents.rend(); ++it)
    {
        if (it->imsi == imsi && !it->success && it->endTime == Time(0))
        {
            it->endTime = Simulator::Now();
            break;
        }
    }

    NS_LOG_UNCOND(Simulator::Now().GetSeconds()
                  << "s [HANDOVER FAILED] IMSI=" << imsi
                  << " CellId=" << cellId);
}

/**
 * @brief Print UE position periodically
 */
void
PrintUePosition(Ptr<Node> ueNode)
{
    // Try to get the most specific mobility model
    Ptr<MobilityModel> mobility = ueNode->GetObject<GaussMarkovMobilityModel>();
    if (!mobility)
    {
        mobility = ueNode->GetObject<WaypointMobilityModel>();
    }
    if (!mobility)
    {
        mobility = ueNode->GetObject<MobilityModel>();
    }
    if (!mobility)
    {
        NS_LOG_UNCOND(Simulator::Now().GetSeconds() << "s [UE] No mobility model found");
        Simulator::Schedule(Seconds(10.0), &PrintUePosition, ueNode);
        return;
    }

    Vector pos = mobility->GetPosition();
    Vector vel = mobility->GetVelocity();

    uint16_t cellId = g_ueServingCell.count(g_ueImsi) ? g_ueServingCell[g_ueImsi] : 0;

    NS_LOG_UNCOND(Simulator::Now().GetSeconds()
                  << "s [UE] pos=(" << pos.x << ", " << pos.y << ")"
                  << " vel=" << vel.GetLength() << " m/s"
                  << " cell=" << cellId);

    Simulator::Schedule(Seconds(10.0), &PrintUePosition, ueNode);
}

//==============================================================================
// Helper Functions
//==============================================================================

/**
 * @brief Build NS-3 random variable string for Normal velocity distribution
 * @param mean Mean velocity in m/s
 * @param variance Variance of velocity (m/s)^2
 * @return String for NS-3 NormalRandomVariable attribute
 */
std::string
BuildNormalVelocityString(double mean, double variance)
{
    std::ostringstream oss;
    oss << "ns3::NormalRandomVariable[Mean=" << mean
        << "|Variance=" << variance
        << "|Bound=" << (3.0 * std::sqrt(variance)) << "]";
    return oss.str();
}

/**
 * @brief Write UE trajectory to CSV file
 * @param ueNode UE node to log
 * @param interval Logging interval in seconds
 */
void
LogUeTrajectory(Ptr<Node> ueNode, double interval)
{
    if (!g_trajectoryLoggingEnabled)
    {
        return;
    }

    // Try to get the most specific mobility model
    Ptr<MobilityModel> mobility = ueNode->GetObject<GaussMarkovMobilityModel>();
    if (!mobility)
    {
        mobility = ueNode->GetObject<WaypointMobilityModel>();
    }
    if (!mobility)
    {
        mobility = ueNode->GetObject<MobilityModel>();
    }
    if (!mobility)
    {
        Simulator::Schedule(Seconds(interval), &LogUeTrajectory, ueNode, interval);
        return;
    }

    Vector pos = mobility->GetPosition();
    Vector vel = mobility->GetVelocity();
    double speed = vel.GetLength();

    uint16_t cellId = g_ueServingCell.count(g_ueImsi) ? g_ueServingCell[g_ueImsi] : 0;

    g_trajectoryFile << std::fixed << std::setprecision(3)
                     << Simulator::Now().GetSeconds() << ","
                     << g_ueImsi << ","
                     << pos.x << "," << pos.y << "," << pos.z << ","
                     << vel.x << "," << vel.y << "," << vel.z << ","
                     << speed << ","
                     << cellId << "\n";

    // Flush to ensure data is written even if simulation crashes
    g_trajectoryFile.flush();

    // Schedule next logging
    Simulator::Schedule(Seconds(interval), &LogUeTrajectory, ueNode, interval);
}

//==============================================================================
// Experiment Directory Helpers
//==============================================================================

/**
 * @brief Generate a timestamped experiment directory name
 * @return Directory path in format "experiment_data/YYYY-MM-DD_HH-MM-SS"
 */
std::string
GenerateExperimentDir()
{
    time_t now = time(nullptr);
    struct tm* localTime = localtime(&now);
    char buffer[64];
    strftime(buffer, sizeof(buffer), "%Y-%m-%d_%H-%M-%S", localTime);
    return "experiment_data/" + std::string(buffer);
}

/**
 * @brief Create a directory if it doesn't exist
 * @param path Directory path to create
 * @return true if directory exists or was created successfully
 */
bool
CreateDirectoryIfNeeded(const std::string& path)
{
    return mkdir(path.c_str(), 0755) == 0 || errno == EEXIST;
}

/**
 * @brief Write experiment configuration to a file
 * @param outputPath Path for the config file
 */
void
WriteExperimentConfig(const std::string& outputPath,
                      uint8_t numRings,
                      const std::string& scenario,
                      double isd,
                      const std::string& mobilityModel,
                      double ueSpeed,
                      double ueSpeedVariance,
                      double gaussAlpha,
                      double gaussTimeStep,
                      const std::string& waypointFile,
                      const std::string& builtinPath,
                      double simTime,
                      double centralFrequency,
                      double bandwidth,
                      double gnbTxPower,
                      uint16_t numerology,
                      double handoverInterruptMs,
                      double wanDelayMs,
                      double s1uDelayMs,
                      double slaThresholdMs,
                      uint32_t packetSize,
                      double intervalMs,
                      double trajectoryLogInterval,
                      uint32_t numBackgroundUes,
                      double bgUeTrafficMbps,
                      uint32_t bgUePacketSize,
                      double loadPercent,
                      double capacityMbps)
{
    std::ofstream configFile(outputPath);
    if (!configFile.is_open())
    {
        NS_LOG_WARN("Failed to open config file: " << outputPath);
        return;
    }

    time_t now = time(nullptr);
    char timeBuffer[64];
    strftime(timeBuffer, sizeof(timeBuffer), "%Y-%m-%d %H:%M:%S", localtime(&now));

    configFile << "# Experiment Configuration\n";
    configFile << "# Generated: " << timeBuffer << "\n\n";

    configFile << std::fixed;

    configFile << "[TOPOLOGY]\n";
    configFile << "num_rings=" << (int)numRings << "\n";
    configFile << "scenario=" << scenario << "\n";
    configFile << std::setprecision(1) << "isd_m=" << isd << "\n";

    configFile << "\n[MOBILITY]\n";
    configFile << "mobility_model=" << mobilityModel << "\n";
    configFile << std::setprecision(2);
    configFile << "ue_speed_mps=" << ueSpeed << "\n";
    configFile << "ue_speed_variance=" << ueSpeedVariance << "\n";
    configFile << "gauss_alpha=" << gaussAlpha << "\n";
    configFile << "gauss_time_step_s=" << gaussTimeStep << "\n";
    if (!waypointFile.empty())
    {
        configFile << "waypoint_file=" << waypointFile << "\n";
    }
    configFile << "builtin_path=" << builtinPath << "\n";
    configFile << std::setprecision(1) << "sim_time_s=" << simTime << "\n";

    configFile << "\n[NR]\n";
    configFile << std::setprecision(0);
    configFile << "central_frequency_hz=" << centralFrequency << "\n";
    configFile << "bandwidth_hz=" << bandwidth << "\n";
    configFile << std::setprecision(1);
    configFile << "gnb_tx_power_dbm=" << gnbTxPower << "\n";
    configFile << "numerology=" << numerology << "\n";

    configFile << "\n[HANDOVER]\n";
    configFile << "handover_interrupt_ms=" << handoverInterruptMs << "\n";
    configFile << "wan_delay_ms=" << wanDelayMs << "\n";
    configFile << "s1u_delay_ms=" << s1uDelayMs << "\n";

    configFile << "\n[TRAFFIC]\n";
    configFile << "packet_size_bytes=" << packetSize << "\n";
    configFile << std::setprecision(1) << "interval_ms=" << intervalMs << "\n";

    configFile << "\n[MEASUREMENT]\n";
    configFile << "sla_threshold_ms=" << slaThresholdMs << "\n";
    configFile << std::setprecision(2) << "trajectory_log_interval_s=" << trajectoryLogInterval << "\n";

    configFile << "\n[BACKGROUND_LOAD]\n";
    configFile << "num_background_ues=" << numBackgroundUes << "\n";
    configFile << std::setprecision(1);
    configFile << "bg_ue_traffic_mbps=" << bgUeTrafficMbps << "\n";
    configFile << "bg_ue_packet_size=" << bgUePacketSize << "\n";
    configFile << "load_percent=" << loadPercent << "\n";
    configFile << "capacity_mbps=" << capacityMbps << "\n";

    configFile.close();
    NS_LOG_INFO("Experiment config written to: " << outputPath);
}

/**
 * @brief Write simulation results summary to a file
 * @param outputPath Path for the results file
 * @param simTime Simulation time in seconds
 * @param slaThresholdMs SLA threshold in milliseconds
 */
void
WriteSimulationResults(const std::string& outputPath, double simTime, double slaThresholdMs)
{
    uint64_t receivedPackets = g_packetCount;
    uint64_t slaViolations = g_slaViolationCount;
    double avgLatency = receivedPackets > 0 ? g_latencySum / receivedPackets : 0.0;
    double minLatency = receivedPackets > 0 ? g_minLatency : 0.0;
    double maxLatency = g_maxLatency;

    // Packets still in pending map at simulation end are considered lost
    uint64_t lostPackets = g_pendingSendPackets.size();
    uint64_t totalPacketsSent = g_totalPacketsSent;
    double packetLossRate = totalPacketsSent > 0 ?
        (100.0 * lostPackets / totalPacketsSent) : 0.0;

    // Handover statistics
    uint32_t totalHandovers = g_handoverEvents.size();
    uint32_t successfulHandovers = 0;
    double totalHandoverDuration = 0.0;
    double minHandoverDuration = 1e9;
    double maxHandoverDuration = 0.0;

    for (const auto& event : g_handoverEvents)
    {
        if (event.success)
        {
            successfulHandovers++;
            double durationMs = (event.endTime - event.startTime).GetMilliSeconds();
            totalHandoverDuration += durationMs;
            if (durationMs < minHandoverDuration) minHandoverDuration = durationMs;
            if (durationMs > maxHandoverDuration) maxHandoverDuration = durationMs;
        }
    }

    double avgHandoverDuration = successfulHandovers > 0 ?
        totalHandoverDuration / successfulHandovers : 0.0;
    double handoverSuccessRate = totalHandovers > 0 ?
        (100.0 * successfulHandovers / totalHandovers) : 100.0;
    double slaViolationRate = receivedPackets > 0 ?
        (100.0 * slaViolations / receivedPackets) : 0.0;

    // Write results to file
    std::ofstream resultsFile(outputPath);
    if (!resultsFile.is_open())
    {
        NS_LOG_WARN("Failed to open results file: " << outputPath);
        return;
    }

    resultsFile << "# Simulation Results Summary\n";
    resultsFile << "# Generated at simulation end\n";
    resultsFile << "\n";

    resultsFile << "[METRICS]\n";
    resultsFile << "metric,value\n";
    resultsFile << "simulation_time_s," << simTime << "\n";
    resultsFile << "sla_threshold_ms," << slaThresholdMs << "\n";
    resultsFile << "\n";

    resultsFile << "[LATENCY]\n";
    resultsFile << "metric,value\n";
    resultsFile << "total_packets_sent," << totalPacketsSent << "\n";
    resultsFile << "total_packets_received," << receivedPackets << "\n";
    resultsFile << "packets_lost," << lostPackets << "\n";
    resultsFile << "packet_loss_rate_percent," << std::fixed << std::setprecision(2) << packetLossRate << "\n";
    resultsFile << "min_latency_ms," << std::fixed << std::setprecision(3) << minLatency << "\n";
    resultsFile << "avg_latency_ms," << std::fixed << std::setprecision(3) << avgLatency << "\n";
    resultsFile << "max_latency_ms," << std::fixed << std::setprecision(3) << maxLatency << "\n";
    resultsFile << "sla_violations," << slaViolations << "\n";
    resultsFile << "sla_violation_rate_percent," << std::fixed << std::setprecision(2) << slaViolationRate << "\n";
    resultsFile << "packets_during_handover," << g_handoverPacketCount << "\n";
    resultsFile << "\n";

    resultsFile << "[HANDOVER]\n";
    resultsFile << "metric,value\n";
    resultsFile << "total_handovers," << totalHandovers << "\n";
    resultsFile << "successful_handovers," << successfulHandovers << "\n";
    resultsFile << "failed_handovers," << (totalHandovers - successfulHandovers) << "\n";
    resultsFile << "handover_success_rate_percent," << std::fixed << std::setprecision(2) << handoverSuccessRate << "\n";
    if (successfulHandovers > 0)
    {
        resultsFile << "avg_handover_duration_ms," << std::fixed << std::setprecision(3) << avgHandoverDuration << "\n";
        resultsFile << "min_handover_duration_ms," << std::fixed << std::setprecision(3) << minHandoverDuration << "\n";
        resultsFile << "max_handover_duration_ms," << std::fixed << std::setprecision(3) << maxHandoverDuration << "\n";
    }
    resultsFile << "\n";

    // Detailed handover events
    resultsFile << "[HANDOVER_EVENTS]\n";
    resultsFile << "start_time_s,end_time_s,duration_ms,imsi,source_cell,target_cell,success\n";
    for (const auto& event : g_handoverEvents)
    {
        double durationMs = event.success ?
            (event.endTime - event.startTime).GetMilliSeconds() : -1.0;
        resultsFile << std::fixed << std::setprecision(3)
                    << event.startTime.GetSeconds() << ","
                    << (event.success ? event.endTime.GetSeconds() : -1.0) << ","
                    << durationMs << ","
                    << event.imsi << ","
                    << event.sourceCell << ","
                    << event.targetCell << ","
                    << (event.success ? 1 : 0) << "\n";
    }
    resultsFile << "\n";

    // Lost packets detail
    resultsFile << "[LOST_PACKETS]\n";
    resultsFile << "seq,send_timestamp_ms\n";
    for (const auto& entry : g_pendingSendPackets)
    {
        resultsFile << entry.first << ","
                    << std::fixed << std::setprecision(3)
                    << (entry.second / 1e6) << "\n";
    }
    resultsFile << "total,," << lostPackets << "\n";

    resultsFile.close();

    // Print summary to console
    NS_LOG_UNCOND("\n═══════════════════════════════════════════════════════════");
    NS_LOG_UNCOND("  SIMULATION RESULTS SUMMARY");
    NS_LOG_UNCOND("═══════════════════════════════════════════════════════════");
    NS_LOG_UNCOND("  Simulation Time:     " << simTime << " s");
    NS_LOG_UNCOND("  SLA Threshold:       " << slaThresholdMs << " ms");
    NS_LOG_UNCOND("───────────────────────────────────────────────────────────");
    NS_LOG_UNCOND("  PACKET METRICS:");
    NS_LOG_UNCOND("    Packets Sent:      " << totalPacketsSent);
    NS_LOG_UNCOND("    Packets Received:  " << receivedPackets);
    NS_LOG_UNCOND("    Packets Lost:      " << lostPackets << " (" << std::fixed << std::setprecision(2) << packetLossRate << "%)");
    NS_LOG_UNCOND("───────────────────────────────────────────────────────────");
    NS_LOG_UNCOND("  LATENCY METRICS:");
    NS_LOG_UNCOND("    Min Latency:       " << std::fixed << std::setprecision(3) << minLatency << " ms");
    NS_LOG_UNCOND("    Average Latency:   " << std::fixed << std::setprecision(3) << avgLatency << " ms");
    NS_LOG_UNCOND("    Max Latency:       " << std::fixed << std::setprecision(3) << maxLatency << " ms");
    NS_LOG_UNCOND("    SLA Violations:    " << slaViolations << " (" << std::fixed << std::setprecision(2) << slaViolationRate << "%)");
    NS_LOG_UNCOND("───────────────────────────────────────────────────────────");
    NS_LOG_UNCOND("  HANDOVER METRICS:");
    NS_LOG_UNCOND("    Total Handovers:   " << totalHandovers);
    NS_LOG_UNCOND("    Successful:        " << successfulHandovers << " (" << std::fixed << std::setprecision(2) << handoverSuccessRate << "%)");
    if (successfulHandovers > 0)
    {
        NS_LOG_UNCOND("    Avg HO Duration:   " << std::fixed << std::setprecision(3) << avgHandoverDuration << " ms");
        NS_LOG_UNCOND("    Min HO Duration:   " << std::fixed << std::setprecision(3) << minHandoverDuration << " ms");
        NS_LOG_UNCOND("    Max HO Duration:   " << std::fixed << std::setprecision(3) << maxHandoverDuration << " ms");
    }
    NS_LOG_UNCOND("═══════════════════════════════════════════════════════════");
    NS_LOG_UNCOND("  Results saved to:    " << outputPath);
    NS_LOG_UNCOND("═══════════════════════════════════════════════════════════\n");
}

//==============================================================================
// Main Function
//==============================================================================

int
main(int argc, char* argv[])
{
    //--------------------------------------------------------------------------
    // Default parameters
    //--------------------------------------------------------------------------

    // Topology parameters
    uint8_t numRings = 1;                    // Number of hexagonal rings (0=1 site, 1=7 sites)
    std::string scenario = "UMa";            // Propagation scenario (UMa, RMa, UMi)
    double isd = 500.0;                      // Inter-site distance in meters

    // Mobility parameters
    double ueSpeed = 30.0;                   // UE speed in m/s (108 km/h)
    double ueSpeedVariance = 2.0;            // Variance for Gaussian velocity (m/s)^2
    double gaussAlpha = 0.85;                // Gauss-Markov memory factor (0=random, 1=linear)
    double gaussTimeStep = 0.5;              // Gauss-Markov update interval (seconds)
    std::string waypointFile = "";           // NS-2 format trace file (empty = use built-in path)
    std::string builtinPath = "linear-y";    // Built-in paths: hexagonal, linear-y, zigzag, highway, urban-grid
    double simTime = 60.0;                   // Simulation time in seconds
    std::string mobilityModel = "linear";    // linear, random, gauss-markov, waypoint

    // NR parameters
    double centralFrequency = 3.5e9;         // 3.5 GHz (n78 band)
    double bandwidth = 20e6;                 // 20 MHz
    double gnbTxPower = 43.0;                // gNB TX power in dBm
    uint16_t numerology = 1;                 // NR numerology

    // Traffic parameters (DL control commands only)
    uint32_t packetSize = 100;               // Control packet size in bytes
    double intervalMs = 50.0;                // Packet interval in ms (20 Hz)

    // Background UE parameters for cell load
    uint32_t numBackgroundUes = 0;           // Number of stationary background UEs (0 = disabled)
    double bgUeTrafficMbps = 5.0;            // Traffic rate per background UE in Mbps
    uint32_t bgUePacketSize = 500;           // Background UE packet size in bytes

    // Capacity testing parameters
    double loadPercent = 0.0;                // Target load as % of capacity (0 = use numBackgroundUes directly)
    double capacityMbps = 0.0;               // Per-cell capacity in Mbps (0 = auto-estimate based on bandwidth)
    bool saturationTest = false;             // Run saturation test to find max capacity

    // Network delays
    double wanDelayMs = 20.0;                // WAN delay in ms (one-way, central cloud)
    double s1uDelayMs = 1.0;                 // S1-U link delay in ms

    // Handover interruption model
    double handoverInterruptMs = 30.0;       // Handover interruption time in ms (realistic: 30-100ms)

    // SLA parameters
    double slaThresholdMs = 50.0;            // SLA threshold in ms (remote driving: 50ms)

    // Experiment output directory
    std::string experimentDir = "";              // Empty = auto-generate timestamped directory

    // Results output (set automatically if experimentDir is used)
    std::string resultsOutputPath = "";          // Empty = use experimentDir, path = override
    std::string latencyOutputPath = "";          // Empty = use experimentDir, path = override
    std::string trajectoryOutputPath = "";       // Empty = use experimentDir, path = override
    double trajectoryLogInterval = 0.5;          // Logging interval in seconds
    std::string simTag = "baseline";
    bool logging = false;

    //--------------------------------------------------------------------------
    // Command line parsing
    //--------------------------------------------------------------------------

    CommandLine cmd(__FILE__);

    // Topology
    cmd.AddValue("numRings", "Number of hexagonal rings (0=1 site, 1=7 sites)", numRings);
    cmd.AddValue("scenario", "Propagation scenario (UMa, RMa, UMi)", scenario);
    cmd.AddValue("isd", "Inter-site distance in meters", isd);

    // Mobility
    cmd.AddValue("ueSpeed", "UE speed in m/s (mean for random distributions)", ueSpeed);
    cmd.AddValue("ueSpeedVariance", "Velocity variance for Gaussian distribution", ueSpeedVariance);
    cmd.AddValue("gaussAlpha", "Gauss-Markov memory factor (0-1)", gaussAlpha);
    cmd.AddValue("gaussTimeStep", "Gauss-Markov update interval in seconds", gaussTimeStep);
    cmd.AddValue("waypointFile", "NS-2 format mobility trace file path", waypointFile);
    cmd.AddValue("builtinPath", "Built-in path: hexagonal, linear-y, zigzag, highway, urban-grid", builtinPath);
    cmd.AddValue("simTime", "Simulation time in seconds", simTime);
    cmd.AddValue("mobilityModel", "Mobility model (linear, random, gauss-markov, waypoint)", mobilityModel);

    // NR
    cmd.AddValue("centralFrequency", "Central frequency in Hz", centralFrequency);
    cmd.AddValue("bandwidth", "System bandwidth in Hz", bandwidth);
    cmd.AddValue("gnbTxPower", "gNB TX power in dBm", gnbTxPower);
    cmd.AddValue("numerology", "NR numerology (0-4)", numerology);

    // Traffic
    cmd.AddValue("packetSize", "Control packet size in bytes", packetSize);
    cmd.AddValue("intervalMs", "Packet interval in ms", intervalMs);

    // Background UEs for cell load
    cmd.AddValue("numBackgroundUes", "Number of stationary background UEs for cell load", numBackgroundUes);
    cmd.AddValue("bgUeTrafficMbps", "Traffic rate per background UE in Mbps", bgUeTrafficMbps);
    cmd.AddValue("bgUePacketSize", "Background UE packet size in bytes", bgUePacketSize);

    // Capacity testing
    cmd.AddValue("loadPercent", "Target load as % of capacity (0=use numBackgroundUes)", loadPercent);
    cmd.AddValue("capacityMbps", "Per-cell capacity in Mbps (0=auto-estimate)", capacityMbps);
    cmd.AddValue("saturationTest", "Run saturation test to find max capacity", saturationTest);

    // Network
    cmd.AddValue("wanDelayMs", "WAN delay in ms (one-way)", wanDelayMs);
    cmd.AddValue("s1uDelayMs", "S1-U link delay in ms", s1uDelayMs);
    cmd.AddValue("handoverInterruptMs", "Handover interruption time in ms (realistic: 30-100ms)", handoverInterruptMs);

    // SLA
    cmd.AddValue("slaThresholdMs", "SLA latency threshold in ms (violation if exceeded)", slaThresholdMs);

    // Experiment output directory
    cmd.AddValue("experimentDir", "Experiment output directory (empty=auto-generate with timestamp)", experimentDir);

    // Output paths (override experimentDir if specified)
    cmd.AddValue("latencyOutputPath", "Override latency CSV path (default: experimentDir/latency.csv)", latencyOutputPath);
    cmd.AddValue("trajectoryOutputPath", "Override trajectory CSV path (default: experimentDir/traj.csv)", trajectoryOutputPath);
    cmd.AddValue("resultsOutputPath", "Override results summary path (default: experimentDir/result.csv)", resultsOutputPath);
    cmd.AddValue("trajectoryLogInterval", "Trajectory logging interval in seconds", trajectoryLogInterval);

    cmd.AddValue("simTag", "Simulation tag for output files", simTag);
    cmd.AddValue("logging", "Enable detailed logging", logging);

    cmd.Parse(argc, argv);

    //--------------------------------------------------------------------------
    // Enable logging if requested
    //--------------------------------------------------------------------------

    if (logging)
    {
        LogComponentEnable("NrBaselineLatency", LOG_LEVEL_INFO);
    }

    // Update global parameters
    g_handoverInterruptMs = handoverInterruptMs;
    g_networkDelayMs = wanDelayMs + s1uDelayMs;  // Used to calculate packet arrival at gNB
    g_slaThresholdMs = slaThresholdMs;

    //--------------------------------------------------------------------------
    // Set up experiment output directory and paths
    //--------------------------------------------------------------------------

    // Generate experiment directory if not specified
    if (experimentDir.empty())
    {
        experimentDir = GenerateExperimentDir();
    }

    // Create experiment directory (and parent if needed)
    CreateDirectoryIfNeeded("experiment_data");
    if (!CreateDirectoryIfNeeded(experimentDir))
    {
        NS_LOG_WARN("Failed to create experiment directory: " << experimentDir);
    }

    // Set output paths to use experiment directory (if not explicitly overridden)
    std::string configOutputPath = experimentDir + "/config.txt";
    if (latencyOutputPath.empty())
    {
        latencyOutputPath = experimentDir + "/latency.csv";
    }
    if (trajectoryOutputPath.empty())
    {
        trajectoryOutputPath = experimentDir + "/traj.csv";
    }
    if (resultsOutputPath.empty())
    {
        resultsOutputPath = experimentDir + "/result.csv";
    }
    std::string flowmonFile = experimentDir + "/" + simTag + "-flowmon.xml";

    // Write experiment configuration
    WriteExperimentConfig(configOutputPath,
                          numRings, scenario, isd,
                          mobilityModel, ueSpeed, ueSpeedVariance,
                          gaussAlpha, gaussTimeStep, waypointFile, builtinPath,
                          simTime,
                          centralFrequency, bandwidth, gnbTxPower, numerology,
                          handoverInterruptMs, wanDelayMs, s1uDelayMs,
                          slaThresholdMs,
                          packetSize, intervalMs,
                          trajectoryLogInterval,
                          numBackgroundUes, bgUeTrafficMbps, bgUePacketSize,
                          loadPercent, capacityMbps);

    //--------------------------------------------------------------------------
    // Capacity testing: Calculate load from percentage if specified
    //--------------------------------------------------------------------------

    // Auto-estimate per-cell capacity based on bandwidth if not specified
    // Rough estimates for NR with numerology 1 (30 kHz SCS):
    //   5 MHz  -> ~15 Mbps per cell
    //   10 MHz -> ~30 Mbps per cell
    //   20 MHz -> ~60 Mbps per cell
    //   50 MHz -> ~150 Mbps per cell
    //   100 MHz -> ~300 Mbps per cell
    if (capacityMbps <= 0.0)
    {
        // Estimate: ~3 Mbps per MHz (conservative for DL with overhead)
        capacityMbps = (bandwidth / 1e6) * 3.0;
    }

    // If loadPercent is specified, calculate background traffic accordingly
    if (loadPercent > 0.0)
    {
        double targetLoadMbps = capacityMbps * (loadPercent / 100.0);

        // Calculate number of background UEs needed
        // Each UE generates bgUeTrafficMbps of traffic
        if (bgUeTrafficMbps > 0)
        {
            numBackgroundUes = static_cast<uint32_t>(std::ceil(targetLoadMbps / bgUeTrafficMbps));
        }

        NS_LOG_UNCOND("\n--- Capacity Testing Mode ---");
        NS_LOG_UNCOND("Estimated per-cell capacity: " << capacityMbps << " Mbps");
        NS_LOG_UNCOND("Target load: " << loadPercent << "% = " << targetLoadMbps << " Mbps");
        NS_LOG_UNCOND("Background UEs calculated: " << numBackgroundUes
                      << " (each " << bgUeTrafficMbps << " Mbps)");
        NS_LOG_UNCOND("Actual load: " << (numBackgroundUes * bgUeTrafficMbps) << " Mbps ("
                      << ((numBackgroundUes * bgUeTrafficMbps) / capacityMbps * 100.0) << "%)");
    }

    // Saturation test mode: progressively increase load
    if (saturationTest)
    {
        NS_LOG_UNCOND("\n=== SATURATION TEST MODE ===");
        NS_LOG_UNCOND("Estimated per-cell capacity: " << capacityMbps << " Mbps");
        NS_LOG_UNCOND("Will test at 50%, 70%, 80%, 90%, 95%, 100%, 110% of capacity");
        NS_LOG_UNCOND("Run with specific --loadPercent values to test each level");
        NS_LOG_UNCOND("============================\n");

        // For saturation test, default to 80% load if no loadPercent specified
        if (loadPercent <= 0.0)
        {
            loadPercent = 80.0;
            double targetLoadMbps = capacityMbps * (loadPercent / 100.0);
            numBackgroundUes = static_cast<uint32_t>(std::ceil(targetLoadMbps / bgUeTrafficMbps));
        }
    }

    //--------------------------------------------------------------------------
    // Print configuration
    //--------------------------------------------------------------------------

    NS_LOG_UNCOND("\n==============================================");
    NS_LOG_UNCOND("NR Baseline Latency Measurement");
    NS_LOG_UNCOND("==============================================");
    NS_LOG_UNCOND("Topology: " << (numRings == 0 ? 1 : (numRings == 1 ? 7 : 19)) << " sites");
    NS_LOG_UNCOND("Scenario: " << scenario << ", ISD: " << isd << " m");
    NS_LOG_UNCOND("Mobility: " << mobilityModel
                  << (mobilityModel == "waypoint" ? " (" + builtinPath + ")" : ""));
    NS_LOG_UNCOND("UE Speed: " << ueSpeed << " m/s (" << ueSpeed * 3.6 << " km/h)");
    if (mobilityModel == "gauss-markov")
    {
        NS_LOG_UNCOND("  Gauss-Markov: alpha=" << gaussAlpha << ", timeStep=" << gaussTimeStep << "s");
    }
    NS_LOG_UNCOND("WAN Delay: " << wanDelayMs << " ms (one-way)");
    NS_LOG_UNCOND("Handover Interruption: " << handoverInterruptMs << " ms");
    NS_LOG_UNCOND("SLA Threshold: " << slaThresholdMs << " ms");
    NS_LOG_UNCOND("DL Control: " << packetSize << " bytes every " << intervalMs << " ms");
    if (numBackgroundUes > 0)
    {
        NS_LOG_UNCOND("----------------------------------------------");
        NS_LOG_UNCOND("Background UEs: " << numBackgroundUes);
        NS_LOG_UNCOND("  Traffic per UE: " << bgUeTrafficMbps << " Mbps ("
                      << bgUePacketSize << " bytes/packet)");
        NS_LOG_UNCOND("  Total cell load: " << (numBackgroundUes * bgUeTrafficMbps) << " Mbps");
    }
    NS_LOG_UNCOND("Simulation: " << simTime << " s");
    NS_LOG_UNCOND("Output: " << experimentDir);
    NS_LOG_UNCOND("==============================================\n");

    //--------------------------------------------------------------------------
    // Initialize CSV output
    //--------------------------------------------------------------------------

    g_latencyCsv.open(latencyOutputPath);
    if (!g_latencyCsv.is_open())
    {
        NS_FATAL_ERROR("Cannot open CSV file: " << latencyOutputPath);
    }
    g_latencyCsv << "timestamp_s,seq,latency_ms,size_bytes,cell_id,during_handover,buffering_delay_ms,sla_violation\n";

    //--------------------------------------------------------------------------
    // Create hexagonal grid topology
    //--------------------------------------------------------------------------

    ScenarioParameters scenarioParams;
    scenarioParams.SetScenarioParameters(scenario);
    scenarioParams.m_isd = isd;
    scenarioParams.SetSectorization(1);  // Single-sector (1 cell per site)

    HexagonalGridScenarioHelper gridScenario;
    gridScenario.SetScenarioParameters(scenarioParams);
    gridScenario.SetNumRings(numRings);
    gridScenario.SetUtNumber(1);  // Single UE (vehicle)
    gridScenario.SetSimTag(simTag);

    // Calculate grid bounds for mobility models
    double gridRadius = isd * (numRings + 1);

    if (mobilityModel == "linear")
    {
        // Linear mobility: UE moves in +Y direction (toward Site 2 for handover)
        gridScenario.CreateScenarioWithMobility(Vector(0, ueSpeed, 0), 0.0);
    }
    else if (mobilityModel == "gauss-markov")
    {
        // Use CreateScenarioWithCustomMobility to avoid aggregation issues
        // (aggregation causes channel model to use old static mobility)

        // Tighter bounds to keep UE within multi-site coverage area
        // With ISD=500m, outer sites are at ~289m radius, so use ~400m bounds
        double gaussBounds = isd / std::sqrt(3.0) + 100.0;

        MobilityHelper gaussHelper;
        gaussHelper.SetMobilityModel("ns3::GaussMarkovMobilityModel",
            "Bounds", BoxValue(Box(-gaussBounds, gaussBounds, -gaussBounds, gaussBounds, 0, 10)),
            "TimeStep", TimeValue(Seconds(gaussTimeStep)),
            "Alpha", DoubleValue(gaussAlpha),
            "MeanVelocity", StringValue(BuildNormalVelocityString(ueSpeed, ueSpeedVariance)),
            "MeanDirection", StringValue("ns3::UniformRandomVariable[Min=0|Max=6.283185307]"),
            "MeanPitch", StringValue("ns3::ConstantRandomVariable[Constant=0]"),
            "NormalVelocity", StringValue("ns3::NormalRandomVariable[Mean=0.0|Variance=1.0|Bound=10.0]"),
            "NormalDirection", StringValue("ns3::NormalRandomVariable[Mean=0.0|Variance=0.6|Bound=1.2]"),
            "NormalPitch", StringValue("ns3::NormalRandomVariable[Mean=0.0|Variance=0.0|Bound=0.0]"));

        // Set initial position between Site 0 and Site 1 (~150m from center)
        // This increases chance of crossing site boundaries and triggering handovers
        Ptr<ListPositionAllocator> gaussPosAlloc = CreateObject<ListPositionAllocator>();
        gaussPosAlloc->Add(Vector(150.0, 0.0, 1.5));
        gaussHelper.SetPositionAllocator(gaussPosAlloc);

        // Create scenario with custom mobility (no aggregation issues)
        gridScenario.CreateScenarioWithCustomMobility(gaussHelper);

        NS_LOG_UNCOND("Gauss-Markov mobility: alpha=" << gaussAlpha
                    << " meanSpeed=" << ueSpeed << " m/s"
                    << " bounds=" << gaussBounds << "m");
    }
    else if (mobilityModel == "waypoint")
    {
        // Determine initial position based on path
        Vector startPos(0, 50, 1.5);  // Default start position
        if (builtinPath == "highway")
        {
            startPos = Vector(-400, -400, 1.5);
        }
        else if (builtinPath == "urban-grid")
        {
            startPos = Vector(-200, -200, 1.5);
        }

        // Configure MobilityHelper with WaypointMobilityModel
        MobilityHelper waypointHelper;
        waypointHelper.SetMobilityModel("ns3::WaypointMobilityModel");
        Ptr<ListPositionAllocator> posAlloc = CreateObject<ListPositionAllocator>();
        posAlloc->Add(startPos);
        waypointHelper.SetPositionAllocator(posAlloc);

        // Create scenario with custom mobility
        gridScenario.CreateScenarioWithCustomMobility(waypointHelper);

        NodeContainer ueNodesTemp = gridScenario.GetUserTerminals();

        if (!waypointFile.empty())
        {
            // Load from NS-2 format file
            Ns2MobilityHelper ns2(waypointFile);
            ns2.Install();
            NS_LOG_UNCOND("Waypoint mobility loaded from file: " << waypointFile);
        }
        else
        {
            // Get the installed WaypointMobilityModel and add waypoints
            Ptr<WaypointMobilityModel> waypointMm =
                ueNodesTemp.Get(0)->GetObject<WaypointMobilityModel>();

            if (!waypointMm)
            {
                NS_FATAL_ERROR("Failed to get WaypointMobilityModel from UE");
            }

            ApplyBuiltinPath(builtinPath, waypointMm, ueSpeed);
        }
    }
    else if (mobilityModel == "random")
    {
        // For random waypoint, first create static scenario
        gridScenario.CreateScenario();

        // Then install random waypoint mobility on UEs
        NodeContainer ueNodesTemp = gridScenario.GetUserTerminals();
        MobilityHelper mobilityHelper;

        Ptr<RandomBoxPositionAllocator> posAlloc = CreateObject<RandomBoxPositionAllocator>();
        posAlloc->SetAttribute("X", StringValue("ns3::UniformRandomVariable[Min=-" +
            std::to_string(gridRadius) + "|Max=" + std::to_string(gridRadius) + "]"));
        posAlloc->SetAttribute("Y", StringValue("ns3::UniformRandomVariable[Min=-" +
            std::to_string(gridRadius) + "|Max=" + std::to_string(gridRadius) + "]"));
        posAlloc->SetAttribute("Z", StringValue("ns3::ConstantRandomVariable[Constant=1.5]"));

        mobilityHelper.SetPositionAllocator(posAlloc);
        mobilityHelper.SetMobilityModel("ns3::RandomWaypointMobilityModel",
            "Speed", StringValue(BuildNormalVelocityString(ueSpeed, ueSpeedVariance)),
            "Pause", StringValue("ns3::ConstantRandomVariable[Constant=0]"),
            "PositionAllocator", PointerValue(posAlloc));

        mobilityHelper.Install(ueNodesTemp);
        NS_LOG_UNCOND("Random waypoint mobility with Normal velocity distribution");
    }
    else
    {
        NS_FATAL_ERROR("Unknown mobilityModel: " << mobilityModel);
    }

    NodeContainer gnbNodes = gridScenario.GetBaseStations();
    NodeContainer ueNodes = gridScenario.GetUserTerminals();

    NS_LOG_UNCOND("Created " << gnbNodes.GetN() << " gNBs and " << ueNodes.GetN() << " UE(s)");

    // Position UE near center for initial attachment (linear mobility only)
    if (mobilityModel == "linear")
    {
        Ptr<ConstantVelocityMobilityModel> ueMobility =
            ueNodes.Get(0)->GetObject<ConstantVelocityMobilityModel>();
        if (ueMobility)
        {
            ueMobility->SetPosition(Vector(0.0, 50.0, 1.5));
            ueMobility->SetVelocity(Vector(0.0, ueSpeed, 0.0));
        }
    }

    // Create background UEs for cell load (stationary, bidirectional traffic)
    NodeContainer backgroundUeNodes;
    if (numBackgroundUes > 0)
    {
        backgroundUeNodes.Create(numBackgroundUes);

        // Place background UEs near each site (1 per site, then cycle)
        // Site positions: center at (0,0), 6 outer sites at ~433m radius, 60° apart
        MobilityHelper bgMobility;
        Ptr<ListPositionAllocator> bgPosAlloc = CreateObject<ListPositionAllocator>();

        // Calculate site positions (7 sites for numRings=1)
        std::vector<Vector> sitePositions;
        sitePositions.push_back(Vector(0, 0, 0));  // Center site

        double siteRadius = isd / std::sqrt(3.0);  // ~433m for ISD=500m
        for (int s = 0; s < 6; s++)
        {
            double siteAngle = M_PI / 6.0 + s * M_PI / 3.0;  // 30°, 90°, 150°, etc.
            sitePositions.push_back(Vector(siteRadius * cos(siteAngle),
                                           siteRadius * sin(siteAngle), 0));
        }

        // Place UEs near sites (offset by 100m to be within cell coverage)
        double ueOffset = 100.0;
        for (uint32_t i = 0; i < numBackgroundUes; i++)
        {
            uint32_t siteIdx = i % sitePositions.size();
            Vector sitePos = sitePositions[siteIdx];

            // Offset UE from site center (rotate offset for multiple UEs per site)
            double offsetAngle = 2.0 * M_PI * (i / sitePositions.size()) / 3.0;
            Vector pos(sitePos.x + ueOffset * cos(offsetAngle),
                       sitePos.y + ueOffset * sin(offsetAngle),
                       1.5);
            bgPosAlloc->Add(pos);
        }

        bgMobility.SetPositionAllocator(bgPosAlloc);
        bgMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
        bgMobility.Install(backgroundUeNodes);

        NS_LOG_UNCOND("Created " << numBackgroundUes << " background UE(s) distributed across "
                      << sitePositions.size() << " sites");
    }

    //--------------------------------------------------------------------------
    // Set up NR network
    //--------------------------------------------------------------------------

    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper> idealBeamformingHelper = CreateObject<IdealBeamformingHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();

    nrHelper->SetBeamformingHelper(idealBeamformingHelper);
    nrHelper->SetEpcHelper(epcHelper);

    // Configure S1-U delay
    epcHelper->SetAttribute("S1uLinkDelay", TimeValue(MilliSeconds(s1uDelayMs)));

    // Configure handover algorithm (A3-RSRP)
    nrHelper->SetHandoverAlgorithmType("ns3::NrA3RsrpHandoverAlgorithm");
    nrHelper->SetHandoverAlgorithmAttribute("Hysteresis", DoubleValue(3.0));
    nrHelper->SetHandoverAlgorithmAttribute("TimeToTrigger", TimeValue(MilliSeconds(256)));

    // Spectrum configuration
    BandwidthPartInfoPtrVector allBwps;
    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(centralFrequency, bandwidth, 1);
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);

    // Channel model with realistic propagation
    // - "ThreeGpp" spectrum model enables fast fading (small-scale fading)
    // - ShadowingEnabled adds slow fading (large-scale fading)
    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigureFactories(scenario, "Default", "ThreeGpp");
    channelHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(true));
    channelHelper->AssignChannelsToBands({band});
    allBwps = CcBwpCreator::GetAllBwps({band});

    // Configure antennas - omnidirectional for single-sector sites
    nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(4));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(4));
    nrHelper->SetGnbAntennaAttribute("AntennaElement",
                                     PointerValue(CreateObject<IsotropicAntennaModel>()));

    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(2));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(4));
    nrHelper->SetUeAntennaAttribute("AntennaElement",
                                    PointerValue(CreateObject<IsotropicAntennaModel>()));

    // Beamforming - use DirectPath for omnidirectional antenna setup
    idealBeamformingHelper->SetAttribute("BeamformingMethod",
                                         TypeIdValue(DirectPathBeamforming::GetTypeId()));

    // Install NR devices
    NetDeviceContainer gnbNetDevs = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    NetDeviceContainer ueNetDevs = nrHelper->InstallUeDevice(ueNodes, allBwps);

    // Configure gNB TX power and numerology
    for (uint32_t i = 0; i < gnbNetDevs.GetN(); ++i)
    {
        nrHelper->GetGnbPhy(gnbNetDevs.Get(i), 0)->SetAttribute("Numerology", UintegerValue(numerology));
        nrHelper->GetGnbPhy(gnbNetDevs.Get(i), 0)->SetAttribute("TxPower", DoubleValue(gnbTxPower));
    }

    // Install NR devices on background UEs
    NetDeviceContainer bgUeNetDevs;
    if (numBackgroundUes > 0)
    {
        bgUeNetDevs = nrHelper->InstallUeDevice(backgroundUeNodes, allBwps);
    }

    //--------------------------------------------------------------------------
    // Set up Remote Host with P2P link to PGW
    //--------------------------------------------------------------------------

    auto [remoteHost, remoteHostIp] = epcHelper->SetupRemoteHost(
        "1Gbps",                          // Data rate
        1500,                             // MTU
        MilliSeconds(wanDelayMs)          // WAN delay (one-way)
    );

    NS_LOG_UNCOND("Remote Host IP: " << remoteHostIp);

    //--------------------------------------------------------------------------
    // Set up IP networking for UE
    //--------------------------------------------------------------------------

    InternetStackHelper internet;
    internet.Install(ueNodes);

    // Install internet stack on background UEs
    if (numBackgroundUes > 0)
    {
        internet.Install(backgroundUeNodes);
    }

    Ipv4InterfaceContainer ueIpIfaces = epcHelper->AssignUeIpv4Address(ueNetDevs);
    Ipv4Address ueIp = ueIpIfaces.GetAddress(0);

    // Set default route for UE
    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> ueStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(ueNodes.Get(0)->GetObject<Ipv4>());
    ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);

    NS_LOG_UNCOND("UE IP: " << ueIp);

    // Assign IPs and configure routes for background UEs
    std::vector<Ipv4Address> bgUeIps;
    if (numBackgroundUes > 0)
    {
        for (uint32_t i = 0; i < numBackgroundUes; i++)
        {
            Ipv4InterfaceContainer bgUeIpIface =
                epcHelper->AssignUeIpv4Address(NetDeviceContainer(bgUeNetDevs.Get(i)));
            bgUeIps.push_back(bgUeIpIface.GetAddress(0));

            // Set default route
            Ptr<Ipv4StaticRouting> bgUeRouting =
                ipv4RoutingHelper.GetStaticRouting(backgroundUeNodes.Get(i)->GetObject<Ipv4>());
            bgUeRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
        }
    }

    //--------------------------------------------------------------------------
    // Attach UE and configure X2 for handover
    //--------------------------------------------------------------------------

    nrHelper->AttachToClosestGnb(ueNetDevs, gnbNetDevs);
    nrHelper->AddX2Interface(gnbNodes);

    // Attach background UEs to closest gNB
    if (numBackgroundUes > 0)
    {
        nrHelper->AttachToClosestGnb(bgUeNetDevs, gnbNetDevs);
    }

    // Get UE IMSI and initial serving cell
    Ptr<NrUeNetDevice> ueDev = ueNetDevs.Get(0)->GetObject<NrUeNetDevice>();
    g_ueImsi = ueDev->GetImsi();
    g_ueServingCell[g_ueImsi] = ueDev->GetCellId();
    g_ueHandoverActive[g_ueImsi] = false;

    NS_LOG_UNCOND("UE IMSI: " << g_ueImsi << ", Initial Cell: " << g_ueServingCell[g_ueImsi]);

    //--------------------------------------------------------------------------
    // Install applications
    //--------------------------------------------------------------------------

    uint16_t dlPort = 1234;
    Time appStartTime = MilliSeconds(500);

    // Calculate data rate from packet size and interval
    double dataRateBps = (packetSize * 8.0) / (intervalMs / 1000.0);

    // DL: OnOff on Remote Host -> PacketSink on UE
    OnOffHelper dlOnOff("ns3::UdpSocketFactory",
                        InetSocketAddress(ueIp, dlPort));
    dlOnOff.SetConstantRate(DataRate(dataRateBps));
    dlOnOff.SetAttribute("PacketSize", UintegerValue(packetSize));
    dlOnOff.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));

    ApplicationContainer dlClientApp = dlOnOff.Install(remoteHost);
    dlClientApp.Start(appStartTime);
    dlClientApp.Stop(Seconds(simTime));

    PacketSinkHelper dlSink("ns3::UdpSocketFactory",
                            InetSocketAddress(Ipv4Address::GetAny(), dlPort));
    dlSink.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));

    ApplicationContainer dlSinkApp = dlSink.Install(ueNodes.Get(0));
    dlSinkApp.Start(appStartTime);
    dlSinkApp.Stop(Seconds(simTime));

    //--------------------------------------------------------------------------
    // Install DL-only traffic on background UEs (creates cell load)
    //--------------------------------------------------------------------------

    if (numBackgroundUes > 0)
    {
        uint16_t bgBasePort = 4000;
        double bgDataRateBps = bgUeTrafficMbps * 1e6;

        for (uint32_t i = 0; i < numBackgroundUes; i++)
        {
            uint16_t dlPort = bgBasePort + i;

            // Stagger start times: 100ms apart to avoid simultaneous bursts
            Time bgStartTime = appStartTime + MilliSeconds(100 * i);

            // DL only: Remote Host -> Background UE (avoids TDD collision)
            OnOffHelper bgDlOnOff("ns3::UdpSocketFactory",
                                   InetSocketAddress(bgUeIps[i], dlPort));
            bgDlOnOff.SetConstantRate(DataRate(bgDataRateBps));
            bgDlOnOff.SetAttribute("PacketSize", UintegerValue(bgUePacketSize));

            ApplicationContainer bgDlApp = bgDlOnOff.Install(remoteHost);
            bgDlApp.Start(bgStartTime);
            bgDlApp.Stop(Seconds(simTime));

            PacketSinkHelper bgDlSink("ns3::UdpSocketFactory",
                                       InetSocketAddress(Ipv4Address::GetAny(), dlPort));
            ApplicationContainer bgDlSinkApp = bgDlSink.Install(backgroundUeNodes.Get(i));
            bgDlSinkApp.Start(bgStartTime);
            bgDlSinkApp.Stop(Seconds(simTime));
        }

        NS_LOG_UNCOND("Background DL traffic: " << numBackgroundUes << " UEs x "
                      << bgUeTrafficMbps << " Mbps = "
                      << (numBackgroundUes * bgUeTrafficMbps) << " Mbps total"
                      << " (staggered start: 100ms apart)");
    }

    //--------------------------------------------------------------------------
    // Connect trace sources
    //--------------------------------------------------------------------------

    // DL send tracking via OnOff application's TxWithSeqTsSize trace
    dlClientApp.Get(0)->TraceConnectWithoutContext("TxWithSeqTsSize",
                                                    MakeCallback(&DlTxCallback));

    // DL latency tracking via PacketSink
    dlSinkApp.Get(0)->TraceConnectWithoutContext("RxWithSeqTsSize",
                                                  MakeCallback(&DlRxCallback));

    // Handover callbacks
    std::string uePath = "/NodeList/" + std::to_string(ueNodes.Get(0)->GetId()) +
                         "/DeviceList/0/NrUeRrc/";
    Config::Connect(uePath + "HandoverStart", MakeCallback(&HandoverStartCallback));
    Config::Connect(uePath + "HandoverEndOk", MakeCallback(&HandoverEndOkCallback));
    Config::Connect(uePath + "HandoverEndError", MakeCallback(&HandoverEndErrorCallback));

    //--------------------------------------------------------------------------
    // FlowMonitor for aggregate statistics
    //--------------------------------------------------------------------------

    FlowMonitorHelper flowmonHelper;
    NodeContainer endpointNodes;
    endpointNodes.Add(remoteHost);
    endpointNodes.Add(ueNodes);
    Ptr<FlowMonitor> monitor = flowmonHelper.Install(endpointNodes);

    //--------------------------------------------------------------------------
    // Schedule UE position printing
    //--------------------------------------------------------------------------

    Simulator::Schedule(Seconds(1.0), &PrintUePosition, ueNodes.Get(0));

    //--------------------------------------------------------------------------
    // Initialize trajectory logging (if enabled)
    //--------------------------------------------------------------------------

    if (!trajectoryOutputPath.empty())
    {
        g_trajectoryFile.open(trajectoryOutputPath);
        if (g_trajectoryFile.is_open())
        {
            g_trajectoryLoggingEnabled = true;
            // Write CSV header
            g_trajectoryFile << "time_s,imsi,x,y,z,vx,vy,vz,speed,cell_id\n";

            // Schedule first logging
            Simulator::Schedule(Seconds(trajectoryLogInterval), &LogUeTrajectory, ueNodes.Get(0), trajectoryLogInterval);

            NS_LOG_UNCOND("Trajectory logging enabled: " << trajectoryOutputPath
                          << " (interval=" << trajectoryLogInterval << "s)");
        }
        else
        {
            NS_LOG_WARN("Failed to open trajectory file: " << trajectoryOutputPath);
        }
    }

    //--------------------------------------------------------------------------
    // Run simulation
    //--------------------------------------------------------------------------

    NS_LOG_UNCOND("\nStarting simulation...\n");
    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    //--------------------------------------------------------------------------
    // Output results
    //--------------------------------------------------------------------------

    g_latencyCsv.close();

    // FlowMonitor statistics
    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier =
        DynamicCast<Ipv4FlowClassifier>(flowmonHelper.GetClassifier());
    FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats();

    NS_LOG_UNCOND("\n--- FlowMonitor Statistics ---");
    for (auto& flow : stats)
    {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(flow.first);

        // Only show DL flow (Remote Host -> UE)
        if (t.destinationAddress == ueIp)
        {
            double avgDelay = flow.second.rxPackets > 0 ?
                flow.second.delaySum.GetMilliSeconds() / flow.second.rxPackets : 0;
            double avgJitter = flow.second.rxPackets > 1 ?
                flow.second.jitterSum.GetMilliSeconds() / (flow.second.rxPackets - 1) : 0;
            double throughput = flow.second.rxBytes * 8.0 /
                (flow.second.timeLastRxPacket - flow.second.timeFirstTxPacket).GetSeconds() / 1000;

            NS_LOG_UNCOND("Flow " << flow.first << " (" << t.sourceAddress << " -> " << t.destinationAddress << ")");
            NS_LOG_UNCOND("  TX packets: " << flow.second.txPackets);
            NS_LOG_UNCOND("  RX packets: " << flow.second.rxPackets);
            NS_LOG_UNCOND("  Lost packets: " << flow.second.lostPackets);
            NS_LOG_UNCOND("  Avg delay: " << std::fixed << std::setprecision(3) << avgDelay << " ms");
            NS_LOG_UNCOND("  Avg jitter: " << avgJitter << " ms");
            NS_LOG_UNCOND("  Throughput: " << throughput << " kbps");
        }
    }

    // Save FlowMonitor to XML
    monitor->SerializeToXmlFile(flowmonFile, true, true);

    // Close trajectory file
    if (g_trajectoryLoggingEnabled && g_trajectoryFile.is_open())
    {
        g_trajectoryFile.close();
    }

    // Write simulation results summary (includes packet loss from pending sends)
    if (!resultsOutputPath.empty())
    {
        WriteSimulationResults(resultsOutputPath, simTime, slaThresholdMs);
    }

    // Print experiment directory summary
    NS_LOG_UNCOND("\n==============================================");
    NS_LOG_UNCOND("Experiment Output Directory: " << experimentDir);
    NS_LOG_UNCOND("  config.txt  - experiment configuration");
    NS_LOG_UNCOND("  latency.csv - per-packet latency data");
    NS_LOG_UNCOND("  traj.csv    - UE trajectory data");
    NS_LOG_UNCOND("  result.csv  - simulation results summary");
    NS_LOG_UNCOND("  " << simTag << "-flowmon.xml - FlowMonitor XML");
    NS_LOG_UNCOND("==============================================\n");

    Simulator::Destroy();
    return 0;
}
