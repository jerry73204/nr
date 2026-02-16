// Copyright (c) 2024
// SPDX-License-Identifier: GPL-2.0-only

/**
 * @ingroup examples
 * @file nr-mec-3gpp-calibration.cc
 * @brief Mobile Edge Computing (MEC) scenario with 3GPP hexagonal grid topology
 *
 * This example combines:
 * - 3GPP hexagonal grid topology from cttc-nr-3gpp-calibration-ho.cc
 * - MEC edge server tunneling from nr-mec-handover.cc
 *
 * Key features:
 * - Hexagonal grid deployment with configurable rings (1 ring = 7 sites, 7 cells)
 * - Single-sector sites (1 cell per site)
 * - One edge server per site
 * - IP-in-UDP tunneling for edge server communication
 * - Handover prediction and dynamic edge server switching
 * - Support for linear or random waypoint UE mobility
 *
 * Network Topology (1 ring example - 7 sites, ISD=500m):
 *
 *                    Site 2
 *                   (0, 500)
 *                    /    \
 *       Site 3      /      \      Site 1
 *    (-433, 250)   /        \   (433, 250)
 *                 /          \
 *                /   Site 0   \
 *               /    (0, 0)    \
 *               \              /
 *    Site 4      \            /      Site 6
 *  (-433, -250)   \          /    (433, -250)
 *                  \        /
 *                   \      /
 *                    Site 5
 *                  (0, -500)
 *
 * Each site has 1 cell, and each site has one edge server.
 * Intra-site handover: no edge server switch
 * Inter-site handover: edge server switch triggered
 */

#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/buildings-module.h"
#include "ns3/config-store-module.h"
#include "ns3/core-module.h"
#include "ns3/csma-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-apps-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/ns2-mobility-helper.h"
#include "ns3/nr-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/tap-bridge-module.h"

#include "builtin-mobility-paths.h"
#include "ue-tunnel-app.h"
#include "edge-tunnel-app.h"
#include "handover-prediction-file.h"
#include "zenoh-latency-measurement.h"

#include <cerrno>
#include <cmath>
#include <ctime>
#include <deque>
#include <fstream>
#include <iomanip>
#include <map>
#include <set>
#include <sstream>
#include <sys/stat.h>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NrMec3gppCalibration");

//==============================================================================
// Global state for handover prediction and edge server tracking
//==============================================================================

// Map from gNB cell ID to edge server node
std::map<uint16_t, Ptr<Node>> g_cellIdToEdgeServer;

// Map from gNB cell ID to edge server IP address
std::map<uint16_t, Ipv4Address> g_cellIdToEdgeServerIp;

// Map from gNB cell ID to site ID (for determining if edge switch is needed)
std::map<uint16_t, uint16_t> g_cellIdToSiteId;

// Current serving cell for each UE (for tracking)
std::map<uint64_t, uint16_t> g_ueCurrentServingCell;

//==============================================================================
// Handover Blackout Model (from nr-baseline-latency.cc)
//==============================================================================

// Handover interruption model parameters
double g_handoverInterruptMs = 30.0;  // Total handover interruption time (ms)
double g_networkDelayMs = 20.0;       // WAN delay (for calculating arrival at gNB)
std::map<uint64_t, Time> g_handoverStartTime;  // IMSI -> actual handover start time
std::map<uint64_t, Time> g_handoverEndTime;    // IMSI -> handover end time (start + interrupt)

// Structure to store measurement history for prediction
struct MeasurementHistory
{
    Time timestamp;
    uint16_t cellId;
    int rsrp;     // in dBm
    double rsrq;  // in dB
};

// History of measurements for each cell (for prediction algorithms)
std::map<uint16_t, std::deque<MeasurementHistory>> g_measurementHistory;
const size_t MAX_HISTORY_SIZE = 10;

// Global reference to UE tunnel apps for dynamic endpoint switching
std::map<uint64_t, Ptr<UeTunnelApp>> g_ueTunnelApps;

// Main UE IMSI (to filter out background UE measurement reports)
uint64_t g_mainUeImsi = 0;

// Pointer to the scenario helper (for index mapping)
NodeDistributionScenarioInterface* g_scenario = nullptr;

// Trajectory logging (buffered in memory, written at simulation end)
std::vector<std::string> g_trajectoryBuffer;
bool g_trajectoryLoggingEnabled = false;

//==============================================================================
// Handover Event Tracking (for result calculation)
//==============================================================================

struct HandoverEvent
{
    Time startTime;
    Time endTime;
    uint64_t imsi;
    uint16_t sourceCell;
    uint16_t targetCell;
    uint16_t sourceSite;
    uint16_t targetSite;
    bool isInterSite;  // true if edge server switch occurred
    bool success;      // true if handover completed successfully
};

std::vector<HandoverEvent> g_handoverEvents;
std::string g_resultsOutputPath;  // Path for results summary file

//==============================================================================
// Handover Prediction Algorithm (Same as nr-mec-handover.cc)
//==============================================================================

/**
 * @brief Calculate RSRP trend using time-based linear regression
 * @param history Measurement history for a cell
 * @return Slope in dB/second (positive = improving signal)
 */
double
CalculateRsrpTrend(const std::deque<MeasurementHistory>& history)
{
    if (history.size() < 2)
    {
        return 0.0;
    }

    double sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
    double t0 = history.front().timestamp.GetSeconds();

    for (const auto& meas : history)
    {
        double x = meas.timestamp.GetSeconds() - t0;
        double y = static_cast<double>(meas.rsrp);
        sumX += x;
        sumY += y;
        sumXY += x * y;
        sumX2 += x * x;
    }

    size_t n = history.size();
    double denom = n * sumX2 - sumX * sumX;
    if (denom == 0)
    {
        return 0.0;
    }

    return (n * sumXY - sumX * sumY) / denom;  // dB/second
}

/**
 * @brief Predict which cell will be the handover target based on RSRP trends
 * @param currentCellId The current serving cell
 * @param currentNeighbors Set of neighbor cell IDs from the current measurement report
 * @return Predicted target cell ID (0 if no prediction)
 */
uint16_t
PredictHandoverTarget(uint16_t currentCellId, const std::set<uint16_t>& currentNeighbors)
{
    auto servingIt = g_measurementHistory.find(currentCellId);
    if (servingIt == g_measurementHistory.end() || servingIt->second.empty())
    {
        return 0;
    }

    int servingRsrp = servingIt->second.back().rsrp;
    const double hysteresis = 3.0;
    const int rsrpFloor = -140;  // Minimum reportable RSRP

    // Calculate serving cell trend (negative = getting weaker = moving away)
    double servingTrend = 0.0;
    if (servingIt->second.size() >= 2)
    {
        servingTrend = CalculateRsrpTrend(servingIt->second);
    }

    uint16_t predictedTarget = 0;
    double bestScore = -999.0;

    // Only consider neighbors that are in the current measurement report
    for (uint16_t cellId : currentNeighbors)
    {
        auto histIt = g_measurementHistory.find(cellId);
        if (histIt == g_measurementHistory.end() || histIt->second.size() < 2)
        {
            continue;
        }
        const auto& history = histIt->second;

        int neighborRsrp = history.back().rsrp;

        // Skip neighbors with floor RSRP (unreliable measurement)
        if (neighborRsrp <= rsrpFloor)
        {
            continue;
        }

        // When serving is at floor, just pick strongest neighbor (ignore trend)
        // because floor measurements are unreliable for trend calculation
        if (servingRsrp <= rsrpFloor)
        {
            // Simply use the best RSRP when serving cell is at floor
            if (neighborRsrp > bestScore)
            {
                bestScore = neighborRsrp;
                predictedTarget = cellId;
            }
            continue;
        }

        // Normal case: use margin and trends for prediction
        double neighborTrend = CalculateRsrpTrend(history);
        double margin = neighborRsrp - servingRsrp - hysteresis;

        // Predict handover when:
        // 1. Neighbor is already stronger than serving (margin >= 0), OR
        // 2. Close to threshold AND (neighbor improving OR serving weakening)
        bool marginClose = margin > -6.0;
        bool neighborImproving = neighborTrend > 0.5;  // dB/s
        bool servingWeakening = servingTrend < -0.5;   // dB/s

        if (margin >= 0 || (marginClose && (neighborImproving || servingWeakening)))
        {
            // Score based on margin and relative trend (neighbor - serving)
            double relativeTrend = neighborTrend - servingTrend;
            double score = margin + relativeTrend * 2.0;
            if (score > bestScore)
            {
                bestScore = score;
                predictedTarget = cellId;
            }
        }
    }

    return predictedTarget;
}

//==============================================================================
// Callback Functions for Monitoring
//==============================================================================

/**
 * @brief Callback when a measurement report is received at the gNB
 */
void
MeasurementReportCallback(std::string path,
                          uint64_t imsi,
                          uint16_t cellId,
                          uint16_t rnti,
                          NrRrcSap::MeasurementReport meas)
{
    // Skip measurement reports from background UEs to reduce computation
    if (g_mainUeImsi != 0 && imsi != g_mainUeImsi)
    {
        return;
    }

    NS_LOG_INFO(Simulator::Now().GetSeconds()
                << "s [MEAS REPORT] IMSI=" << imsi << " RNTI=" << rnti
                << " ServingCell=" << cellId);

    int servingRsrp = static_cast<int>(meas.measResults.measResultPCell.rsrpResult) - 140;
    double servingRsrq = (static_cast<int>(meas.measResults.measResultPCell.rsrqResult) - 40) / 2.0;

    NS_LOG_INFO("  Serving Cell RSRP=" << servingRsrp << " dBm, RSRQ=" << servingRsrq << " dB");

    // Update history for serving cell
    MeasurementHistory servingMeas{Simulator::Now(), cellId, servingRsrp, servingRsrq};
    auto& servingHistory = g_measurementHistory[cellId];
    servingHistory.push_back(servingMeas);
    if (servingHistory.size() > MAX_HISTORY_SIZE)
    {
        servingHistory.pop_front();
    }

    // Process neighbor cell measurements and collect current neighbor cell IDs
    std::set<uint16_t> currentNeighbors;
    if (meas.measResults.haveMeasResultNeighCells)
    {
        for (const auto& neighbor : meas.measResults.measResultListEutra)
        {
            int neighborRsrp = static_cast<int>(neighbor.rsrpResult) - 140;
            double neighborRsrq = (static_cast<int>(neighbor.rsrqResult) - 40) / 2.0;

            NS_LOG_INFO("  Neighbor Cell " << neighbor.physCellId
                        << ": RSRP=" << neighborRsrp << " dBm, RSRQ=" << neighborRsrq << " dB");

            MeasurementHistory neighborMeas{Simulator::Now(), neighbor.physCellId, neighborRsrp, neighborRsrq};
            auto& neighborHistory = g_measurementHistory[neighbor.physCellId];
            neighborHistory.push_back(neighborMeas);
            if (neighborHistory.size() > MAX_HISTORY_SIZE)
            {
                neighborHistory.pop_front();
            }

            // Track current neighbors for prediction
            currentNeighbors.insert(neighbor.physCellId);
        }
    }

    // Run handover prediction (only considers neighbors in current measurement report)
    uint16_t predictedTarget = PredictHandoverTarget(cellId, currentNeighbors);

    Ipv4Address targetEdgeIp = Ipv4Address("0.0.0.0");
    double rsrpTrend = 0.0;

    if (predictedTarget > 0 && g_cellIdToEdgeServerIp.count(predictedTarget) > 0)
    {
        targetEdgeIp = g_cellIdToEdgeServerIp[predictedTarget];
        const auto& history = g_measurementHistory[predictedTarget];
        rsrpTrend = CalculateRsrpTrend(history);

        // Check if this would be an inter-site handover
        uint16_t currentSite = g_cellIdToSiteId[cellId];
        uint16_t targetSite = g_cellIdToSiteId[predictedTarget];

        if (currentSite != targetSite)
        {
            NS_LOG_INFO("  [PREDICTION] Inter-site handover to Cell " << predictedTarget
                        << " (Site " << currentSite << " -> " << targetSite
                        << ", Edge: " << targetEdgeIp << ")");
        }
        else
        {
            NS_LOG_INFO("  [PREDICTION] Intra-site handover to Cell " << predictedTarget
                        << " (same site " << currentSite << ", no edge switch)");
        }
    }

    // Update prediction file
    HandoverPredictionFile::GetInstance().UpdatePrediction(
        imsi, cellId, predictedTarget, targetEdgeIp, servingRsrp, rsrpTrend);
}

/**
 * @brief Callback when handover starts at the UE
 */
void
HandoverStartCallback(std::string path,
                      uint64_t imsi,
                      uint16_t sourceCellId,
                      uint16_t rnti,
                      uint16_t targetCellId)
{
    Time now = Simulator::Now();

    // Update current serving cell before handover completes
    // This ensures HandoverEndOkCallback knows the previous cell
    g_ueCurrentServingCell[imsi] = sourceCellId;

    // Set handover window for blackout model: [now, now + handoverInterruptMs]
    // Packets that ARRIVE at gNB during this window will experience buffering delay
    g_handoverStartTime[imsi] = now;
    g_handoverEndTime[imsi] = now + MilliSeconds(g_handoverInterruptMs);

    uint16_t sourceSite = g_cellIdToSiteId.count(sourceCellId) ? g_cellIdToSiteId[sourceCellId] : 0;
    uint16_t targetSite = g_cellIdToSiteId.count(targetCellId) ? g_cellIdToSiteId[targetCellId] : 0;
    bool isIntraSite = (sourceSite == targetSite);

    NS_LOG_UNCOND("\n======== HANDOVER EVENT ========");
    NS_LOG_UNCOND(now.GetSeconds()
                  << "s [HANDOVER START] IMSI=" << imsi
                  << " Cell " << sourceCellId << " (Site " << sourceSite << ")"
                  << " -> Cell " << targetCellId << " (Site " << targetSite << ")"
                  << (isIntraSite ? " [INTRA-SITE]" : " [INTER-SITE]")
                  << " (interrupt: " << g_handoverInterruptMs << "ms)");

    // Mark handover as active and set handover window for latency measurement
    if (g_ueTunnelApps.count(imsi) > 0 && g_ueTunnelApps[imsi])
    {
        NS_LOG_UNCOND("  Tunnel endpoint BEFORE: " << g_ueTunnelApps[imsi]->GetTunnelEndpoint());
        g_ueTunnelApps[imsi]->SetHandoverActive(true);
        g_ueTunnelApps[imsi]->SetHandoverWindow(now, g_handoverEndTime[imsi], g_networkDelayMs);
    }

    if (!isIntraSite)
    {
        Ipv4Address sourceEdgeIp = g_cellIdToEdgeServerIp.count(sourceCellId) ?
            g_cellIdToEdgeServerIp[sourceCellId] : Ipv4Address("0.0.0.0");
        Ipv4Address targetEdgeIp = g_cellIdToEdgeServerIp.count(targetCellId) ?
            g_cellIdToEdgeServerIp[targetCellId] : Ipv4Address("0.0.0.0");

        NS_LOG_UNCOND("  Inter-site handover: Edge " << sourceEdgeIp << " -> " << targetEdgeIp);

        // Write handover start event to file
        HandoverPredictionFile::GetInstance().WriteHandoverEvent(
            imsi, sourceCellId, targetCellId, sourceEdgeIp, targetEdgeIp, "handover_start");
    }
    else
    {
        Ipv4Address edgeIp = g_cellIdToEdgeServerIp.count(sourceCellId) ?
            g_cellIdToEdgeServerIp[sourceCellId] : Ipv4Address("0.0.0.0");
        NS_LOG_UNCOND("  Intra-site handover: edge server unchanged (Edge IP: " << edgeIp << ")");
    }

    // Record handover event for statistics
    HandoverEvent event;
    event.startTime = now;
    event.endTime = Time(0);  // Will be set when handover completes
    event.imsi = imsi;
    event.sourceCell = sourceCellId;
    event.targetCell = targetCellId;
    event.sourceSite = sourceSite;
    event.targetSite = targetSite;
    event.isInterSite = !isIntraSite;
    event.success = false;  // Will be set to true if handover succeeds
    g_handoverEvents.push_back(event);
}

/**
 * @brief Callback when handover completes successfully
 */
void
HandoverEndOkCallback(std::string path, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
    uint16_t previousCell = g_ueCurrentServingCell.count(imsi) ? g_ueCurrentServingCell[imsi] : 0;
    uint16_t previousSite = g_cellIdToSiteId.count(previousCell) ? g_cellIdToSiteId[previousCell] : 0;
    uint16_t newSite = g_cellIdToSiteId.count(cellId) ? g_cellIdToSiteId[cellId] : 0;
    bool isIntraSite = (previousSite == newSite);

    g_ueCurrentServingCell[imsi] = cellId;

    NS_LOG_UNCOND(Simulator::Now().GetSeconds()
                  << "s [HANDOVER SUCCESS] IMSI=" << imsi
                  << " New ServingCell=" << cellId << " (Site " << newSite << ")"
                  << (isIntraSite ? " [INTRA-SITE]" : " [INTER-SITE]"));

    // Clear handover active flag for latency measurement
    if (g_ueTunnelApps.count(imsi) > 0 && g_ueTunnelApps[imsi])
    {
        g_ueTunnelApps[imsi]->SetHandoverActive(false);
        NS_LOG_UNCOND("  Tunnel endpoint AFTER: " << g_ueTunnelApps[imsi]->GetTunnelEndpoint());
    }
    NS_LOG_UNCOND("================================\n");

    // Only switch tunnel endpoint if this is an inter-site handover
    if (previousSite != newSite && g_cellIdToEdgeServerIp.count(cellId) > 0)
    {
        Ipv4Address newEdgeServerIp = g_cellIdToEdgeServerIp[cellId];
        NS_LOG_UNCOND("  Switching to Edge Server at " << newEdgeServerIp);

        // Switch tunnel endpoint
        if (g_ueTunnelApps.count(imsi) > 0 && g_ueTunnelApps[imsi])
        {
            g_ueTunnelApps[imsi]->UpdateTunnelEndpoint(newEdgeServerIp);
        }

        // Write handover event to file
        Ipv4Address previousEdgeIp = g_cellIdToEdgeServerIp.count(previousCell) ?
            g_cellIdToEdgeServerIp[previousCell] : Ipv4Address("0.0.0.0");
        HandoverPredictionFile::GetInstance().WriteHandoverEvent(
            imsi, previousCell, cellId, previousEdgeIp, newEdgeServerIp, "handover_success");
    }
    else
    {
        NS_LOG_UNCOND("  Intra-site handover: edge server unchanged");
    }

    // Update handover event record - mark as successful and set end time
    Time now = Simulator::Now();
    for (auto it = g_handoverEvents.rbegin(); it != g_handoverEvents.rend(); ++it)
    {
        if (it->imsi == imsi && !it->success && it->targetCell == cellId)
        {
            it->endTime = now;
            it->success = true;
            break;
        }
    }
}

/**
 * @brief Callback when handover fails
 */
void
HandoverEndErrorCallback(std::string path, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
    NS_LOG_UNCOND(Simulator::Now().GetSeconds()
                  << "s [HANDOVER FAILED] IMSI=" << imsi << " CellId=" << cellId);

    // Update handover event record - mark as failed and set end time
    Time now = Simulator::Now();
    for (auto it = g_handoverEvents.rbegin(); it != g_handoverEvents.rend(); ++it)
    {
        if (it->imsi == imsi && !it->success)
        {
            it->endTime = now;
            // success remains false
            break;
        }
    }
}

/**
 * @brief Print UE position periodically
 */
void
PrintUePosition(NodeContainer ueNodes)
{
    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        Ptr<Node> ue = ueNodes.Get(i);

        // Try to get the most specific mobility model
        Ptr<MobilityModel> mobility = ue->GetObject<GaussMarkovMobilityModel>();
        if (!mobility)
        {
            mobility = ue->GetObject<WaypointMobilityModel>();
        }
        if (!mobility)
        {
            mobility = ue->GetObject<MobilityModel>();
        }
        if (!mobility)
        {
            continue;
        }

        Vector pos = mobility->GetPosition();
        Vector vel = mobility->GetVelocity();

        // Get IMSI from the first NR device
        uint64_t imsi = 0;
        for (uint32_t d = 0; d < ue->GetNDevices(); ++d)
        {
            Ptr<NrUeNetDevice> nrDev = DynamicCast<NrUeNetDevice>(ue->GetDevice(d));
            if (nrDev)
            {
                imsi = nrDev->GetImsi();
                break;
            }
        }

        uint16_t servingCell = g_ueCurrentServingCell.count(imsi) ? g_ueCurrentServingCell[imsi] : 0;
        uint16_t siteId = g_cellIdToSiteId.count(servingCell) ? g_cellIdToSiteId[servingCell] : 0;

        NS_LOG_UNCOND(Simulator::Now().GetSeconds()
                      << "s [UE " << i << "] IMSI=" << imsi
                      << " pos=(" << pos.x << ", " << pos.y << ")"
                      << " vel=" << vel.GetLength() << " m/s"
                      << " cell=" << servingCell << " site=" << siteId);
    }

    Simulator::Schedule(Seconds(10.0), &PrintUePosition, ueNodes);
}

//==============================================================================
// Remote Host Packet Monitor (for end-to-end latency measurement)
//==============================================================================

/**
 * @brief Callback to capture packets entering NS-3 from external Zenoh publisher
 *
 * This is the START point for end-to-end latency measurement.
 * Packets arrive from Docker via tap_remote -> ghost node -> Remote Host CSMA.
 */
bool
RemoteHostPacketMonitor(Ptr<NetDevice> device,
                        Ptr<const Packet> packet,
                        uint16_t protocol,
                        const Address& source,
                        const Address& destination,
                        NetDevice::PacketType packetType)
{
    // Only handle IP packets
    if (protocol != 0x0800)
    {
        return false;
    }

    // Extract Zenoh sequence number from payload pattern "[XXXX]"
    static uint32_t debugCount = 0;
    bool enableDebug = (debugCount < 20);
    debugCount++;

    auto seq = ExtractZenohPayloadSeq(packet, enableDebug);
    if (seq)
    {
        // Record send time at Remote Host (START of NS-3 path)
        // Use special node ID 0xFFFF for Remote Host
        ZenohLatencyTracker::GetInstance().RecordSend(*seq, ZenohLatencyTracker::NODE_REMOTE_HOST);
        NS_LOG_DEBUG("[REMOTE_HOST] Zenoh seq=" << *seq << " enters NS-3");
    }

    // Return false to allow normal packet processing
    return false;
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
 * @param ueNodes Container of UE nodes to log
 * @param interval Logging interval in seconds
 */
void
LogUeTrajectory(NodeContainer ueNodes, double interval)
{
    if (!g_trajectoryLoggingEnabled)
    {
        return;
    }

    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        Ptr<Node> ue = ueNodes.Get(i);

        // Try to get the most specific mobility model
        // (GaussMarkov if aggregated, otherwise generic MobilityModel)
        Ptr<MobilityModel> mobility = ue->GetObject<GaussMarkovMobilityModel>();
        if (!mobility)
        {
            mobility = ue->GetObject<WaypointMobilityModel>();
        }
        if (!mobility)
        {
            mobility = ue->GetObject<MobilityModel>();
        }
        if (!mobility)
        {
            continue;
        }

        Vector pos = mobility->GetPosition();
        Vector vel = mobility->GetVelocity();
        double speed = vel.GetLength();

        // Get IMSI from the first NR device
        uint64_t imsi = 0;
        for (uint32_t d = 0; d < ue->GetNDevices(); ++d)
        {
            Ptr<NrUeNetDevice> nrDev = DynamicCast<NrUeNetDevice>(ue->GetDevice(d));
            if (nrDev)
            {
                imsi = nrDev->GetImsi();
                break;
            }
        }

        uint16_t servingCell = g_ueCurrentServingCell.count(imsi) ? g_ueCurrentServingCell[imsi] : 0;
        uint16_t siteId = g_cellIdToSiteId.count(servingCell) ? g_cellIdToSiteId[servingCell] : 0;

        std::ostringstream oss;
        oss << std::fixed << std::setprecision(3)
            << Simulator::Now().GetSeconds() << ","
            << imsi << ","
            << pos.x << "," << pos.y << "," << pos.z << ","
            << vel.x << "," << vel.y << "," << vel.z << ","
            << speed << ","
            << servingCell << "," << siteId;
        g_trajectoryBuffer.push_back(oss.str());
    }

    // Schedule next logging
    Simulator::Schedule(Seconds(interval), &LogUeTrajectory, ueNodes, interval);
}

//==============================================================================
// Results Calculation and Output
//==============================================================================

/**
 * @brief Calculate and write simulation results to file
 *
 * Calculates:
 * - Average latency, min/max latency
 * - Packet loss rate (based on unmatched sent packets)
 * - SLA violation rate
 * - Handover statistics (count, duration, success rate)
 *
 * Output format matches experiment_data/mec_hex_XXX/latency.csv and traj.csv
 */
void
WriteSimulationResults(const std::string& outputPath, double simTime, double slaThresholdMs)
{
    // Get latency statistics from ZenohLatencyTracker
    auto& tracker = ZenohLatencyTracker::GetInstance();
    uint64_t receivedPackets = tracker.GetTotalPackets();
    uint64_t slaViolations = tracker.GetSlaViolations();
    double avgLatency = tracker.GetAvgLatency();
    double minLatency = tracker.GetMinLatency();
    double maxLatency = tracker.GetMaxLatency();

    // Calculate packet loss (pending packets at end of simulation are considered lost)
    uint64_t lostPackets = tracker.GetPendingPackets();
    uint64_t totalPacketsSent = receivedPackets + lostPackets;
    double packetLossRate = totalPacketsSent > 0 ?
        (100.0 * lostPackets / totalPacketsSent) : 0.0;

    // Handover statistics
    uint32_t totalHandovers = g_handoverEvents.size();
    uint32_t successfulHandovers = 0;
    uint32_t interSiteHandovers = 0;
    uint32_t intraSiteHandovers = 0;
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
        if (event.isInterSite)
        {
            interSiteHandovers++;
        }
        else
        {
            intraSiteHandovers++;
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

    // CSV format section for easy parsing
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
    resultsFile << "\n";

    resultsFile << "[HANDOVER]\n";
    resultsFile << "metric,value\n";
    resultsFile << "total_handovers," << totalHandovers << "\n";
    resultsFile << "successful_handovers," << successfulHandovers << "\n";
    resultsFile << "failed_handovers," << (totalHandovers - successfulHandovers) << "\n";
    resultsFile << "handover_success_rate_percent," << std::fixed << std::setprecision(2) << handoverSuccessRate << "\n";
    resultsFile << "inter_site_handovers," << interSiteHandovers << "\n";
    resultsFile << "intra_site_handovers," << intraSiteHandovers << "\n";
    if (successfulHandovers > 0)
    {
        resultsFile << "avg_handover_duration_ms," << std::fixed << std::setprecision(3) << avgHandoverDuration << "\n";
        resultsFile << "min_handover_duration_ms," << std::fixed << std::setprecision(3) << minHandoverDuration << "\n";
        resultsFile << "max_handover_duration_ms," << std::fixed << std::setprecision(3) << maxHandoverDuration << "\n";
    }
    resultsFile << "\n";

    // Detailed handover events
    resultsFile << "[HANDOVER_EVENTS]\n";
    resultsFile << "start_time_s,end_time_s,duration_ms,imsi,source_cell,target_cell,source_site,target_site,is_inter_site,success\n";
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
                    << event.sourceSite << ","
                    << event.targetSite << ","
                    << (event.isInterSite ? 1 : 0) << ","
                    << (event.success ? 1 : 0) << "\n";
    }

    // Get lost packet details for correlation with connected sites
    auto lostPacketDetails = tracker.GetLostPacketDetails();

    // Build site connection periods from handover events
    struct SiteConnectionPeriod {
        uint16_t siteId;
        double startTimeMs;
        double endTimeMs;
        int packetsLost;
    };
    std::vector<SiteConnectionPeriod> connectionPeriods;

    double simEndMs = simTime * 1000.0;

    if (g_handoverEvents.empty())
    {
        // No handovers - single period for entire simulation (site unknown, use 0)
        connectionPeriods.push_back({0, 0.0, simEndMs, 0});
    }
    else
    {
        // First period: simulation start to first handover
        connectionPeriods.push_back({
            g_handoverEvents[0].sourceSite,
            0.0,
            g_handoverEvents[0].startTime.GetMilliSeconds(),
            0
        });

        // Intermediate periods: between handovers
        for (size_t i = 0; i < g_handoverEvents.size(); ++i)
        {
            const auto& ho = g_handoverEvents[i];
            double periodStart = ho.endTime.GetMilliSeconds();
            double periodEnd = (i + 1 < g_handoverEvents.size())
                ? g_handoverEvents[i + 1].startTime.GetMilliSeconds()
                : simEndMs;

            connectionPeriods.push_back({
                ho.targetSite,
                periodStart,
                periodEnd,
                0
            });
        }
    }

    // Classify each lost packet by connected site and store details
    struct LostPacketInfo {
        uint16_t siteId;
        uint32_t sn;
        double timestampMs;
    };
    std::vector<LostPacketInfo> lostPacketsBySite;

    for (const auto& loss : lostPacketDetails)
    {
        double sendTimeMs = loss.first;
        uint32_t sn = loss.second;
        for (auto& period : connectionPeriods)
        {
            if (sendTimeMs >= period.startTimeMs && sendTimeMs < period.endTimeMs)
            {
                period.packetsLost++;
                lostPacketsBySite.push_back({period.siteId, sn, sendTimeMs});
                break;
            }
        }
    }

    // Output per-site connection periods with loss counts
    resultsFile << "\n[SITE_CONNECTION]\n";
    resultsFile << "site_id,connect_start_s,connect_end_s,packets_lost\n";
    for (const auto& period : connectionPeriods)
    {
        resultsFile << period.siteId << ","
                    << std::fixed << std::setprecision(3)
                    << (period.startTimeMs / 1000.0) << ","
                    << (period.endTimeMs / 1000.0) << ","
                    << period.packetsLost << "\n";
    }

    // List each lost packet with site, sequence number, and timestamp
    resultsFile << "\n[LOST_PACKETS]\n";
    resultsFile << "site_id,sn,timestamp_ms\n";
    for (const auto& pkt : lostPacketsBySite)
    {
        resultsFile << pkt.siteId << ","
                    << pkt.sn << ","
                    << std::fixed << std::setprecision(3)
                    << pkt.timestampMs << "\n";
    }
    resultsFile << "total,," << lostPacketDetails.size() << "\n";

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
    NS_LOG_UNCOND("    Inter-Site:        " << interSiteHandovers);
    NS_LOG_UNCOND("    Intra-Site:        " << intraSiteHandovers);
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
 * @param params Map of parameter names to values
 */
void
WriteExperimentConfig(const std::string& outputPath,
                      uint8_t numRings,
                      const std::string& scenario,
                      double isd,
                      uint32_t numUes,
                      double maxUeDistance,
                      bool extendedCoverage,
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
                      bool enableTap,
                      const std::string& hoAlgorithmType,
                      double hoHysteresis,
                      double hoTimeToTriggerMs,
                      double handoverInterruptMs,
                      double wanDelayMs,
                      double slaThresholdMs,
                      uint16_t sourceEdgeNodeId,
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

    // Get current time for header
    time_t now = time(nullptr);
    char timeBuffer[64];
    strftime(timeBuffer, sizeof(timeBuffer), "%Y-%m-%d %H:%M:%S", localtime(&now));

    configFile << "# Experiment Configuration\n";
    configFile << "# Generated: " << timeBuffer << "\n\n";

    // Use fixed-point format with appropriate precision for readability
    configFile << std::fixed;

    configFile << "[TOPOLOGY]\n";
    configFile << "num_rings=" << (int)numRings << "\n";
    configFile << "scenario=" << scenario << "\n";
    configFile << std::setprecision(1) << "isd_m=" << isd << "\n";
    configFile << "num_ues=" << numUes << "\n";
    configFile << "max_ue_distance_m=" << maxUeDistance << "\n";
    configFile << "extended_coverage=" << (extendedCoverage ? "true" : "false") << "\n";

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

    configFile << "\n[HANDOVER_ALGORITHM]\n";
    configFile << "algorithm_type=" << hoAlgorithmType << "\n";
    configFile << "hysteresis_db=" << hoHysteresis << "\n";
    configFile << "time_to_trigger_ms=" << hoTimeToTriggerMs << "\n";

    configFile << "\n[HANDOVER]\n";
    configFile << "handover_interrupt_ms=" << handoverInterruptMs << "\n";
    configFile << "wan_delay_ms=" << wanDelayMs << "\n";

    configFile << "\n[MEASUREMENT]\n";
    configFile << "sla_threshold_ms=" << slaThresholdMs << "\n";
    configFile << "source_edge_node_id=" << sourceEdgeNodeId << "\n";
    configFile << std::setprecision(2) << "trajectory_log_interval_s=" << trajectoryLogInterval << "\n";

    configFile << "\n[TAP_BRIDGE]\n";
    configFile << "enable_tap=" << (enableTap ? "true" : "false") << "\n";

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

//==============================================================================
// Main Function
//==============================================================================

int
main(int argc, char* argv[])
{
    // Topology parameters
    uint8_t numRings = 1;                        // Number of hexagonal rings (1 = 7 sites)
    std::string scenario = "UMa";                // Propagation scenario (UMa, RMa, UMi)
    double isd = 500.0;                          // Inter-site distance in meters
    uint32_t numUes = 1;                         // Number of UEs
    double maxUeDistance = 1000.0;               // Max UE distance to closest site
    bool extendedCoverage = false;               // Extended coverage: outer ring shares inner edges

    // Mobility parameters
    std::string mobilityModel = "linear";        // linear, random, gauss-markov, waypoint
    double ueSpeed = 10.0;                       // UE speed in m/s (mean for random)
    double ueSpeedVariance = 2.0;                // Variance for Gaussian velocity (m/s)^2
    double gaussAlpha = 0.85;                    // Gauss-Markov memory factor (0=random, 1=linear)
    double gaussTimeStep = 0.5;                  // Gauss-Markov update interval (seconds)
    std::string waypointFile = "";               // NS-2 format trace file (empty = use built-in path)
    std::string builtinPath = "linear-y";        // Built-in paths: hexagonal, linear-y, zigzag, highway, urban-grid, kaohsiung, l-corner, u-corner
    double simTime = 300.0;                      // Simulation time in seconds

    // NR parameters
    double centralFrequency = 3.5e9;             // 3.5 GHz (n78 band)
    double bandwidth = 20e6;                     // 20 MHz
    double gnbTxPower = 50.0;                    // gNB TX power in dBm (increased for omnidirectional)
    uint16_t numerology = 1;                     // NR numerology

    // Tap bridge parameters
    bool enableTap = false;
    std::string tapUeDevice = "tap_ue";
    std::string tapEdgePrefix = "tap_edge";

    // Handover prediction
    std::string predictionFilePath = "/tmp/ns3_handover/ns3_handover";
    double predictionWriteIntervalMs = 100.0;

    // Experiment output directory
    std::string experimentDir = "";              // Empty = auto-generate timestamped directory

    // Results output (set automatically if experimentDir is used)
    std::string resultsOutputPath = "";          // Empty = use experimentDir, path = override
    std::string latencyOutputPath = "";          // Empty = use experimentDir, path = override
    std::string trajectoryOutputPath = "";       // Empty = use experimentDir, path = override
    double trajectoryLogInterval = 0.5;          // Logging interval in seconds

    // Latency measurement
    double slaThresholdMs = 50.0;
    uint16_t sourceEdgeNodeId = 5;  // Edge node that publishes critical data

    // Remote Host (traffic source / controller)
    bool enableRemoteHost = true;
    std::string tapRemoteHostDevice = "tap_remote";
    double wanDelayMs = 20.0;        // WAN delay in milliseconds (one-way)
    std::string wanDataRate = "1Gbps";  // WAN link data rate
    uint16_t remoteHostEdgeId = 5;   // Edge server to connect remote host to

    // Handover blackout model
    double handoverInterruptMs = 30.0;  // Handover interruption time (realistic: 30-100ms)

    // Handover algorithm parameters (computed based on bandwidth, used for config output)
    std::string hoAlgorithmType = "NrA3RsrpHandoverAlgorithm";
    double hoHysteresis = 3.0;          // Will be updated based on bandwidth
    double hoTimeToTriggerMs = 256.0;   // Will be updated based on bandwidth

    // Other
    bool logging = true;

    // Background UE parameters for cell load
    uint32_t numBackgroundUes = 0;           // Number of stationary background UEs (0 = disabled)
    double bgUeTrafficMbps = 20.0;            // Traffic rate per background UE in Mbps
    uint32_t bgUePacketSize = 500;          // Background UE packet size in bytes
    double bgTrafficStartDelay = 0.5;        // Delay before starting background traffic (seconds)

    // Capacity testing parameters
    double loadPercent = 0.0;                // Target load as % of capacity (0 = use numBackgroundUes directly)
    double capacityMbps = 0.0;               // Per-cell capacity in Mbps (0 = auto-estimate)
    bool saturationTest = false;             // Run saturation test to find max capacity

    CommandLine cmd(__FILE__);

    // Topology
    cmd.AddValue("numRings", "Number of hexagonal rings (0=1 site, 1=7 sites, 2=13 sites)", numRings);
    cmd.AddValue("scenario", "Propagation scenario (UMa, RMa, UMi)", scenario);
    cmd.AddValue("isd", "Inter-site distance in meters", isd);
    cmd.AddValue("numUes", "Number of UEs", numUes);
    cmd.AddValue("maxUeDistance", "Max UE distance to closest site", maxUeDistance);
    cmd.AddValue("extendedCoverage", "Extended coverage: outer ring sites share inner edge servers", extendedCoverage);

    // Mobility
    cmd.AddValue("mobilityModel", "UE mobility model (linear, random, gauss-markov, waypoint)", mobilityModel);
    cmd.AddValue("ueSpeed", "UE speed in m/s (mean for random distributions)", ueSpeed);
    cmd.AddValue("ueSpeedVariance", "Velocity variance for Gaussian distribution", ueSpeedVariance);
    cmd.AddValue("gaussAlpha", "Gauss-Markov memory factor (0-1)", gaussAlpha);
    cmd.AddValue("gaussTimeStep", "Gauss-Markov update interval in seconds", gaussTimeStep);
    cmd.AddValue("waypointFile", "NS-2 format mobility trace file path", waypointFile);
    cmd.AddValue("builtinPath", "Built-in path: hexagonal, linear-y, zigzag, highway, urban-grid, kaohsiung, l-corner, u-corner", builtinPath);
    cmd.AddValue("simTime", "Simulation time in seconds", simTime);

    // NR
    cmd.AddValue("centralFrequency", "Central frequency in Hz", centralFrequency);
    cmd.AddValue("bandwidth", "System bandwidth in Hz", bandwidth);
    cmd.AddValue("gnbTxPower", "gNB TX power in dBm", gnbTxPower);
    cmd.AddValue("numerology", "NR numerology (0-4)", numerology);

    // Tap bridge
    cmd.AddValue("enableTap", "Enable tap bridge for external connectivity", enableTap);
    cmd.AddValue("tapUeDevice", "Tap device name prefix for UE", tapUeDevice);
    cmd.AddValue("tapEdgePrefix", "Tap device name prefix for edge servers", tapEdgePrefix);

    // Handover prediction
    cmd.AddValue("predictionFilePath", "Path for handover prediction file", predictionFilePath);
    cmd.AddValue("predictionWriteIntervalMs", "Min interval between prediction writes (ms)", predictionWriteIntervalMs);

    // Experiment output directory
    cmd.AddValue("experimentDir", "Experiment output directory (empty=auto-generate with timestamp)", experimentDir);

    // Output paths (override experimentDir if specified)
    cmd.AddValue("latencyOutputPath", "Override latency CSV path (default: experimentDir/latency.csv)", latencyOutputPath);
    cmd.AddValue("trajectoryOutputPath", "Override trajectory CSV path (default: experimentDir/traj.csv)", trajectoryOutputPath);
    cmd.AddValue("resultsOutputPath", "Override results summary path (default: experimentDir/result.csv)", resultsOutputPath);
    cmd.AddValue("trajectoryLogInterval", "Trajectory logging interval in seconds", trajectoryLogInterval);

    // Latency measurement
    cmd.AddValue("slaThresholdMs", "SLA threshold in milliseconds", slaThresholdMs);
    cmd.AddValue("sourceEdgeNodeId", "Edge node ID that publishes critical data", sourceEdgeNodeId);

    // Remote Host
    cmd.AddValue("enableRemoteHost", "Enable remote host (controller) node", enableRemoteHost);
    cmd.AddValue("tapRemoteHostDevice", "Tap device name for remote host", tapRemoteHostDevice);
    cmd.AddValue("wanDelayMs", "WAN link delay in milliseconds (one-way)", wanDelayMs);
    cmd.AddValue("wanDataRate", "WAN link data rate (e.g., 1Gbps)", wanDataRate);
    cmd.AddValue("remoteHostEdgeId", "Edge server ID to connect remote host to", remoteHostEdgeId);

    // Handover blackout model
    cmd.AddValue("handoverInterruptMs", "Handover interruption time in ms (realistic: 30-100ms)", handoverInterruptMs);

    // Other
    cmd.AddValue("logging", "Enable detailed logging", logging);

    // Background UEs for cell load
    cmd.AddValue("numBackgroundUes", "Number of stationary background UEs for cell load", numBackgroundUes);
    cmd.AddValue("bgUeTrafficMbps", "Traffic rate per background UE in Mbps", bgUeTrafficMbps);
    cmd.AddValue("bgUePacketSize", "Background UE packet size in bytes", bgUePacketSize);
    cmd.AddValue("bgTrafficStartDelay", "Delay before starting background traffic in seconds (allows TAP connections to establish)", bgTrafficStartDelay);

    // Capacity testing
    cmd.AddValue("loadPercent", "Target load as % of capacity (0=use numBackgroundUes)", loadPercent);
    cmd.AddValue("capacityMbps", "Per-cell capacity in Mbps (0=auto-estimate)", capacityMbps);
    cmd.AddValue("saturationTest", "Run saturation test to find max capacity", saturationTest);

    cmd.Parse(argc, argv);

    // Update global handover interrupt parameters
    g_handoverInterruptMs = handoverInterruptMs;
    g_networkDelayMs = wanDelayMs;  // Used to calculate packet arrival at gNB

    // Auto-estimate per-cell capacity from bandwidth (~3 Mbps per MHz)
    if (capacityMbps <= 0.0)
    {
        capacityMbps = (bandwidth / 1e6) * 3.0;
    }

    // Set handover algorithm parameters based on bandwidth (matches config at line ~1800)
    if (bandwidth == 5e6)
    {
        hoHysteresis = 5.0;
        hoTimeToTriggerMs = 480.0;
    }
    else
    {
        hoHysteresis = 3.0;
        hoTimeToTriggerMs = 256.0;
    }

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

    // Write experiment configuration
    WriteExperimentConfig(configOutputPath,
                          numRings,
                          scenario,
                          isd,
                          numUes,
                          maxUeDistance,
                          extendedCoverage,
                          mobilityModel,
                          ueSpeed,
                          ueSpeedVariance,
                          gaussAlpha,
                          gaussTimeStep,
                          waypointFile,
                          builtinPath,
                          simTime,
                          centralFrequency,
                          bandwidth,
                          gnbTxPower,
                          numerology,
                          enableTap,
                          hoAlgorithmType,
                          hoHysteresis,
                          hoTimeToTriggerMs,
                          handoverInterruptMs,
                          wanDelayMs,
                          slaThresholdMs,
                          sourceEdgeNodeId,
                          trajectoryLogInterval,
                          numBackgroundUes,
                          bgUeTrafficMbps,
                          bgUePacketSize,
                          loadPercent,
                          capacityMbps);

    NS_LOG_UNCOND("Experiment directory: " << experimentDir);

    // Initialize handover prediction file system
    HandoverPredictionFile::GetInstance().Initialize(
        predictionFilePath,
        MilliSeconds(predictionWriteIntervalMs));

    // Initialize Zenoh latency measurement
    ZenohLatencyTracker::GetInstance().Initialize(
        latencyOutputPath,
        slaThresholdMs,
        sourceEdgeNodeId);

    // Configure real-time simulator when tap bridge is enabled
    if (enableTap)
    {
        GlobalValue::Bind("SimulatorImplementationType", StringValue("ns3::RealtimeSimulatorImpl"));
        GlobalValue::Bind("ChecksumEnabled", BooleanValue(true));
    }

    if (logging)
    {
        LogComponentEnable("NrMec3gppCalibration", LOG_LEVEL_INFO);
    }

    NS_LOG_INFO("==============================================");
    NS_LOG_INFO("NR MEC 3GPP Calibration Simulation");
    NS_LOG_INFO("==============================================");
    NS_LOG_INFO("Rings: " << (int)numRings << " (sites will be determined by scenario helper)");
    NS_LOG_INFO("Scenario: " << scenario);
    NS_LOG_INFO("ISD: " << isd << " m");
    NS_LOG_INFO("UEs: " << numUes);
    NS_LOG_INFO("Mobility: " << mobilityModel << " at " << ueSpeed << " m/s");
    NS_LOG_INFO("Handover Blackout: " << handoverInterruptMs << " ms (WAN=" << wanDelayMs << "ms)");
    NS_LOG_INFO("==============================================");

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
    gridScenario.SetUtNumber(numUes);
    gridScenario.SetMaxUeDistanceToClosestSite(maxUeDistance);

    // Calculate grid bounds for mobility models
    double gridRadius = isd * (numRings + 1);

    if (mobilityModel == "linear")
    {
        gridScenario.CreateScenarioWithMobility(Vector(ueSpeed, 0, 0), 0.0);
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

        // Create scenario with custom mobility (no aggregation issues)
        gridScenario.CreateScenarioWithCustomMobility(waypointHelper);

        NodeContainer ueNodesTemp = gridScenario.GetUserTerminals();

        if (!waypointFile.empty())
        {
            // Load from NS-2 format file
            Ns2MobilityHelper ns2(waypointFile);
            ns2.Install();
            NS_LOG_INFO("Waypoint mobility loaded from file: " << waypointFile);
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
        NS_LOG_INFO("Random waypoint mobility with Normal velocity distribution");
    }
    else
    {
        NS_FATAL_ERROR("Unknown mobilityModel: " << mobilityModel);
    }

    NodeContainer gnbNodes = gridScenario.GetBaseStations();
    NodeContainer ueNodes = gridScenario.GetUserTerminals();
    uint32_t numSites = gridScenario.GetNumSites();
    uint32_t numCells = gnbNodes.GetN();

    g_scenario = &gridScenario;

    NS_LOG_INFO("Created " << numSites << " sites with " << numCells << " cells");
    NS_LOG_INFO("Created " << ueNodes.GetN() << " UEs");

    // Calculate numBackgroundUes based on total network capacity (per-cell load)
    if (loadPercent > 0.0)
    {
        double totalNetworkCapacity = numCells * capacityMbps;
        double targetTotalLoad = totalNetworkCapacity * (loadPercent / 100.0);
        numBackgroundUes = static_cast<uint32_t>(
            std::ceil(targetTotalLoad / bgUeTrafficMbps));

        NS_LOG_UNCOND("\n--- Capacity Testing Mode ---");
        NS_LOG_UNCOND("Per-cell capacity: " << capacityMbps << " Mbps");
        NS_LOG_UNCOND("Total cells: " << numCells);
        NS_LOG_UNCOND("Total network capacity: " << totalNetworkCapacity << " Mbps");
        NS_LOG_UNCOND("Target per-cell load: " << loadPercent << "%");
        NS_LOG_UNCOND("Total load needed: " << targetTotalLoad << " Mbps");
        NS_LOG_UNCOND("Background UEs: " << numBackgroundUes
                      << " (each " << bgUeTrafficMbps << " Mbps)");
    }

    // Create background UE nodes for cell load
    NodeContainer backgroundUeNodes;
    if (numBackgroundUes > 0)
    {
        backgroundUeNodes.Create(numBackgroundUes);

        // Compute site positions (hexagonal pattern, same as gridScenario)
        std::vector<Vector> sitePositions;
        sitePositions.push_back(Vector(0, 0, 0));  // Center site
        double siteRadius = isd;
        for (uint32_t ring = 1; ring <= numRings; ring++)
        {
            for (uint32_t i = 0; i < 6 * ring; i++)
            {
                double siteAngle = M_PI / 6 + (2.0 * M_PI * i) / (6 * ring);
                sitePositions.push_back(Vector(siteRadius * ring * cos(siteAngle),
                                               siteRadius * ring * sin(siteAngle),
                                               0));
            }
        }

        // Position background UEs statically across cell sites
        Ptr<ListPositionAllocator> bgPosAlloc = CreateObject<ListPositionAllocator>();
        double ueOffset = 50.0;  // Distance from site center
        for (uint32_t i = 0; i < numBackgroundUes; i++)
        {
            uint32_t siteIdx = i % sitePositions.size();
            Vector sitePos = sitePositions[siteIdx];
            // Place UEs around site center with angular offset
            double offsetAngle = 2.0 * M_PI * (i / sitePositions.size()) / 3.0;
            Vector pos(sitePos.x + ueOffset * cos(offsetAngle),
                       sitePos.y + ueOffset * sin(offsetAngle),
                       1.5);
            bgPosAlloc->Add(pos);
        }

        MobilityHelper bgMobility;
        bgMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
        bgMobility.SetPositionAllocator(bgPosAlloc);
        bgMobility.Install(backgroundUeNodes);

        NS_LOG_INFO("Created " << numBackgroundUes << " background UE(s) distributed across "
                    << sitePositions.size() << " sites");
    }

    // Special positioning for "linear" mode only
    // Other mobility models (gauss-markov, waypoint, random) handle their own positioning
    //
    // Site positions (1 ring, ISD=500m):
    //   Site 0: (0, 0)         - center
    //   Site 1: (433, 250)     - upper right
    //   Site 2: (0, 500)       - top
    //   Site 3: (-433, 250)    - upper left
    //   Site 4: (-433, -250)   - lower left
    //   Site 5: (0, -500)      - bottom
    //   Site 6: (433, -250)    - lower right
    if (ueNodes.GetN() > 0 && mobilityModel == "linear")
    {
        Ptr<ConstantVelocityMobilityModel> firstUeMobility =
            ueNodes.Get(0)->GetObject<ConstantVelocityMobilityModel>();
        if (firstUeMobility)
        {
            // Start slightly offset from center, move toward Site 2 (at 0, 500)
            // This will enter Site 2's Sector 2 (pointing 270°, i.e., downward)
            firstUeMobility->SetPosition(Vector(0.0, -200.0, 1.5));
            // Move purely in +Y direction toward Site 2
            firstUeMobility->SetVelocity(Vector(0.0, ueSpeed, 0.0));
            NS_LOG_INFO("First UE positioned at (0, 200, 1.5) moving toward Site 2 at "
                        << ueSpeed << " m/s (vx=0, vy=" << ueSpeed << ")");
        }
    }

    //--------------------------------------------------------------------------
    // Set up NR network
    //--------------------------------------------------------------------------
    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper> idealBeamformingHelper = CreateObject<IdealBeamformingHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();

    nrHelper->SetBeamformingHelper(idealBeamformingHelper);
    nrHelper->SetEpcHelper(epcHelper);

    // Configure handover algorithm
    nrHelper->SetHandoverAlgorithmType("ns3::NrA3RsrpHandoverAlgorithm");
    nrHelper->SetHandoverAlgorithmAttribute("Hysteresis", DoubleValue(hoHysteresis));
    nrHelper->SetHandoverAlgorithmAttribute("TimeToTrigger", TimeValue(MilliSeconds(hoTimeToTriggerMs)));

    // Spectrum configuration (single band, overlapping)
    BandwidthPartInfoPtrVector allBwps;
    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(centralFrequency, bandwidth, 1);
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);

    // Channel model
    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigureFactories(scenario, "Default", "ThreeGpp");
    channelHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));
    channelHelper->AssignChannelsToBands({band});
    allBwps = CcBwpCreator::GetAllBwps({band});

    // Configure antennas (omnidirectional with moderate array gain for single-sector)
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

    // Disable RLC retransmission for realistic handover interruption
    // RLC_UM_ALWAYS: Unacknowledged Mode - packets are dropped if not delivered, no buffering
    // This makes handover interruption visible as packet loss rather than hidden by retransmission
    Config::SetDefault("ns3::NrGnbRrc::EpsBearerToRlcMapping",
                       EnumValue(NrGnbRrc::RLC_UM_ALWAYS));

    // Increase RLC UM buffer size to prevent packet drops under load
    // Default is 10KB which causes TAP bridge connections to fail when background traffic is present
    Config::SetDefault("ns3::NrRlcUm::MaxTxBufferSize", UintegerValue(1024 * 1024));  // 1 MB

    // Install NR devices
    NetDeviceContainer gnbNetDevs = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    NetDeviceContainer ueNetDevs = nrHelper->InstallUeDevice(ueNodes, allBwps);

    // Install NR on background UEs
    NetDeviceContainer bgUeNetDevs;
    if (numBackgroundUes > 0)
    {
        bgUeNetDevs = nrHelper->InstallUeDevice(backgroundUeNodes, allBwps);

        // Update config for background UE antenna
        for (auto it = bgUeNetDevs.Begin(); it != bgUeNetDevs.End(); ++it)
        {
            DynamicCast<NrUeNetDevice>(*it)->UpdateConfig();
        }
        NS_LOG_INFO("Installed NR on " << bgUeNetDevs.GetN() << " background UEs");
    }

    // Configure gNB TX power and numerology
    for (uint32_t i = 0; i < gnbNetDevs.GetN(); ++i)
    {
        nrHelper->GetGnbPhy(gnbNetDevs.Get(i), 0)->SetAttribute("Numerology", UintegerValue(numerology));
        nrHelper->GetGnbPhy(gnbNetDevs.Get(i), 0)->SetAttribute("TxPower", DoubleValue(gnbTxPower));
    }

    //--------------------------------------------------------------------------
    // Build cell ID to site ID mapping
    // NOTE: Only map by actual NR cell ID, not node index, to avoid collisions
    // NR assigns cell IDs as 1, 3, 5, 7, ... (incrementing by 2 due to CsgId)
    // Node indices 0-20 would collide with actual cell IDs if both were used as keys
    //--------------------------------------------------------------------------
    for (uint32_t nodeIdx = 0; nodeIdx < numCells; ++nodeIdx)
    {
        uint16_t siteId = gridScenario.GetSiteIndex(nodeIdx);

        // Get actual cell ID from the device (this is what RRC/handover uses)
        Ptr<NrGnbNetDevice> gnbNetDevice = gnbNetDevs.Get(nodeIdx)->GetObject<NrGnbNetDevice>();
        uint16_t actualCellId = gnbNetDevice->GetCellId();
        g_cellIdToSiteId[actualCellId] = siteId;

        NS_LOG_INFO("Node " << nodeIdx << " (cellId=" << actualCellId << ") -> Site " << siteId);
    }

    //--------------------------------------------------------------------------
    // Set up IP networking for UEs
    //--------------------------------------------------------------------------
    InternetStackHelper internet;
    internet.Install(ueNodes);

    Ipv4InterfaceContainer ueIpIfaces = epcHelper->AssignUeIpv4Address(ueNetDevs);

    // Set default route for UEs
    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        Ptr<Ipv4StaticRouting> ueStaticRouting =
            ipv4RoutingHelper.GetStaticRouting(ueNodes.Get(i)->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
    }

    // Install IP stack on background UEs
    Ipv4InterfaceContainer bgUeIpIfaces;
    if (numBackgroundUes > 0)
    {
        internet.Install(backgroundUeNodes);
        bgUeIpIfaces = epcHelper->AssignUeIpv4Address(bgUeNetDevs);

        // Set default route for background UEs
        for (uint32_t i = 0; i < backgroundUeNodes.GetN(); ++i)
        {
            Ptr<Ipv4StaticRouting> ueStaticRouting =
                ipv4RoutingHelper.GetStaticRouting(backgroundUeNodes.Get(i)->GetObject<Ipv4>());
            ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
        }

        // Attach background UEs to closest gNB
        nrHelper->AttachToClosestGnb(bgUeNetDevs, gnbNetDevs);
        NS_LOG_INFO("Background UEs attached to network");
    }

    //--------------------------------------------------------------------------
    // Set up edge servers
    // Normal mode: one edge server per site
    // Extended coverage: only inner 7 sites get edge servers, outer ring shares
    //--------------------------------------------------------------------------
    Ptr<Node> pgw = epcHelper->GetPgwNode();

    // For extended coverage with numRings>=2, only create edge servers for inner sites
    uint32_t numEdgeServers = numSites;
    if (extendedCoverage && numRings >= 2)
    {
        numEdgeServers = 7;  // Inner ring (center + 6 surrounding sites)
        NS_LOG_UNCOND("Extended coverage enabled: " << numEdgeServers
                      << " edge servers for " << numSites << " sites");
    }

    NodeContainer edgeServerNodes;
    edgeServerNodes.Create(numEdgeServers);
    NodeContainer ghostNodes;
    ghostNodes.Create(numEdgeServers);

    internet.Install(edgeServerNodes);
    // Ghost nodes don't need IP stack - they're pure L2 bridges

    std::vector<Ptr<NetDevice>> edgeServerOuterDevices;
    std::vector<Ptr<NetDevice>> edgeServerInnerDevices;
    std::vector<Ptr<NetDevice>> ghostDevices;
    std::vector<Ipv4Address> edgeServerIps;  // Store IPs for outer ring mapping

    for (uint32_t siteId = 0; siteId < numEdgeServers; ++siteId)
    {
        //----------------------------------------------------------------------
        // P2P: PGW <-> EdgeServer
        //----------------------------------------------------------------------
        PointToPointHelper p2pPgwEdge;
        p2pPgwEdge.SetDeviceAttribute("DataRate", StringValue("10Gbps"));
        p2pPgwEdge.SetChannelAttribute("Delay", TimeValue(MicroSeconds(1)));  // Co-located edge

        NodeContainer pgwEdgeLink;
        pgwEdgeLink.Add(pgw);
        pgwEdgeLink.Add(edgeServerNodes.Get(siteId));

        NetDeviceContainer pgwEdgeDevices = p2pPgwEdge.Install(pgwEdgeLink);

        // IP: 10.{siteId+1}.0.0/24
        std::ostringstream subnet;
        subnet << "10." << (siteId + 1) << ".0.0";
        Ipv4AddressHelper ipv4Helper;
        ipv4Helper.SetBase(subnet.str().c_str(), "255.255.255.0");

        Ipv4InterfaceContainer pgwEdgeIpIfaces = ipv4Helper.Assign(pgwEdgeDevices);
        Ipv4Address edgeServerIp = pgwEdgeIpIfaces.GetAddress(1);
        edgeServerIps.push_back(edgeServerIp);  // Store for outer ring mapping

        edgeServerOuterDevices.push_back(pgwEdgeDevices.Get(1));

        //----------------------------------------------------------------------
        // CSMA: EdgeServer <-> GhostNode (for tap bridge)
        //----------------------------------------------------------------------
        CsmaHelper csmaEdgeGhost;
        csmaEdgeGhost.SetChannelAttribute("DataRate", DataRateValue(DataRate("10Gbps")));
        csmaEdgeGhost.SetChannelAttribute("Delay", TimeValue(MicroSeconds(0)));  // Virtual link to TapBridge

        NodeContainer edgeGhostLink;
        edgeGhostLink.Add(edgeServerNodes.Get(siteId));
        edgeGhostLink.Add(ghostNodes.Get(siteId));

        NetDeviceContainer edgeGhostDevices = csmaEdgeGhost.Install(edgeGhostLink);

        // IP: 10.{siteId+1}.1.0/24 for EdgeServer only
        std::ostringstream ghostSubnet;
        ghostSubnet << "10." << (siteId + 1) << ".1.0";
        Ipv4AddressHelper ghostIpHelper;
        ghostIpHelper.SetBase(ghostSubnet.str().c_str(), "255.255.255.0");

        ghostIpHelper.Assign(edgeGhostDevices.Get(0));  // EdgeServer only

        edgeServerInnerDevices.push_back(edgeGhostDevices.Get(0));
        ghostDevices.push_back(edgeGhostDevices.Get(1));

        // Set up routing from edge server to UE network (via PGW)
        Ptr<Ipv4StaticRouting> edgeRouting =
            ipv4RoutingHelper.GetStaticRouting(edgeServerNodes.Get(siteId)->GetObject<Ipv4>());
        edgeRouting->SetDefaultRoute(pgwEdgeIpIfaces.GetAddress(0), 1);

        // Add route on PGW for ghost segment
        Ptr<Ipv4StaticRouting> pgwRouting =
            ipv4RoutingHelper.GetStaticRouting(pgw->GetObject<Ipv4>());
        pgwRouting->AddNetworkRouteTo(
            Ipv4Address(ghostSubnet.str().c_str()),
            Ipv4Mask("255.255.255.0"),
            pgwEdgeIpIfaces.GetAddress(1),
            pgw->GetObject<Ipv4>()->GetInterfaceForAddress(pgwEdgeIpIfaces.GetAddress(0))
        );

        // Map this site's cell to this edge server (1 cell per site)
        // NOTE: Only map by actual NR cell ID to avoid collisions with node indices
        uint32_t nodeIdx = siteId;  // 1 cell per site
        if (nodeIdx < numCells)
        {
            // Get actual cell ID from the device
            Ptr<NrGnbNetDevice> gnbNetDevice = gnbNetDevs.Get(nodeIdx)->GetObject<NrGnbNetDevice>();
            uint16_t actualCellId = gnbNetDevice->GetCellId();
            g_cellIdToEdgeServer[actualCellId] = edgeServerNodes.Get(siteId);
            g_cellIdToEdgeServerIp[actualCellId] = edgeServerIp;
        }

        NS_LOG_INFO("Edge Server " << siteId << " (IP: " << edgeServerIp
                    << ") serves Site " << siteId << " (cell " << siteId << ")");
    }

    //--------------------------------------------------------------------------
    // Extended coverage: Map outer ring cells to inner edge servers
    // Sector-based mapping: outer sites 7-18 map to inner sites 1-6
    //   Sites 7-8 → Edge 1, Sites 9-10 → Edge 2, Sites 11-12 → Edge 3
    //   Sites 13-14 → Edge 4, Sites 15-16 → Edge 5, Sites 17-18 → Edge 6
    //--------------------------------------------------------------------------
    if (extendedCoverage && numRings >= 2 && numSites > numEdgeServers)
    {
        for (uint32_t outerSiteId = numEdgeServers; outerSiteId < numSites; ++outerSiteId)
        {
            // Map outer site to nearest inner site's edge (sector-based)
            // Sites 7-8 → Edge 1, Sites 9-10 → Edge 2, etc.
            uint32_t innerSiteId = ((outerSiteId - 7) / 2) + 1;
            if (innerSiteId >= numEdgeServers)
            {
                innerSiteId = numEdgeServers - 1;  // Safety clamp
            }

            Ipv4Address innerEdgeIp = edgeServerIps[innerSiteId];
            Ptr<Node> innerEdgeNode = edgeServerNodes.Get(innerSiteId);

            // Map this outer site's cell to the inner edge server (1 cell per site)
            uint32_t nodeIdx = outerSiteId;  // 1 cell per site
            if (nodeIdx < numCells)
            {
                Ptr<NrGnbNetDevice> gnbNetDevice = gnbNetDevs.Get(nodeIdx)->GetObject<NrGnbNetDevice>();
                uint16_t actualCellId = gnbNetDevice->GetCellId();
                g_cellIdToEdgeServer[actualCellId] = innerEdgeNode;
                g_cellIdToEdgeServerIp[actualCellId] = innerEdgeIp;
            }

            NS_LOG_UNCOND("Outer Site " << outerSiteId << " -> Edge " << innerSiteId
                          << " (IP: " << innerEdgeIp << ")");
        }
    }

    //--------------------------------------------------------------------------
    // Set up Remote Host (controller / traffic source)
    // Remote Host -> WAN -> Edge Server (remoteHostEdgeId) -> Zenoh -> all edges
    //--------------------------------------------------------------------------
    Ptr<Node> remoteHostNode = nullptr;
    Ptr<Node> remoteHostGhostNode = nullptr;
    Ptr<NetDevice> remoteHostGhostDevice = nullptr;
    Ptr<NetDevice> edgeWanDevice = nullptr;  // WAN P2P device on source edge (for ingress tracking)

    if (enableRemoteHost && enableTap)
    {
        // Validate remoteHostEdgeId (must be within numEdgeServers, not numSites)
        if (remoteHostEdgeId >= numEdgeServers)
        {
            NS_LOG_WARN("remoteHostEdgeId " << remoteHostEdgeId << " >= numEdgeServers " << numEdgeServers
                        << ", using edge 0 instead");
            remoteHostEdgeId = 0;
        }

        // Create Remote Host node
        remoteHostNode = CreateObject<Node>();
        internet.Install(remoteHostNode);

        // Create Ghost node for Remote Host (for tap bridge)
        remoteHostGhostNode = CreateObject<Node>();
        // Ghost node doesn't need IP stack - it's pure L2 bridge

        // WAN link: Remote Host <-> Edge Server (remoteHostEdgeId)
        // Using point-to-point link to simulate WAN with configurable delay
        PointToPointHelper wanP2p;
        wanP2p.SetDeviceAttribute("DataRate", StringValue(wanDataRate));
        wanP2p.SetChannelAttribute("Delay", TimeValue(MilliSeconds(wanDelayMs)));

        NodeContainer wanNodes;
        wanNodes.Add(remoteHostNode);
        wanNodes.Add(edgeServerNodes.Get(remoteHostEdgeId));

        NetDeviceContainer wanDevices = wanP2p.Install(wanNodes);
        edgeWanDevice = wanDevices.Get(1);  // Edge server's WAN P2P device (8.0.0.2)

        // Assign IP addresses for WAN link: 8.0.0.0/24
        // Using 8.x.x.x range to avoid conflicts with:
        // - Docker bridge networks (172.17.x.x, 192.168.x.x)
        // - UE network (7.0.x.x)
        // - Edge networks (10.x.x.x)
        Ipv4AddressHelper wanIpHelper;
        wanIpHelper.SetBase("8.0.0.0", "255.255.255.0");
        Ipv4InterfaceContainer wanIpIfaces = wanIpHelper.Assign(wanDevices);
        Ipv4Address remoteHostWanIp = wanIpIfaces.GetAddress(0);  // 8.0.0.1
        Ipv4Address edgeWanIp = wanIpIfaces.GetAddress(1);        // 8.0.0.2

        // CSMA link: Remote Host <-> Ghost Node (for tap bridge)
        CsmaHelper remoteHostCsma;
        remoteHostCsma.SetChannelAttribute("DataRate", DataRateValue(DataRate("10Gbps")));
        remoteHostCsma.SetChannelAttribute("Delay", TimeValue(MicroSeconds(0)));

        NodeContainer remoteHostGhostLink;
        remoteHostGhostLink.Add(remoteHostNode);
        remoteHostGhostLink.Add(remoteHostGhostNode);

        NetDeviceContainer remoteHostGhostDevices = remoteHostCsma.Install(remoteHostGhostLink);

        // Assign IP for Remote Host's CSMA interface (ghost segment): 8.0.1.0/24
        // Docker controller will use 8.0.1.x addresses
        Ipv4AddressHelper remoteHostCsmaIpHelper;
        remoteHostCsmaIpHelper.SetBase("8.0.1.0", "255.255.255.0");
        remoteHostCsmaIpHelper.Assign(remoteHostGhostDevices.Get(0));  // Remote Host only (8.0.1.1)

        remoteHostGhostDevice = remoteHostGhostDevices.Get(1);

        // Install promiscuous callback on Remote Host's CSMA device to capture
        // packets entering the simulation from external Zenoh publisher
        // This is the true entry point for end-to-end latency measurement
        remoteHostGhostDevices.Get(0)->SetPromiscReceiveCallback(
            MakeCallback(&RemoteHostPacketMonitor));

        // Set up routing on Remote Host
        // Default route via Edge Server
        Ptr<Ipv4StaticRouting> remoteHostRouting =
            ipv4RoutingHelper.GetStaticRouting(remoteHostNode->GetObject<Ipv4>());
        remoteHostRouting->SetDefaultRoute(edgeWanIp, 1);

        // Add route on target Edge Server for Remote Host's ghost segment
        Ptr<Ipv4StaticRouting> targetEdgeRouting =
            ipv4RoutingHelper.GetStaticRouting(edgeServerNodes.Get(remoteHostEdgeId)->GetObject<Ipv4>());
        // Route for remote host ghost segment (8.0.1.0/24) via Remote Host WAN IP
        targetEdgeRouting->AddNetworkRouteTo(
            Ipv4Address("8.0.1.0"),
            Ipv4Mask("255.255.255.0"),
            remoteHostWanIp,
            edgeServerNodes.Get(remoteHostEdgeId)->GetObject<Ipv4>()->GetInterfaceForAddress(edgeWanIp)
        );

        // Add route on PGW for Remote Host network (for return traffic)
        Ptr<Ipv4StaticRouting> pgwRouting =
            ipv4RoutingHelper.GetStaticRouting(pgw->GetObject<Ipv4>());
        // First find the edge server's PGW-facing IP
        Ipv4Address targetEdgePgwIp = Ipv4Address("10.1.0.2");  // Default
        for (uint32_t i = 0; i < edgeServerNodes.Get(remoteHostEdgeId)->GetObject<Ipv4>()->GetNInterfaces(); ++i)
        {
            Ipv4Address addr = edgeServerNodes.Get(remoteHostEdgeId)->GetObject<Ipv4>()->GetAddress(i, 0).GetLocal();
            std::ostringstream expectedBase;
            expectedBase << "10." << (remoteHostEdgeId + 1) << ".0.";
            if (addr == Ipv4Address((expectedBase.str() + "2").c_str()))
            {
                targetEdgePgwIp = addr;
                break;
            }
        }
        // Route 8.0.x.0/16 via target edge server
        pgwRouting->AddNetworkRouteTo(
            Ipv4Address("8.0.0.0"),
            Ipv4Mask("255.255.0.0"),
            targetEdgePgwIp,
            1  // PGW interface to edge network
        );

        NS_LOG_UNCOND("\n==============================================");
        NS_LOG_UNCOND("Remote Host Setup:");
        NS_LOG_UNCOND("  WAN Link: Remote Host <-> Edge " << remoteHostEdgeId);
        NS_LOG_UNCOND("  WAN Delay: " << wanDelayMs << " ms (one-way)");
        NS_LOG_UNCOND("  WAN Data Rate: " << wanDataRate);
        NS_LOG_UNCOND("  Remote Host WAN IP: " << remoteHostWanIp);
        NS_LOG_UNCOND("  Edge WAN IP: " << edgeWanIp);
        NS_LOG_UNCOND("  Remote Host Ghost Subnet: 8.0.1.0/24 (Docker controller uses 8.0.1.x)");
        NS_LOG_UNCOND("==============================================\n");
    }

    //--------------------------------------------------------------------------
    // Install tap bridges (if enabled)
    //--------------------------------------------------------------------------
    // Store UE's inner CSMA device for tunnel app (needs to be accessible later)
    Ptr<NetDevice> ueInnerCsmaDevice = nullptr;
    Ipv4Address ueIpAddr = ueIpIfaces.GetAddress(0);  // Store first UE's NR IP for tunnel

    if (enableTap)
    {
        TapBridgeHelper tapBridge;
        tapBridge.SetAttribute("Mode", StringValue("UseBridge"));

        // Install tap bridges on edge server ghost nodes
        for (uint32_t siteId = 0; siteId < numEdgeServers; ++siteId)
        {
            std::string tapDeviceName = tapEdgePrefix + std::to_string(siteId);
            tapBridge.SetAttribute("DeviceName", StringValue(tapDeviceName));
            tapBridge.Install(ghostNodes.Get(siteId), ghostDevices[siteId]);
            NS_LOG_UNCOND("Tap bridge installed on Ghost Node " << siteId << " (" << tapDeviceName << ")");
        }

        //----------------------------------------------------------------------
        // Install tap bridge for Remote Host (if enabled)
        //----------------------------------------------------------------------
        if (enableRemoteHost && remoteHostGhostNode && remoteHostGhostDevice)
        {
            tapBridge.SetAttribute("DeviceName", StringValue(tapRemoteHostDevice));
            tapBridge.Install(remoteHostGhostNode, remoteHostGhostDevice);
            NS_LOG_UNCOND("Tap bridge installed on Remote Host Ghost Node (" << tapRemoteHostDevice << ")");
        }

        //----------------------------------------------------------------------
        // Set up tap bridge for UE (first UE only, similar to nr-mec-handover.cc)
        //----------------------------------------------------------------------
        // Set up Ghost node for UE
        NodeContainer ueTapLinkNodes;
        ueTapLinkNodes.Add(ueNodes.Get(0));
        Ptr<Node> ueGhostNode = CreateObject<Node>();
        ueTapLinkNodes.Add(ueGhostNode);

        // Ghost node doesn't need Internet stack - it's a pure L2 bridge

        CsmaHelper ueTapCsma;
        ueTapCsma.SetChannelAttribute("DataRate", StringValue("10Gbps"));
        ueTapCsma.SetChannelAttribute("Delay", StringValue("0"));

        NetDeviceContainer ueTapDevices = ueTapCsma.Install(ueTapLinkNodes);

        // Store UE's CSMA device for tunnel app
        ueInnerCsmaDevice = ueTapDevices.Get(0);

        // Only assign IP to UE's CSMA interface, not ghost node
        // Ghost node acts as L2 bridge - external Linux host will have its own IP
        Ipv4AddressHelper ipv4Tap;
        ipv4Tap.SetBase("7.0.1.0", "255.255.255.0");
        Ipv4InterfaceContainer ueTapIpIfaces = ipv4Tap.Assign(ueTapDevices.Get(0));  // UE only
        Ipv4Address ueLanIp = ueTapIpIfaces.GetAddress(0);

        Ptr<Ipv4> ueIpv4 = ueNodes.Get(0)->GetObject<Ipv4>();
        ueIpv4->SetAttribute("IpForward", BooleanValue(true));

        // Disable forwarding specifically on the CSMA interface
        // This prevents normal IP forwarding of packets from client subnet
        // The tunnel app will handle forwarding these packets via UDP encapsulation
        int32_t csmaIfIndex = ueIpv4->GetInterfaceForDevice(ueTapDevices.Get(0));
        if (csmaIfIndex >= 0)
        {
            ueIpv4->SetForwarding(csmaIfIndex, false);
            NS_LOG_UNCOND("Disabled IP forwarding on UE CSMA interface " << csmaIfIndex);
        }

        // Install tap bridge on UE ghost node
        tapBridge.SetAttribute("DeviceName", StringValue(tapUeDevice));
        tapBridge.Install(ueTapLinkNodes.Get(1), ueTapDevices.Get(1));

        NS_LOG_UNCOND("Tap bridge installed on UE (UseBridge mode)");
        NS_LOG_UNCOND("  UE WAN IP (5G): " << ueIpAddr);
        NS_LOG_UNCOND("  UE LAN IP (Tap): " << ueLanIp);
        NS_LOG_UNCOND("  Tap device: " << tapUeDevice);

        // Debug: Print UE routing table
        // NS_LOG_UNCOND("\nUE Routing Table:");
        // NS_LOG_UNCOND("UE has " << ueIpv4->GetNInterfaces() << " interfaces:");
        // for (uint32_t i = 0; i < ueIpv4->GetNInterfaces(); ++i)
        // {
        //     NS_LOG_UNCOND("  Interface " << i << ": " << ueIpv4->GetAddress(i, 0).GetLocal());
        // }
        // Ptr<Ipv4StaticRouting> ueRouting =
        //     ipv4RoutingHelper.GetStaticRouting(ueIpv4);
        // ueRouting->PrintRoutingTable(Create<OutputStreamWrapper>(&std::cout));
    }

    //--------------------------------------------------------------------------
    // Set up UDP Tunnel Applications (if tap enabled)
    // These tunnel client packets through the 5G network using IP-in-UDP encapsulation
    // to bypass GTP filtering that drops packets with non-UE source IPs
    //--------------------------------------------------------------------------
    if (enableTap && ueInnerCsmaDevice)
    {
        // Get first UE's IMSI for tunnel app registration
        Ptr<NrUeNetDevice> firstUeDev = ueNetDevs.Get(0)->GetObject<NrUeNetDevice>();
        uint64_t firstUeImsi = firstUeDev->GetImsi();

        // Install UeTunnelApp on first UE node
        // Captures packets from client (7.0.1.x) on CSMA interface
        // Encapsulates and sends to Edge Server via 5G NR interface
        Ptr<UeTunnelApp> ueTunnelApp = CreateObject<UeTunnelApp>();
        ueTunnelApp->SetInnerDevice(ueInnerCsmaDevice);
        ueTunnelApp->SetClientSubnet(Ipv4Address("7.0.1.0"), Ipv4Mask("255.255.255.0"));

        // Set initial tunnel endpoint to Edge Server 0 (10.1.0.2)
        // This will dynamically switch based on handover via HandoverEndOkCallback
        Ipv4Address edgeServer0Ip = Ipv4Address("10.1.0.2");
        ueTunnelApp->SetTunnelEndpoint(edgeServer0Ip, 5000);
        ueTunnelApp->SetLocalPort(5000);

        ueNodes.Get(0)->AddApplication(ueTunnelApp);
        ueTunnelApp->SetStartTime(Seconds(1.0));
        ueTunnelApp->SetStopTime(Seconds(simTime));

        // Store global reference for handover-triggered endpoint switching
        g_ueTunnelApps[firstUeImsi] = ueTunnelApp;

        NS_LOG_UNCOND("\nUE Tunnel App installed on UE 0 (IMSI=" << firstUeImsi << "):");
        NS_LOG_UNCOND("  Inner device: CSMA (7.0.1.1)");
        NS_LOG_UNCOND("  Client subnet: 7.0.1.0/24");
        NS_LOG_UNCOND("  Initial tunnel endpoint: " << edgeServer0Ip << ":5000");
        NS_LOG_UNCOND("  (Endpoint will switch dynamically on handover)");

        // Install EdgeTunnelApp on each edge server
        // Ghost Node Architecture:
        //   PGW <--P2P--> EdgeServer <--CSMA--> GhostNode <--TapBridge--> Docker
        //                     |
        //                EdgeTunnelApp
        //   - OuterDevice: EdgeServer's P2P device facing PGW
        //   - InnerDevice: EdgeServer's CSMA device facing GhostNode
        for (uint32_t siteId = 0; siteId < numEdgeServers; ++siteId)
        {
            Ptr<EdgeTunnelApp> edgeTunnelApp = CreateObject<EdgeTunnelApp>();

            // Set both devices (both belong to EdgeServer node):
            // - Outer: EdgeServer's P2P device facing PGW (tunnel packets arrive here)
            // - Inner: EdgeServer's CSMA device facing GhostNode (forwards to Docker)
            edgeTunnelApp->SetOuterDevice(edgeServerOuterDevices[siteId]);
            edgeTunnelApp->SetInnerDevice(edgeServerInnerDevices[siteId]);

            // Set inner subnet (10.x.1.0/24) - packets to other subnets will be dropped
            // This ensures connection breaks after handover instead of routing via PGW
            std::ostringstream innerSubnetStr;
            innerSubnetStr << "10." << (siteId + 1) << ".1.0";
            edgeTunnelApp->SetInnerSubnet(Ipv4Address(innerSubnetStr.str().c_str()),
                                          Ipv4Mask("255.255.255.0"));

            // Add tunnel mapping: packets to 7.0.1.x should go to UE NR IP
            edgeTunnelApp->AddTunnelMapping(
                Ipv4Address("7.0.1.0"),
                Ipv4Mask("255.255.255.0"),
                ueIpAddr  // UE's NR interface IP (7.0.0.x)
            );
            edgeTunnelApp->SetLocalPort(5000);
            edgeTunnelApp->SetTunnelPort(5000);
            edgeTunnelApp->SetEdgeNodeId(siteId);

            // For the source edge (connected to Remote Host), set WAN device for ingress tracking
            if (siteId == remoteHostEdgeId && edgeWanDevice)
            {
                edgeTunnelApp->SetWanDevice(edgeWanDevice);
            }

            edgeServerNodes.Get(siteId)->AddApplication(edgeTunnelApp);
            edgeTunnelApp->SetStartTime(Seconds(1.0));
            edgeTunnelApp->SetStopTime(Seconds(simTime));

            NS_LOG_UNCOND("Edge Tunnel App " << siteId << " installed:");
            NS_LOG_UNCOND("  Outer device: " << edgeServerOuterDevices[siteId]->GetAddress());
            NS_LOG_UNCOND("  Inner device: " << edgeServerInnerDevices[siteId]->GetAddress());
            NS_LOG_UNCOND("  Inner subnet: " << innerSubnetStr.str() << "/24");
            NS_LOG_UNCOND("  Mapping: 7.0.1.0/24 -> " << ueIpAddr);
        }
    }

    //--------------------------------------------------------------------------
    // Attach UEs to closest gNB
    //--------------------------------------------------------------------------
    nrHelper->AttachToClosestGnb(ueNetDevs, gnbNetDevs);

    //--------------------------------------------------------------------------
    // Install background DL traffic (PGW -> Background UEs)
    //--------------------------------------------------------------------------
    if (numBackgroundUes > 0)
    {
        Ptr<Node> pgwNode = epcHelper->GetPgwNode();
        uint16_t bgDlPort = 3000;
        double bgDataRateBps = bgUeTrafficMbps * 1e6;
        uint32_t bgPacketSizeBits = bgUePacketSize * 8;
        double bgIntervalSec = static_cast<double>(bgPacketSizeBits) / bgDataRateBps;

        for (uint32_t i = 0; i < numBackgroundUes; i++)
        {
            // Sink on background UE
            PacketSinkHelper bgSinkHelper("ns3::UdpSocketFactory",
                InetSocketAddress(Ipv4Address::GetAny(), bgDlPort + i));
            ApplicationContainer bgSinkApp = bgSinkHelper.Install(backgroundUeNodes.Get(i));
            bgSinkApp.Start(Seconds(0.1));

            // Source on PGW
            OnOffHelper bgOnOffHelper("ns3::UdpSocketFactory",
                InetSocketAddress(bgUeIpIfaces.GetAddress(i), bgDlPort + i));
            bgOnOffHelper.SetAttribute("DataRate", DataRateValue(DataRate(static_cast<uint64_t>(bgDataRateBps))));
            bgOnOffHelper.SetAttribute("PacketSize", UintegerValue(bgUePacketSize));
            bgOnOffHelper.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
            bgOnOffHelper.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));

            ApplicationContainer bgSourceApp = bgOnOffHelper.Install(pgwNode);
            // Stagger start times to avoid burst, with configurable delay
            bgSourceApp.Start(Seconds(bgTrafficStartDelay + i * 0.1));
        }

        double totalBgTraffic = numBackgroundUes * bgUeTrafficMbps;
        double lastStartTime = bgTrafficStartDelay + (numBackgroundUes - 1) * 0.1;
        NS_LOG_UNCOND("\nBackground DL traffic installed:");
        NS_LOG_UNCOND("  " << numBackgroundUes << " UEs x " << bgUeTrafficMbps << " Mbps = " << totalBgTraffic << " Mbps total");
        NS_LOG_UNCOND("  Packet size: " << bgUePacketSize << " bytes, interval: " << (bgIntervalSec * 1000) << " ms");
        NS_LOG_UNCOND("  Start delay: " << bgTrafficStartDelay << "s, staggered 100ms apart (last starts at " << lastStartTime << "s)");
    }

    // Initialize serving cell tracking and update tunnel endpoint based on actual attachment
    for (uint32_t i = 0; i < ueNetDevs.GetN(); ++i)
    {
        Ptr<NrUeNetDevice> ueDev = ueNetDevs.Get(i)->GetObject<NrUeNetDevice>();
        uint64_t imsi = ueDev->GetImsi();
        uint16_t cellId = ueDev->GetCellId();
        g_ueCurrentServingCell[imsi] = cellId;

        // Set main UE IMSI (first UE is the main one)
        if (i == 0)
        {
            g_mainUeImsi = imsi;
        }

        NS_LOG_UNCOND("UE " << i << " (IMSI=" << imsi << ") attached to cell " << cellId);

        // Update tunnel endpoint for first UE based on actual serving cell
        if (i == 0 && enableTap && g_ueTunnelApps.count(imsi) > 0)
        {
            if (g_cellIdToEdgeServerIp.count(cellId) > 0)
            {
                Ipv4Address actualEdgeIp = g_cellIdToEdgeServerIp[cellId];
                g_ueTunnelApps[imsi]->SetTunnelEndpoint(actualEdgeIp, 5000);
                NS_LOG_UNCOND("  Updated tunnel endpoint to " << actualEdgeIp << " (matching serving cell)");
            }
            else
            {
                NS_LOG_WARN("  WARNING: No edge server mapping for cell " << cellId);
            }
        }
    }

    //--------------------------------------------------------------------------
    // Add X2 interfaces between all gNBs for handover
    //--------------------------------------------------------------------------
    nrHelper->AddX2Interface(gnbNodes);

    //--------------------------------------------------------------------------
    // Configure measurement reporting
    //--------------------------------------------------------------------------

    // A3 event for early detection (large negative offset)
    NrRrcSap::ReportConfigEutra reportConfigA3Early;
    reportConfigA3Early.triggerType = NrRrcSap::ReportConfigEutra::EVENT;
    reportConfigA3Early.eventId = NrRrcSap::ReportConfigEutra::EVENT_A3;
    reportConfigA3Early.a3Offset = -30;  // -15 dB for early detection
    reportConfigA3Early.hysteresis = 0;
    reportConfigA3Early.timeToTrigger = 0;
    reportConfigA3Early.reportOnLeave = false;
    reportConfigA3Early.triggerQuantity = NrRrcSap::ReportConfigEutra::RSRP;
    reportConfigA3Early.reportQuantity = NrRrcSap::ReportConfigEutra::BOTH;
    reportConfigA3Early.maxReportCells = 8;
    reportConfigA3Early.reportInterval = NrRrcSap::ReportConfigEutra::MS480;

    // // Standard A3 event for handover
    // NrRrcSap::ReportConfigEutra reportConfigA3;
    // reportConfigA3.triggerType = NrRrcSap::ReportConfigEutra::EVENT;
    // reportConfigA3.eventId = NrRrcSap::ReportConfigEutra::EVENT_A3;
    // reportConfigA3.a3Offset = 0;
    // reportConfigA3.hysteresis = 6;  // 3 dB
    // reportConfigA3.timeToTrigger = 256;
    // reportConfigA3.reportOnLeave = false;
    // reportConfigA3.triggerQuantity = NrRrcSap::ReportConfigEutra::RSRP;
    // reportConfigA3.reportQuantity = NrRrcSap::ReportConfigEutra::BOTH;
    // reportConfigA3.maxReportCells = 8;
    // reportConfigA3.reportInterval = NrRrcSap::ReportConfigEutra::MS480;

    // Add measurement configs to all gNBs
    for (uint32_t i = 0; i < gnbNodes.GetN(); ++i)
    {
        Ptr<NrGnbRrc> gnbRrc = gnbNodes.Get(i)->GetDevice(0)->GetObject<NrGnbNetDevice>()->GetRrc();
        gnbRrc->AddUeMeasReportConfig(reportConfigA3Early);
        // gnbRrc->AddUeMeasReportConfig(reportConfigA3);
    }

    //--------------------------------------------------------------------------
    // Connect trace sources
    //--------------------------------------------------------------------------

    // Measurement report callback to all gNBs
    for (uint32_t i = 0; i < gnbNodes.GetN(); ++i)
    {
        std::ostringstream path;
        path << "/NodeList/" << gnbNodes.Get(i)->GetId()
             << "/DeviceList/0/NrGnbRrc/RecvMeasurementReport";
        Config::Connect(path.str(), MakeCallback(&MeasurementReportCallback));
    }

    // Handover callbacks to UEs
    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        std::ostringstream uePath;
        uePath << "/NodeList/" << ueNodes.Get(i)->GetId() << "/DeviceList/0/NrUeRrc/";

        Config::Connect(uePath.str() + "HandoverStart", MakeCallback(&HandoverStartCallback));
        Config::Connect(uePath.str() + "HandoverEndOk", MakeCallback(&HandoverEndOkCallback));
        Config::Connect(uePath.str() + "HandoverEndError", MakeCallback(&HandoverEndErrorCallback));
    }

    //--------------------------------------------------------------------------
    // Schedule UE position printing
    //--------------------------------------------------------------------------
    Simulator::Schedule(Seconds(10.0), &PrintUePosition, ueNodes);

    //--------------------------------------------------------------------------
    // Initialize trajectory logging (if enabled)
    //--------------------------------------------------------------------------
    if (!trajectoryOutputPath.empty())
    {
        g_trajectoryLoggingEnabled = true;
        g_trajectoryBuffer.clear();

        // Schedule first logging (data buffered in memory, written at simulation end)
        Simulator::Schedule(Seconds(trajectoryLogInterval), &LogUeTrajectory, ueNodes, trajectoryLogInterval);

        NS_LOG_UNCOND("Trajectory logging enabled: " << trajectoryOutputPath
                      << " (interval=" << trajectoryLogInterval << "s, buffered)");
    }

    //--------------------------------------------------------------------------
    // Print summary
    //--------------------------------------------------------------------------
    NS_LOG_UNCOND("\n==============================================");
    NS_LOG_UNCOND("Network Summary:");
    NS_LOG_UNCOND("  Sites: " << numSites);
    NS_LOG_UNCOND("  Cells: " << numCells);
    if (extendedCoverage && numEdgeServers < numSites)
    {
        NS_LOG_UNCOND("  Edge Servers: " << numEdgeServers << " (extended coverage: outer ring shares inner edges)");
    }
    else
    {
        NS_LOG_UNCOND("  Edge Servers: " << numEdgeServers << " (one per site)");
    }
    NS_LOG_UNCOND("  UEs: " << ueNodes.GetN());
    if (numBackgroundUes > 0)
    {
        double totalBgTraffic = numBackgroundUes * bgUeTrafficMbps;
        double totalNetworkCap = numCells * capacityMbps;
        double perCellLoad = totalBgTraffic / numCells;
        NS_LOG_UNCOND("  Background UEs: " << numBackgroundUes);
        NS_LOG_UNCOND("  Total bg traffic: " << totalBgTraffic << " Mbps ("
                      << (totalBgTraffic / totalNetworkCap * 100) << "% of network)");
        NS_LOG_UNCOND("  Per-cell load: " << perCellLoad << " Mbps ("
                      << (perCellLoad / capacityMbps * 100) << "% of " << capacityMbps << " Mbps)");
    }
    NS_LOG_UNCOND("==============================================\n");

    //--------------------------------------------------------------------------
    // Run simulation
    //--------------------------------------------------------------------------
    NS_LOG_INFO("Starting simulation...");
    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    //--------------------------------------------------------------------------
    // Print final statistics
    //--------------------------------------------------------------------------
    NS_LOG_UNCOND("\n==============================================");
    NS_LOG_UNCOND("Simulation Complete");
    NS_LOG_UNCOND("==============================================");

    // Print Zenoh latency measurement summary
    ZenohLatencyTracker::GetInstance().PrintSummary();
    ZenohLatencyTracker::GetInstance().Close();

    // Write buffered trajectory data to file
    if (g_trajectoryLoggingEnabled && !g_trajectoryBuffer.empty())
    {
        std::ofstream trajFile(trajectoryOutputPath);
        if (trajFile.is_open())
        {
            trajFile << "time_s,imsi,x,y,z,vx,vy,vz,speed,cell_id,site_id\n";
            for (const auto& row : g_trajectoryBuffer)
            {
                trajFile << row << "\n";
            }
            trajFile.close();
            NS_LOG_UNCOND("Trajectory saved to: " << trajectoryOutputPath
                          << " (" << g_trajectoryBuffer.size() << " records)");
        }
        else
        {
            NS_LOG_WARN("Failed to open trajectory file: " << trajectoryOutputPath);
        }
    }

    // Write simulation results summary
    if (!resultsOutputPath.empty())
    {
        WriteSimulationResults(resultsOutputPath, simTime, slaThresholdMs);
    }

    // Print experiment directory summary
    NS_LOG_UNCOND("\n==============================================");
    NS_LOG_UNCOND("Experiment Output Directory: " << experimentDir);
    NS_LOG_UNCOND("  config.txt  - experiment configuration");
    NS_LOG_UNCOND("  latency.csv - packet latency data");
    NS_LOG_UNCOND("  traj.csv    - UE trajectory data");
    NS_LOG_UNCOND("  result.csv  - simulation results summary");
    NS_LOG_UNCOND("==============================================\n");

    Simulator::Destroy();
    return 0;
}
