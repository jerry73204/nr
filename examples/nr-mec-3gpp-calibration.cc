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
 * - Hexagonal grid deployment with configurable rings (1 ring = 7 sites, 21 cells)
 * - Triple-sectorized sites (3 sectors per site)
 * - One edge server per site (shared by all 3 sectors)
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
 * Each site has 3 sectors, and each site has one edge server.
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

#include "ue-tunnel-app.h"
#include "edge-tunnel-app.h"
#include "handover-prediction-file.h"
#include "zenoh-latency-measurement.h"

#include <cmath>
#include <deque>
#include <fstream>
#include <iomanip>
#include <map>

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
double g_handoverInterruptMs = 50.0;  // Total handover interruption time (ms)
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

// Pointer to the scenario helper (for index mapping)
NodeDistributionScenarioInterface* g_scenario = nullptr;

// Trajectory logging
std::ofstream g_trajectoryFile;
bool g_trajectoryLoggingEnabled = false;

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
 * @return Predicted target cell ID (0 if no prediction)
 */
uint16_t
PredictHandoverTarget(uint16_t currentCellId)
{
    auto servingIt = g_measurementHistory.find(currentCellId);
    if (servingIt == g_measurementHistory.end() || servingIt->second.empty())
    {
        return 0;
    }

    int servingRsrp = servingIt->second.back().rsrp;
    const double hysteresis = 3.0;

    uint16_t predictedTarget = 0;
    double bestScore = -999.0;

    for (const auto& [cellId, history] : g_measurementHistory)
    {
        if (cellId == currentCellId || history.size() < 2)
        {
            continue;
        }

        int neighborRsrp = history.back().rsrp;
        double trend = CalculateRsrpTrend(history);
        double margin = neighborRsrp - servingRsrp - hysteresis;

        if (margin > -10.0 && (trend > 0 || margin > 0))
        {
            double score = margin + trend * 2.0;
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

    // Process neighbor cell measurements
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
        }
    }

    // Run handover prediction
    uint16_t predictedTarget = PredictHandoverTarget(cellId);

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

    NS_LOG_UNCOND(now.GetSeconds()
                  << "s [HANDOVER START] IMSI=" << imsi
                  << " Cell " << sourceCellId << " (Site " << sourceSite << ")"
                  << " -> Cell " << targetCellId << " (Site " << targetSite << ")"
                  << " (interrupt: " << g_handoverInterruptMs << "ms)");

    // Mark handover as active and set handover window for latency measurement
    if (g_ueTunnelApps.count(imsi) > 0 && g_ueTunnelApps[imsi])
    {
        g_ueTunnelApps[imsi]->SetHandoverActive(true);
        g_ueTunnelApps[imsi]->SetHandoverWindow(now, g_handoverEndTime[imsi], g_networkDelayMs);
    }

    if (sourceSite != targetSite)
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
        NS_LOG_UNCOND("  Intra-site handover: no edge server switch");
    }
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

    g_ueCurrentServingCell[imsi] = cellId;

    NS_LOG_UNCOND(Simulator::Now().GetSeconds()
                  << "s [HANDOVER SUCCESS] IMSI=" << imsi
                  << " New ServingCell=" << cellId << " (Site " << newSite << ")");

    // Clear handover active flag for latency measurement
    if (g_ueTunnelApps.count(imsi) > 0 && g_ueTunnelApps[imsi])
    {
        g_ueTunnelApps[imsi]->SetHandoverActive(false);
    }

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
}

/**
 * @brief Callback when handover fails
 */
void
HandoverEndErrorCallback(std::string path, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
    NS_LOG_UNCOND(Simulator::Now().GetSeconds()
                  << "s [HANDOVER FAILED] IMSI=" << imsi << " CellId=" << cellId);
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

        g_trajectoryFile << std::fixed << std::setprecision(3)
                         << Simulator::Now().GetSeconds() << ","
                         << imsi << ","
                         << pos.x << "," << pos.y << "," << pos.z << ","
                         << vel.x << "," << vel.y << "," << vel.z << ","
                         << speed << ","
                         << servingCell << "," << siteId << "\n";
    }

    // Flush to ensure data is written even if simulation crashes
    g_trajectoryFile.flush();

    // Schedule next logging
    Simulator::Schedule(Seconds(interval), &LogUeTrajectory, ueNodes, interval);
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
    std::string builtinPath = "linear-y";        // Built-in paths: hexagonal, linear-y, zigzag
    double simTime = 300.0;                      // Simulation time in seconds

    // NR parameters
    double centralFrequency = 3.5e9;             // 3.5 GHz (n78 band)
    double bandwidth = 20e6;                     // 20 MHz
    double gnbTxPower = 43.0;                    // gNB TX power in dBm
    uint16_t numerology = 1;                     // NR numerology

    // Tap bridge parameters
    bool enableTap = false;
    std::string tapUeDevice = "tap_ue";
    std::string tapEdgePrefix = "tap_edge";

    // Handover prediction
    std::string predictionFilePath = "/tmp/ns3_handover/ns3_handover";
    double predictionWriteIntervalMs = 100.0;

    // Latency measurement
    std::string latencyOutputPath = "/tmp/zenoh_latency.csv";
    double slaThresholdMs = 50.0;
    uint16_t sourceEdgeNodeId = 5;  // Edge node that publishes critical data

    // Trajectory logging
    std::string trajectoryOutputPath = "";       // Empty = disabled, path = enable CSV output
    double trajectoryLogInterval = 0.5;          // Logging interval in seconds

    // Remote Host (traffic source / controller)
    bool enableRemoteHost = true;
    std::string tapRemoteHostDevice = "tap_remote";
    double wanDelayMs = 20.0;        // WAN delay in milliseconds (one-way)
    std::string wanDataRate = "1Gbps";  // WAN link data rate
    uint16_t remoteHostEdgeId = 5;   // Edge server to connect remote host to

    // Handover blackout model
    double handoverInterruptMs = 30.0;  // Handover interruption time (realistic: 30-100ms)

    // Other
    bool logging = true;

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
    cmd.AddValue("builtinPath", "Built-in path: hexagonal, linear-y, zigzag, highway, urban-grid", builtinPath);
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

    // Latency measurement
    cmd.AddValue("latencyOutputPath", "Output path for latency CSV file", latencyOutputPath);
    cmd.AddValue("slaThresholdMs", "SLA threshold in milliseconds", slaThresholdMs);
    cmd.AddValue("sourceEdgeNodeId", "Edge node ID that publishes critical data", sourceEdgeNodeId);

    // Trajectory logging
    cmd.AddValue("trajectoryOutputPath", "Path for trajectory CSV file (empty=disabled)", trajectoryOutputPath);
    cmd.AddValue("trajectoryLogInterval", "Trajectory logging interval in seconds", trajectoryLogInterval);

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

    cmd.Parse(argc, argv);

    // Update global handover interrupt parameters
    g_handoverInterruptMs = handoverInterruptMs;
    g_networkDelayMs = wanDelayMs;  // Used to calculate packet arrival at gNB

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
    scenarioParams.SetSectorization(3);  // Triple-sectorized

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
        // Create static scenario first, then replace mobility on first UE
        gridScenario.CreateScenario();

        NodeContainer ueNodesTemp = gridScenario.GetUserTerminals();

        // Create Gauss-Markov mobility model directly and set position
        Ptr<GaussMarkovMobilityModel> gaussMobility = CreateObject<GaussMarkovMobilityModel>();
        gaussMobility->SetAttribute("Bounds", BoxValue(Box(-gridRadius, gridRadius, -gridRadius, gridRadius, 0, 10)));
        gaussMobility->SetAttribute("TimeStep", TimeValue(Seconds(gaussTimeStep)));
        gaussMobility->SetAttribute("Alpha", DoubleValue(gaussAlpha));
        gaussMobility->SetAttribute("MeanVelocity", StringValue(BuildNormalVelocityString(ueSpeed, ueSpeedVariance)));
        gaussMobility->SetAttribute("MeanDirection", StringValue("ns3::UniformRandomVariable[Min=0|Max=6.283185307]"));
        gaussMobility->SetAttribute("MeanPitch", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
        gaussMobility->SetAttribute("NormalVelocity", StringValue("ns3::NormalRandomVariable[Mean=0.0|Variance=1.0|Bound=10.0]"));
        gaussMobility->SetAttribute("NormalDirection", StringValue("ns3::NormalRandomVariable[Mean=0.0|Variance=0.2|Bound=0.4]"));
        gaussMobility->SetAttribute("NormalPitch", StringValue("ns3::NormalRandomVariable[Mean=0.0|Variance=0.0|Bound=0.0]"));

        // Set initial position
        gaussMobility->SetPosition(Vector(0.0, 50.0, 1.5));

        // Replace mobility model on first UE
        Ptr<Node> firstUe = ueNodesTemp.Get(0);
        Ptr<MobilityModel> oldMobility = firstUe->GetObject<MobilityModel>();
        if (oldMobility)
        {
            firstUe->AggregateObject(gaussMobility);
            // Note: We can't remove old mobility, but the new one will be used
            // when explicitly fetched by type
        }
        else
        {
            firstUe->AggregateObject(gaussMobility);
        }

        NS_LOG_INFO("Gauss-Markov mobility: alpha=" << gaussAlpha
                    << " meanSpeed=" << ueSpeed << " m/s"
                    << " variance=" << ueSpeedVariance);
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

            if (builtinPath == "hexagonal")
            {
                // Circular path visiting all sites (triggers multiple handovers)
                waypointMm->AddWaypoint(Waypoint(Seconds(0), Vector(0, 50, 1.5)));
                waypointMm->AddWaypoint(Waypoint(Seconds(30), Vector(200, 200, 1.5)));
                waypointMm->AddWaypoint(Waypoint(Seconds(60), Vector(0, 400, 1.5)));
                waypointMm->AddWaypoint(Waypoint(Seconds(90), Vector(-200, 200, 1.5)));
                waypointMm->AddWaypoint(Waypoint(Seconds(120), Vector(-200, -200, 1.5)));
                waypointMm->AddWaypoint(Waypoint(Seconds(150), Vector(0, -400, 1.5)));
                waypointMm->AddWaypoint(Waypoint(Seconds(180), Vector(200, -200, 1.5)));
                waypointMm->AddWaypoint(Waypoint(Seconds(210), Vector(0, 50, 1.5)));
                NS_LOG_INFO("Waypoint mobility: hexagonal path visiting all sites");
            }
            else if (builtinPath == "linear-y")
            {
                double totalDist = 450.0;
                double travelTime = totalDist / ueSpeed;
                waypointMm->AddWaypoint(Waypoint(Seconds(0), Vector(0, 50, 1.5)));
                waypointMm->AddWaypoint(Waypoint(Seconds(travelTime), Vector(0, 500, 1.5)));
                NS_LOG_INFO("Waypoint mobility: linear-y path toward Site 2");
            }
            else if (builtinPath == "zigzag")
            {
                waypointMm->AddWaypoint(Waypoint(Seconds(0), Vector(0, 50, 1.5)));
                waypointMm->AddWaypoint(Waypoint(Seconds(20), Vector(150, 150, 1.5)));
                waypointMm->AddWaypoint(Waypoint(Seconds(40), Vector(-150, 250, 1.5)));
                waypointMm->AddWaypoint(Waypoint(Seconds(60), Vector(150, 350, 1.5)));
                waypointMm->AddWaypoint(Waypoint(Seconds(80), Vector(-150, 450, 1.5)));
                NS_LOG_INFO("Waypoint mobility: zigzag path crossing sectors");
            }
            else if (builtinPath == "highway")
            {
                double highwaySpeed = 30.0;
                double distance = std::sqrt(2) * 800.0;
                double travelTime = distance / highwaySpeed;

                waypointMm->AddWaypoint(Waypoint(Seconds(0), Vector(-400, -400, 1.5)));
                waypointMm->AddWaypoint(Waypoint(Seconds(travelTime), Vector(400, 400, 1.5)));
                NS_LOG_INFO("Waypoint mobility: highway path (speed=" << highwaySpeed
                            << " m/s, travel time=" << travelTime << "s)");
            }
            else if (builtinPath == "urban-grid")
            {
                double driveSpeed = 10.0;
                double blockSize = 200.0;
                double blockTime = blockSize / driveSpeed;
                double stopTime = 5.0;

                double t = 0.0;
                waypointMm->AddWaypoint(Waypoint(Seconds(t), Vector(-200, -200, 1.5)));

                t += blockTime;
                waypointMm->AddWaypoint(Waypoint(Seconds(t), Vector(0, -200, 1.5)));
                t += stopTime;
                waypointMm->AddWaypoint(Waypoint(Seconds(t), Vector(0, -200, 1.5)));

                t += blockTime;
                waypointMm->AddWaypoint(Waypoint(Seconds(t), Vector(0, 0, 1.5)));
                t += stopTime;
                waypointMm->AddWaypoint(Waypoint(Seconds(t), Vector(0, 0, 1.5)));

                t += blockTime;
                waypointMm->AddWaypoint(Waypoint(Seconds(t), Vector(200, 0, 1.5)));
                t += stopTime;
                waypointMm->AddWaypoint(Waypoint(Seconds(t), Vector(200, 0, 1.5)));

                t += blockTime;
                waypointMm->AddWaypoint(Waypoint(Seconds(t), Vector(200, 200, 1.5)));
                t += stopTime;
                waypointMm->AddWaypoint(Waypoint(Seconds(t), Vector(200, 200, 1.5)));

                t += blockTime;
                waypointMm->AddWaypoint(Waypoint(Seconds(t), Vector(0, 200, 1.5)));
                t += stopTime;
                waypointMm->AddWaypoint(Waypoint(Seconds(t), Vector(0, 200, 1.5)));

                t += blockTime;
                waypointMm->AddWaypoint(Waypoint(Seconds(t), Vector(0, 400, 1.5)));

                NS_LOG_INFO("Waypoint mobility: urban-grid path with " << stopTime
                            << "s stops at intersections (total time=" << t << "s)");
            }
            else
            {
                NS_LOG_WARN("Unknown builtinPath: " << builtinPath << ", using linear-y");
                double totalDist = 450.0;
                double travelTime = totalDist / ueSpeed;
                waypointMm->AddWaypoint(Waypoint(Seconds(0), Vector(0, 50, 1.5)));
                waypointMm->AddWaypoint(Waypoint(Seconds(travelTime), Vector(0, 500, 1.5)));
            }
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
            firstUeMobility->SetPosition(Vector(0.0, 50.0, 1.5));
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
    nrHelper->SetHandoverAlgorithmAttribute("Hysteresis", DoubleValue(3.0));
    nrHelper->SetHandoverAlgorithmAttribute("TimeToTrigger", TimeValue(MilliSeconds(256)));

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

    // Configure antennas
    nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(4));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(8));
    nrHelper->SetGnbAntennaAttribute("AntennaElement",
                                     PointerValue(CreateObject<ThreeGppAntennaModel>()));

    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(2));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(4));
    nrHelper->SetUeAntennaAttribute("AntennaElement",
                                    PointerValue(CreateObject<IsotropicAntennaModel>()));

    // Beamforming
    idealBeamformingHelper->SetAttribute("BeamformingMethod",
                                         TypeIdValue(CellScanBeamforming::GetTypeId()));

    // Install NR devices
    NetDeviceContainer gnbNetDevs = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    NetDeviceContainer ueNetDevs = nrHelper->InstallUeDevice(ueNodes, allBwps);

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

        // Map ALL cells of this site to this edge server
        // NOTE: Only map by actual NR cell ID to avoid collisions with node indices
        for (uint32_t sector = 0; sector < 3; ++sector)
        {
            uint32_t nodeIdx = siteId * 3 + sector;
            if (nodeIdx < numCells)
            {
                // Get actual cell ID from the device
                Ptr<NrGnbNetDevice> gnbNetDevice = gnbNetDevs.Get(nodeIdx)->GetObject<NrGnbNetDevice>();
                uint16_t actualCellId = gnbNetDevice->GetCellId();
                g_cellIdToEdgeServer[actualCellId] = edgeServerNodes.Get(siteId);
                g_cellIdToEdgeServerIp[actualCellId] = edgeServerIp;
            }
        }

        NS_LOG_INFO("Edge Server " << siteId << " (IP: " << edgeServerIp
                    << ") serves Site " << siteId << " (cells "
                    << siteId * 3 << "-" << std::min(siteId * 3 + 2, numCells - 1) << ")");
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

            // Map all 3 sectors of this outer site to the inner edge server
            for (uint32_t sector = 0; sector < 3; ++sector)
            {
                uint32_t nodeIdx = outerSiteId * 3 + sector;
                if (nodeIdx < numCells)
                {
                    Ptr<NrGnbNetDevice> gnbNetDevice = gnbNetDevs.Get(nodeIdx)->GetObject<NrGnbNetDevice>();
                    uint16_t actualCellId = gnbNetDevice->GetCellId();
                    g_cellIdToEdgeServer[actualCellId] = innerEdgeNode;
                    g_cellIdToEdgeServerIp[actualCellId] = innerEdgeIp;
                }
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

    // Initialize serving cell tracking and update tunnel endpoint based on actual attachment
    for (uint32_t i = 0; i < ueNetDevs.GetN(); ++i)
    {
        Ptr<NrUeNetDevice> ueDev = ueNetDevs.Get(i)->GetObject<NrUeNetDevice>();
        uint64_t imsi = ueDev->GetImsi();
        uint16_t cellId = ueDev->GetCellId();
        g_ueCurrentServingCell[imsi] = cellId;

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
        g_trajectoryFile.open(trajectoryOutputPath);
        if (g_trajectoryFile.is_open())
        {
            g_trajectoryLoggingEnabled = true;
            // Write CSV header
            g_trajectoryFile << "time_s,imsi,x,y,z,vx,vy,vz,speed,cell_id,site_id\n";

            // Schedule first logging
            Simulator::Schedule(Seconds(trajectoryLogInterval), &LogUeTrajectory, ueNodes, trajectoryLogInterval);

            NS_LOG_UNCOND("Trajectory logging enabled: " << trajectoryOutputPath
                          << " (interval=" << trajectoryLogInterval << "s)");
        }
        else
        {
            NS_LOG_WARN("Failed to open trajectory file: " << trajectoryOutputPath);
        }
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

    // Close trajectory file
    if (g_trajectoryLoggingEnabled && g_trajectoryFile.is_open())
    {
        g_trajectoryFile.close();
        NS_LOG_UNCOND("Trajectory saved to: " << trajectoryOutputPath);
    }

    Simulator::Destroy();
    return 0;
}
