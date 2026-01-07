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

#include <cmath>
#include <fstream>
#include <iomanip>
#include <map>

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
            bufferingDelayMs = timeInBuffer.GetMilliSeconds();
        }
    }

    // Also check if handover is currently active (for real-time tracking)
    if (!duringHandover && g_ueHandoverActive.count(g_ueImsi) && g_ueHandoverActive[g_ueImsi])
    {
        duringHandover = true;
    }

    // Total latency = base latency + buffering delay
    double totalLatencyMs = baseLatency.GetMilliSeconds() + bufferingDelayMs;

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
    uint32_t bgUePacketSize = 1200;          // Background UE packet size in bytes

    // Network delays
    double wanDelayMs = 20.0;                // WAN delay in ms (one-way, central cloud)
    double s1uDelayMs = 1.0;                 // S1-U link delay in ms

    // Handover interruption model
    double handoverInterruptMs = 30.0;       // Handover interruption time in ms (realistic: 30-100ms)

    // SLA parameters
    double slaThresholdMs = 50.0;            // SLA threshold in ms (remote driving: 50ms)

    // Output
    std::string latencyOutputPath = "/tmp/baseline_latency.csv";
    std::string simTag = "baseline";
    bool logging = false;

    // Trajectory logging
    std::string trajectoryOutputPath = "";       // Empty = disabled, path = enable CSV output
    double trajectoryLogInterval = 0.5;          // Logging interval in seconds

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

    // Network
    cmd.AddValue("wanDelayMs", "WAN delay in ms (one-way)", wanDelayMs);
    cmd.AddValue("s1uDelayMs", "S1-U link delay in ms", s1uDelayMs);
    cmd.AddValue("handoverInterruptMs", "Handover interruption time in ms (realistic: 30-100ms)", handoverInterruptMs);

    // SLA
    cmd.AddValue("slaThresholdMs", "SLA latency threshold in ms (violation if exceeded)", slaThresholdMs);

    // Output
    cmd.AddValue("latencyOutputPath", "CSV output filename", latencyOutputPath);
    cmd.AddValue("simTag", "Simulation tag for output files", simTag);
    cmd.AddValue("logging", "Enable detailed logging", logging);

    // Trajectory logging
    cmd.AddValue("trajectoryOutputPath", "Path for trajectory CSV file (empty=disabled)", trajectoryOutputPath);
    cmd.AddValue("trajectoryLogInterval", "Trajectory logging interval in seconds", trajectoryLogInterval);

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
    scenarioParams.SetSectorization(3);  // Triple-sectorized

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

            if (builtinPath == "hexagonal")
            {
                // Circular path visiting all sites (triggers multiple handovers)
                double hexSpeed = 12.0;  // m/s for this path
                double t = 0.0;

                // Start near center (Site 0)
                waypointMm->AddWaypoint(Waypoint(Seconds(t), Vector(0, 50, 1.5)));

                // Move to near Site 1 (433, 250) - distance ~450m
                t += 450.0 / hexSpeed;
                waypointMm->AddWaypoint(Waypoint(Seconds(t), Vector(400, 230, 1.5)));

                // Move to near Site 2 (0, 500) - distance ~470m
                t += 470.0 / hexSpeed;
                waypointMm->AddWaypoint(Waypoint(Seconds(t), Vector(0, 480, 1.5)));

                // Move to near Site 3 (-433, 250) - distance ~470m
                t += 470.0 / hexSpeed;
                waypointMm->AddWaypoint(Waypoint(Seconds(t), Vector(-400, 230, 1.5)));

                // Move to near Site 4 (-433, -250) - distance ~480m
                t += 480.0 / hexSpeed;
                waypointMm->AddWaypoint(Waypoint(Seconds(t), Vector(-400, -230, 1.5)));

                // Move to near Site 5 (0, -500) - distance ~470m
                t += 470.0 / hexSpeed;
                waypointMm->AddWaypoint(Waypoint(Seconds(t), Vector(0, -480, 1.5)));

                // Move to near Site 6 (433, -250) - distance ~470m
                t += 470.0 / hexSpeed;
                waypointMm->AddWaypoint(Waypoint(Seconds(t), Vector(400, -230, 1.5)));

                // Return to center - distance ~470m
                t += 470.0 / hexSpeed;
                waypointMm->AddWaypoint(Waypoint(Seconds(t), Vector(0, 50, 1.5)));

                NS_LOG_UNCOND("Waypoint mobility: hexagonal path visiting all sites (total time=" << t << "s)");
            }
            else if (builtinPath == "linear-y")
            {
                double totalDist = 450.0;
                double travelTime = totalDist / ueSpeed;
                waypointMm->AddWaypoint(Waypoint(Seconds(0), Vector(0, 50, 1.5)));
                waypointMm->AddWaypoint(Waypoint(Seconds(travelTime), Vector(0, 500, 1.5)));
                NS_LOG_UNCOND("Waypoint mobility: linear-y path toward Site 2 (time=" << travelTime << "s)");
            }
            else if (builtinPath == "zigzag")
            {
                waypointMm->AddWaypoint(Waypoint(Seconds(0), Vector(0, 50, 1.5)));
                waypointMm->AddWaypoint(Waypoint(Seconds(20), Vector(150, 150, 1.5)));
                waypointMm->AddWaypoint(Waypoint(Seconds(40), Vector(-150, 250, 1.5)));
                waypointMm->AddWaypoint(Waypoint(Seconds(60), Vector(150, 350, 1.5)));
                waypointMm->AddWaypoint(Waypoint(Seconds(80), Vector(-150, 450, 1.5)));
                NS_LOG_UNCOND("Waypoint mobility: zigzag path crossing sectors");
            }
            else if (builtinPath == "highway")
            {
                double highwaySpeed = 20.0;
                double distance = std::sqrt(2) * 800.0;
                double travelTime = distance / highwaySpeed;

                waypointMm->AddWaypoint(Waypoint(Seconds(0), Vector(-400, -400, 1.5)));
                waypointMm->AddWaypoint(Waypoint(Seconds(travelTime), Vector(400, 400, 1.5)));
                NS_LOG_UNCOND("Waypoint mobility: highway path (speed=" << highwaySpeed
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

                NS_LOG_UNCOND("Waypoint mobility: urban-grid path with " << stopTime
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

    NS_LOG_UNCOND("\n==============================================");
    NS_LOG_UNCOND("Simulation Complete");
    NS_LOG_UNCOND("==============================================");

    // Print latency statistics
    if (g_packetCount > 0)
    {
        double avgLatency = g_latencySum / g_packetCount;
        double slaViolationRate = (100.0 * g_slaViolationCount) / g_packetCount;

        NS_LOG_UNCOND("Packets received: " << g_packetCount);
        NS_LOG_UNCOND("Average latency: " << std::fixed << std::setprecision(3) << avgLatency << " ms");
        NS_LOG_UNCOND("Min latency: " << g_minLatency << " ms");
        NS_LOG_UNCOND("Max latency: " << g_maxLatency << " ms");
        NS_LOG_UNCOND("Packets during handover: " << g_handoverPacketCount);
        NS_LOG_UNCOND("----------------------------------------------");
        NS_LOG_UNCOND("SLA Threshold: " << g_slaThresholdMs << " ms");
        NS_LOG_UNCOND("SLA Violations: " << g_slaViolationCount
                      << " (" << std::fixed << std::setprecision(2) << slaViolationRate << "%)");
    }
    else
    {
        NS_LOG_UNCOND("No packets received!");
    }

    NS_LOG_UNCOND("\nPer-packet results: " << latencyOutputPath);

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
    std::string flowmonFile = simTag + "-flowmon.xml";
    monitor->SerializeToXmlFile(flowmonFile, true, true);
    NS_LOG_UNCOND("\nFlowMonitor XML: " << flowmonFile);

    // Close trajectory file
    if (g_trajectoryLoggingEnabled && g_trajectoryFile.is_open())
    {
        g_trajectoryFile.close();
        NS_LOG_UNCOND("Trajectory saved to: " << trajectoryOutputPath);
    }

    NS_LOG_UNCOND("==============================================\n");

    Simulator::Destroy();
    return 0;
}
