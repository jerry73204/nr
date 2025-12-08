// Copyright (c) 2024
// SPDX-License-Identifier: GPL-2.0-only

/**
 * @ingroup examples
 * @file nr-mec-handover.cc
 * @brief Mobile Edge Computing (MEC) scenario with handover and edge server switching
 *
 * This example demonstrates:
 * - Multiple gNBs (base stations) in a linear deployment
 * - Each gNB connected to a different edge server
 * - A moving UE (simulating a driving car) that performs handovers
 * - Measurement report monitoring for handover prediction
 * - Traffic switching between edge servers during handover
 *
 * Network Topology:
 *
 *   Edge Server 0    Edge Server 1    Edge Server 2
 *        |                |                |
 *      gNB 0            gNB 1            gNB 2
 *        |________________|________________|
 *                         |
 *                    UE (moving car)
 *                    ================>
 *
 * The UE moves from left to right, performing handovers as it travels.
 * Each handover causes traffic to switch to the new serving gNB's edge server.
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
#include "ns3/tap-bridge-module.h"
#include "ns3/nr-module.h"
#include "ns3/point-to-point-module.h"

#include "ue-tunnel-app.h"
#include "edge-tunnel-app.h"
#include "handover-prediction-file.h"

#include <iomanip>
#include <map>
#include <deque>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NrMecHandover");

//==============================================================================
// Global state for handover prediction and edge server tracking
//==============================================================================

// Map from gNB cell ID to edge server node
std::map<uint16_t, Ptr<Node>> g_cellIdToEdgeServer;

// Map from gNB cell ID to edge server IP address
std::map<uint16_t, Ipv4Address> g_cellIdToEdgeServerIp;

// Current serving cell for UE (for tracking)
uint16_t g_currentServingCell = 0;

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

// Global reference to UE tunnel app for dynamic endpoint switching
Ptr<UeTunnelApp> g_ueTunnelApp = nullptr;

//==============================================================================
// Handover Prediction Algorithm (Simple Linear Extrapolation)
//==============================================================================

/**
 * @brief Predict which cell will be the handover target based on RSRP trends
 * @param currentCellId The current serving cell
 * @return Predicted target cell ID (0 if no prediction)
 */
uint16_t
PredictHandoverTarget(uint16_t currentCellId)
{
    uint16_t predictedTarget = 0;
    double bestTrend = 0.0;

    for (const auto& [cellId, history] : g_measurementHistory)
    {
        if (cellId == currentCellId || history.size() < 3)
        {
            continue;
        }

        // Calculate RSRP trend (simple linear regression slope)
        double sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
        size_t n = history.size();
        for (size_t i = 0; i < n; ++i)
        {
            double x = static_cast<double>(i);
            double y = static_cast<double>(history[i].rsrp);
            sumX += x;
            sumY += y;
            sumXY += x * y;
            sumX2 += x * x;
        }
        double slope = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

        // Positive slope means improving signal - potential handover target
        if (slope > bestTrend && history.back().rsrp > -100)  // Also check current RSRP is reasonable
        {
            bestTrend = slope;
            predictedTarget = cellId;
        }
    }

    return predictedTarget;
}

//==============================================================================
// Callback Functions for Monitoring
//==============================================================================

/**
 * @brief Debug callback for tracing IP packets on UE
 */
void
UeIpRxTrace(Ptr<const Packet> packet, Ptr<Ipv4> ipv4, uint32_t interface)
{
    Ipv4Header ipHeader;
    Ptr<Packet> copy = packet->Copy();
    copy->RemoveHeader(ipHeader);
    NS_LOG_UNCOND(Simulator::Now().GetSeconds()
                  << "s [UE IP RX] Interface=" << interface
                  << " Src=" << ipHeader.GetSource()
                  << " Dst=" << ipHeader.GetDestination()
                  << " Proto=" << (uint32_t)ipHeader.GetProtocol());
}

void
UeIpTxTrace(Ptr<const Packet> packet, Ptr<Ipv4> ipv4, uint32_t interface)
{
    Ipv4Header ipHeader;
    Ptr<Packet> copy = packet->Copy();
    copy->RemoveHeader(ipHeader);
    NS_LOG_UNCOND(Simulator::Now().GetSeconds()
                  << "s [UE IP TX] Interface=" << interface
                  << " Src=" << ipHeader.GetSource()
                  << " Dst=" << ipHeader.GetDestination());
}

void
UeIpDropTrace(const Ipv4Header& header, Ptr<const Packet> packet, Ipv4L3Protocol::DropReason reason, Ptr<Ipv4> ipv4, uint32_t interface)
{
    NS_LOG_UNCOND(Simulator::Now().GetSeconds()
                  << "s [UE IP DROP] Interface=" << interface
                  << " Src=" << header.GetSource()
                  << " Dst=" << header.GetDestination()
                  << " Reason=" << reason);
}

// PGW tracing callbacks
void
PgwIpRxTrace(Ptr<const Packet> packet, Ptr<Ipv4> ipv4, uint32_t interface)
{
    Ipv4Header ipHeader;
    Ptr<Packet> copy = packet->Copy();
    copy->RemoveHeader(ipHeader);
    Ipv4Address src = ipHeader.GetSource();
    Ipv4Address dst = ipHeader.GetDestination();
    // Only log traffic between UE (7.0.x.x) and Edge (10.x.x.x)
    uint32_t srcVal = src.Get();
    uint32_t dstVal = dst.Get();
    uint32_t ueSubnet = Ipv4Address("7.0.0.0").Get();
    uint32_t ueMask = Ipv4Mask("255.0.0.0").Get();
    uint32_t edgeSubnet = Ipv4Address("10.0.0.0").Get();
    uint32_t edgeMask = Ipv4Mask("255.0.0.0").Get();
    bool isSrcUe = (srcVal & ueMask) == ueSubnet;
    bool isDstUe = (dstVal & ueMask) == ueSubnet;
    bool isSrcEdge = (srcVal & edgeMask) == edgeSubnet;
    bool isDstEdge = (dstVal & edgeMask) == edgeSubnet;
    // Log if packet is between UE and Edge subnets
    if ((isSrcUe && isDstEdge) || (isSrcEdge && isDstUe))
    {
        NS_LOG_UNCOND(Simulator::Now().GetSeconds()
                      << "s [PGW IP RX] Interface=" << interface
                      << " Src=" << src << " Dst=" << dst);
    }
}

void
PgwIpTxTrace(Ptr<const Packet> packet, Ptr<Ipv4> ipv4, uint32_t interface)
{
    Ipv4Header ipHeader;
    Ptr<Packet> copy = packet->Copy();
    copy->RemoveHeader(ipHeader);
    Ipv4Address src = ipHeader.GetSource();
    Ipv4Address dst = ipHeader.GetDestination();
    // Only log traffic between UE (7.0.x.x) and Edge (10.x.x.x)
    uint32_t srcVal = src.Get();
    uint32_t dstVal = dst.Get();
    uint32_t ueSubnet = Ipv4Address("7.0.0.0").Get();
    uint32_t ueMask = Ipv4Mask("255.0.0.0").Get();
    uint32_t edgeSubnet = Ipv4Address("10.0.0.0").Get();
    uint32_t edgeMask = Ipv4Mask("255.0.0.0").Get();
    bool isSrcUe = (srcVal & ueMask) == ueSubnet;
    bool isDstUe = (dstVal & ueMask) == ueSubnet;
    bool isSrcEdge = (srcVal & edgeMask) == edgeSubnet;
    bool isDstEdge = (dstVal & edgeMask) == edgeSubnet;
    // Log if packet is between UE and Edge subnets
    if ((isSrcUe && isDstEdge) || (isSrcEdge && isDstUe))
    {
        NS_LOG_UNCOND(Simulator::Now().GetSeconds()
                      << "s [PGW IP TX] Interface=" << interface
                      << " Src=" << src << " Dst=" << dst);
    }
}

// Edge Server tracing callbacks
void
EdgeIpRxTrace(Ptr<const Packet> packet, Ptr<Ipv4> ipv4, uint32_t interface)
{
    Ipv4Header ipHeader;
    Ptr<Packet> copy = packet->Copy();
    copy->RemoveHeader(ipHeader);
    Ipv4Address src = ipHeader.GetSource();
    Ipv4Address dst = ipHeader.GetDestination();
    // Only log traffic between UE (7.0.x.x) and Edge (10.x.x.x)
    uint32_t srcVal = src.Get();
    uint32_t dstVal = dst.Get();
    uint32_t ueSubnet = Ipv4Address("7.0.0.0").Get();
    uint32_t ueMask = Ipv4Mask("255.0.0.0").Get();
    uint32_t edgeSubnet = Ipv4Address("10.0.0.0").Get();
    uint32_t edgeMask = Ipv4Mask("255.0.0.0").Get();
    bool isSrcUe = (srcVal & ueMask) == ueSubnet;
    bool isDstUe = (dstVal & ueMask) == ueSubnet;
    bool isSrcEdge = (srcVal & edgeMask) == edgeSubnet;
    bool isDstEdge = (dstVal & edgeMask) == edgeSubnet;
    // Log if packet is between UE and Edge subnets
    if ((isSrcUe && isDstEdge) || (isSrcEdge && isDstUe))
    {
        NS_LOG_UNCOND(Simulator::Now().GetSeconds()
                      << "s [EDGE IP RX] Interface=" << interface
                      << " Src=" << src << " Dst=" << dst
                      << " Proto=" << (uint32_t)ipHeader.GetProtocol());
    }
}

void
EdgeIpTxTrace(Ptr<const Packet> packet, Ptr<Ipv4> ipv4, uint32_t interface)
{
    Ipv4Header ipHeader;
    Ptr<Packet> copy = packet->Copy();
    copy->RemoveHeader(ipHeader);
    Ipv4Address src = ipHeader.GetSource();
    Ipv4Address dst = ipHeader.GetDestination();
    // Only log traffic between UE (7.0.x.x) and Edge (10.x.x.x)
    uint32_t srcVal = src.Get();
    uint32_t dstVal = dst.Get();
    uint32_t ueSubnet = Ipv4Address("7.0.0.0").Get();
    uint32_t ueMask = Ipv4Mask("255.0.0.0").Get();
    uint32_t edgeSubnet = Ipv4Address("10.0.0.0").Get();
    uint32_t edgeMask = Ipv4Mask("255.0.0.0").Get();
    bool isSrcUe = (srcVal & ueMask) == ueSubnet;
    bool isDstUe = (dstVal & ueMask) == ueSubnet;
    bool isSrcEdge = (srcVal & edgeMask) == edgeSubnet;
    bool isDstEdge = (dstVal & edgeMask) == edgeSubnet;
    // Log if packet is between UE and Edge subnets
    if ((isSrcUe && isDstEdge) || (isSrcEdge && isDstUe))
    {
        NS_LOG_UNCOND(Simulator::Now().GetSeconds()
                      << "s [EDGE IP TX] Interface=" << interface
                      << " Src=" << src << " Dst=" << dst);
    }
}

/**
 * @brief Callback when a measurement report is received at the gNB
 * This is the key hook for handover prediction
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

    // Extract serving cell measurements
    // RSRP range: 0-97 maps to -140 to -44 dBm
    // RSRQ range: 0-34 maps to -19.5 to -3 dB
    int servingRsrp = static_cast<int>(meas.measResults.measResultPCell.rsrpResult) - 140;  // Convert to dBm
    double servingRsrq = (static_cast<int>(meas.measResults.measResultPCell.rsrqResult) - 40) / 2.0;  // Convert to dB

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

            // Update history for neighbor cell
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

    // Write prediction to file (only if changed or interval elapsed)
    Ipv4Address targetEdgeIp = Ipv4Address("0.0.0.0");
    double rsrpTrend = 0.0;

    if (predictedTarget > 0 && g_cellIdToEdgeServerIp.count(predictedTarget) > 0)
    {
        targetEdgeIp = g_cellIdToEdgeServerIp[predictedTarget];

        // Calculate trend from history if available
        const auto& history = g_measurementHistory[predictedTarget];
        if (history.size() >= 2)
        {
            rsrpTrend = static_cast<double>(history.back().rsrp - history.front().rsrp)
                        / static_cast<double>(history.size() - 1);
        }

        NS_LOG_INFO("  [PREDICTION] Handover likely to Cell " << predictedTarget
                    << " (Edge: " << targetEdgeIp << ", trend: " << rsrpTrend << " dB/meas)");
    }

    // Update prediction file (low overhead - only writes on change)
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
    NS_LOG_UNCOND(Simulator::Now().GetSeconds()
                  << "s [HANDOVER START] IMSI=" << imsi
                  << " Source=" << sourceCellId << " -> Target=" << targetCellId);

    Ipv4Address sourceEdgeIp = Ipv4Address("0.0.0.0");
    Ipv4Address targetEdgeIp = Ipv4Address("0.0.0.0");

    if (g_cellIdToEdgeServerIp.count(sourceCellId) > 0)
    {
        sourceEdgeIp = g_cellIdToEdgeServerIp[sourceCellId];
    }
    if (g_cellIdToEdgeServerIp.count(targetCellId) > 0)
    {
        targetEdgeIp = g_cellIdToEdgeServerIp[targetCellId];
    }

    if (sourceEdgeIp != Ipv4Address("0.0.0.0") && targetEdgeIp != Ipv4Address("0.0.0.0"))
    {
        NS_LOG_UNCOND("  Edge Server switching: " << sourceEdgeIp << " -> " << targetEdgeIp);
    }

    // Write handover start event to file
    HandoverPredictionFile::GetInstance().WriteHandoverEvent(
        imsi, sourceCellId, targetCellId, sourceEdgeIp, targetEdgeIp, "handover_start");
}

/**
 * @brief Callback when handover completes successfully
 */
void
HandoverEndOkCallback(std::string path, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
    NS_LOG_UNCOND(Simulator::Now().GetSeconds()
                  << "s [HANDOVER SUCCESS] IMSI=" << imsi
                  << " New ServingCell=" << cellId);

    uint16_t previousCell = g_currentServingCell;
    g_currentServingCell = cellId;

    Ipv4Address previousEdgeIp = Ipv4Address("0.0.0.0");
    Ipv4Address newEdgeServerIp = Ipv4Address("0.0.0.0");

    if (g_cellIdToEdgeServerIp.count(previousCell) > 0)
    {
        previousEdgeIp = g_cellIdToEdgeServerIp[previousCell];
    }

    if (g_cellIdToEdgeServerIp.count(cellId) > 0)
    {
        newEdgeServerIp = g_cellIdToEdgeServerIp[cellId];
        NS_LOG_UNCOND("  Now connected to Edge Server at " << newEdgeServerIp);

        // Switch tunnel endpoint to new Edge Server
        if (g_ueTunnelApp)
        {
            g_ueTunnelApp->UpdateTunnelEndpoint(newEdgeServerIp);
        }
    }

    // Write handover success event to file
    HandoverPredictionFile::GetInstance().WriteHandoverEvent(
        imsi, previousCell, cellId, previousEdgeIp, newEdgeServerIp, "handover_success");
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
PrintUePosition(Ptr<Node> ueNode)
{
    Ptr<MobilityModel> mobility = ueNode->GetObject<MobilityModel>();
    Vector pos = mobility->GetPosition();
    Vector vel = mobility->GetVelocity();

    NS_LOG_UNCOND(Simulator::Now().GetSeconds()
                  << "s [UE POSITION] x=" << pos.x << "m, y=" << pos.y << "m"
                  << " | Velocity=" << vel.x << " m/s"
                  << " | ServingCell=" << g_currentServingCell);

    Simulator::Schedule(Seconds(10.0), &PrintUePosition, ueNode);
}

//==============================================================================
// Main Function
//==============================================================================

int
main(int argc, char* argv[])
{
    // Simulation parameters
    uint16_t numGnbs = 2;                    // Number of gNBs (and edge servers)
    double gnbSpacing = 500.0;               // Distance between gNBs in meters
    double simTime = 3000.0;                   // Simulation time in seconds
    // double ueSpeed = 20.0;                   // UE speed in m/s (72 km/h)
    double ueSpeed = 2;
    double ueStartX = 50.0;                  // UE starting X position (near first gNB for stable attachment)
    bool logging = true;                     // Enable detailed logging
    std::string handoverAlgo = "A3Rsrp";     // Handover algorithm: A3Rsrp or A2A4Rsrq

    // NR parameters
    double centralFrequency = 3.5e9;         // 3.5 GHz (n78 band)
    double bandwidth = 20e6;                 // 20 MHz
    double gnbTxPower = 43.0;                // gNB TX power in dBm
    uint16_t numerology = 1;                 // NR numerology (1 = 30 kHz SCS)

    // Tap bridge parameters
    bool enableTap = false;                  // Enable tap bridge for external connectivity
    std::string tapUeDevice = "tap_z_pre_sub";   // Tap device for UE
    std::string tapEdge0Device = "tap_edge0";    // Tap device for edge server 0
    std::string tapEdge1Device = "tap_edge1";    // Tap device for edge server 1

    // Handover prediction file parameters
    std::string predictionFilePath = "/tmp/ns3_handover";  // Base path for prediction files
    double predictionWriteIntervalMs = 100.0;              // Minimum interval between file writes

    CommandLine cmd(__FILE__);
    cmd.AddValue("numGnbs", "Number of gNBs", numGnbs);
    cmd.AddValue("gnbSpacing", "Distance between gNBs (m)", gnbSpacing);
    cmd.AddValue("simTime", "Simulation time (s)", simTime);
    cmd.AddValue("ueSpeed", "UE speed (m/s)", ueSpeed);
    cmd.AddValue("logging", "Enable detailed logging", logging);
    cmd.AddValue("handoverAlgo", "Handover algorithm (A3Rsrp or A2A4Rsrq)", handoverAlgo);
    cmd.AddValue("centralFrequency", "Central frequency (Hz)", centralFrequency);
    cmd.AddValue("bandwidth", "System bandwidth (Hz)", bandwidth);
    cmd.AddValue("gnbTxPower", "gNB TX power (dBm)", gnbTxPower);
    cmd.AddValue("enableTap", "Enable tap bridge for external connectivity", enableTap);
    cmd.AddValue("tapUeDevice", "Tap device name for UE", tapUeDevice);
    cmd.AddValue("tapEdge0Device", "Tap device name for edge server 0", tapEdge0Device);
    cmd.AddValue("tapEdge1Device", "Tap device name for edge server 1", tapEdge1Device);
    cmd.AddValue("predictionFilePath", "Base path for handover prediction files", predictionFilePath);
    cmd.AddValue("predictionWriteIntervalMs", "Min interval between prediction file writes (ms)", predictionWriteIntervalMs);
    cmd.Parse(argc, argv);

    // Initialize handover prediction file system
    HandoverPredictionFile::GetInstance().Initialize(
        predictionFilePath,
        MilliSeconds(predictionWriteIntervalMs));
    NS_LOG_UNCOND("Handover prediction file initialized: " << predictionFilePath << ".json");

    // Configure real-time simulator and checksums when tap bridge is enabled
    if (enableTap)
    {
        GlobalValue::Bind("SimulatorImplementationType", StringValue("ns3::RealtimeSimulatorImpl"));
        GlobalValue::Bind("ChecksumEnabled", BooleanValue(true));
        NS_LOG_UNCOND("Tap bridge enabled - using real-time simulator");
    }

    if (logging)
    {
        LogComponentEnable("NrMecHandover", LOG_LEVEL_INFO);
    }

    NS_LOG_INFO("==============================================");
    NS_LOG_INFO("NR MEC Handover Simulation");
    NS_LOG_INFO("==============================================");
    NS_LOG_INFO("Number of gNBs: " << numGnbs);
    NS_LOG_INFO("gNB spacing: " << gnbSpacing << " m");
    NS_LOG_INFO("UE speed: " << ueSpeed << " m/s (" << ueSpeed * 3.6 << " km/h)");
    NS_LOG_INFO("Handover algorithm: " << handoverAlgo);
    NS_LOG_INFO("==============================================");

    //--------------------------------------------------------------------------
    // Create nodes: gNBs, UE, and edge servers
    //--------------------------------------------------------------------------
    NodeContainer gnbNodes;
    gnbNodes.Create(numGnbs);

    NodeContainer ueNodes;
    ueNodes.Create(1);

    NodeContainer edgeServerNodes;
    edgeServerNodes.Create(numGnbs);  // One edge server per gNB

    // Ghost nodes for tap bridge (one per edge server)
    // Architecture: PGW <--CSMA1--> EdgeServer <--CSMA2--> GhostNode <--TapBridge--> Docker
    NodeContainer ghostNodes;
    ghostNodes.Create(numGnbs);

    //--------------------------------------------------------------------------
    // Set up mobility
    //--------------------------------------------------------------------------

    // gNBs: Fixed positions in a line
    MobilityHelper gnbMobility;
    gnbMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    Ptr<ListPositionAllocator> gnbPositionAlloc = CreateObject<ListPositionAllocator>();
    for (uint16_t i = 0; i < numGnbs; ++i)
    {
        double x = i * gnbSpacing;
        gnbPositionAlloc->Add(Vector(x, 0.0, 25.0));  // 25m height
        NS_LOG_INFO("gNB " << i << " position: (" << x << ", 0, 25)");
    }
    gnbMobility.SetPositionAllocator(gnbPositionAlloc);
    gnbMobility.Install(gnbNodes);

    // UE: Moving from left to right at constant velocity
    MobilityHelper ueMobility;
    ueMobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    Ptr<ListPositionAllocator> uePositionAlloc = CreateObject<ListPositionAllocator>();
    uePositionAlloc->Add(Vector(ueStartX, 50.0, 1.5));  // Start position, 1.5m height
    ueMobility.SetPositionAllocator(uePositionAlloc);
    ueMobility.Install(ueNodes);

    // Set UE velocity
    Ptr<ConstantVelocityMobilityModel> ueMobilityModel =
        ueNodes.Get(0)->GetObject<ConstantVelocityMobilityModel>();
    ueMobilityModel->SetVelocity(Vector(ueSpeed, 0.0, 0.0));

    NS_LOG_INFO("UE starting at (" << ueStartX << ", 50, 1.5) moving at " << ueSpeed << " m/s");

    // Edge servers: Fixed positions (conceptually co-located with gNBs)
    MobilityHelper edgeMobility;
    edgeMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    edgeMobility.Install(edgeServerNodes);
    edgeMobility.Install(ghostNodes);

    //--------------------------------------------------------------------------
    // Set up NR network
    //--------------------------------------------------------------------------
    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper> idealBeamformingHelper = CreateObject<IdealBeamformingHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();

    nrHelper->SetBeamformingHelper(idealBeamformingHelper);
    nrHelper->SetEpcHelper(epcHelper);

    // Configure handover algorithm
    if (handoverAlgo == "A3Rsrp")
    {
        nrHelper->SetHandoverAlgorithmType("ns3::NrA3RsrpHandoverAlgorithm");
        nrHelper->SetHandoverAlgorithmAttribute("Hysteresis", DoubleValue(3.0));
        nrHelper->SetHandoverAlgorithmAttribute("TimeToTrigger", TimeValue(MilliSeconds(256)));
    }
    else if (handoverAlgo == "A2A4Rsrq")
    {
        nrHelper->SetHandoverAlgorithmType("ns3::NrA2A4RsrqHandoverAlgorithm");
        nrHelper->SetHandoverAlgorithmAttribute("ServingCellThreshold", UintegerValue(30));
        nrHelper->SetHandoverAlgorithmAttribute("NeighbourCellOffset", UintegerValue(1));
    }

    // Spectrum configuration
    BandwidthPartInfoPtrVector allBwps;
    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(centralFrequency, bandwidth, 1);
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);

    // Configure channel
    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigureFactories("UMa", "Default", "ThreeGpp");  // Urban Macro
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

    //--------------------------------------------------------------------------
    // Set up IP networking for UE
    //--------------------------------------------------------------------------
    InternetStackHelper internet;
    internet.Install(ueNodes);

    Ipv4InterfaceContainer ueIpIfaces = epcHelper->AssignUeIpv4Address(ueNetDevs);
    Ipv4Address ueIpAddr = ueIpIfaces.GetAddress(0);  //7.x.x.x

    // Set default route for UE
    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> ueStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(ueNodes.Get(0)->GetObject<Ipv4>());
    ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);

    //--------------------------------------------------------------------------
    // Set up tap bridge for external connectivity (if enabled)
    //--------------------------------------------------------------------------
    // Store UE's inner CSMA device for tunnel app (needs to be accessible later)
    Ptr<NetDevice> ueInnerCsmaDevice = nullptr;

    if (enableTap)
    {
        // Set up Ghost node
        NodeContainer tapLinkNodes;
        tapLinkNodes.Add(ueNodes.Get(0));
        Ptr<Node> ghostNode = CreateObject<Node>();
        tapLinkNodes.Add(ghostNode);

        // Ghost node doesn't need Internet stack - it's a pure L2 bridge

        CsmaHelper tapCsma;
        tapCsma.SetChannelAttribute("DataRate", StringValue("10Gbps"));
        tapCsma.SetChannelAttribute("Delay", StringValue("0"));

        NetDeviceContainer tapDevices = tapCsma.Install(tapLinkNodes);

        // Store UE's CSMA device for tunnel app
        ueInnerCsmaDevice = tapDevices.Get(0);

        // Only assign IP to UE's CSMA interface, not ghost node
        // Ghost node acts as L2 bridge - external Linux host will have its own IP
        Ipv4AddressHelper ipv4Tap;
        ipv4Tap.SetBase("7.0.1.0", "255.255.255.0");
        Ipv4InterfaceContainer tapIpIfaces = ipv4Tap.Assign(tapDevices.Get(0));  // UE only
        Ipv4Address ueLanIp = tapIpIfaces.GetAddress(0);

        Ptr<Ipv4> ipv4 = ueNodes.Get(0)->GetObject<Ipv4>();
        ipv4->SetAttribute("IpForward", BooleanValue(true));

        // Disable forwarding specifically on the CSMA interface
        // This prevents normal IP forwarding of packets from client subnet
        // The tunnel app will handle forwarding these packets via UDP encapsulation
        int32_t csmaIfIndex = ipv4->GetInterfaceForDevice(tapDevices.Get(0));
        if (csmaIfIndex >= 0)
        {
            ipv4->SetForwarding(csmaIfIndex, false);
            NS_LOG_UNCOND("Disabled IP forwarding on UE CSMA interface " << csmaIfIndex);
        }

        TapBridgeHelper tapBridge;
        tapBridge.SetAttribute("Mode", StringValue("UseBridge"));

        // Install tap bridge on UE
        tapBridge.SetAttribute("DeviceName", StringValue(tapUeDevice));

        tapBridge.Install(tapLinkNodes.Get(1), tapDevices.Get(1));

        NS_LOG_UNCOND("Tap bridge installed on UE (UseBridge mode)");
        NS_LOG_UNCOND("  UE WAN IP (5G): " << ueIpAddr);
        NS_LOG_UNCOND("  UE LAN IP (Tap): " << ueLanIp);
        NS_LOG_UNCOND("  Tap device: " << tapUeDevice);
        NS_LOG_UNCOND("  Please set Linux tap0 IP to 192.168.1.2/24 and Gateway to " << ueLanIp);

        // Debug: Print UE routing table
        NS_LOG_UNCOND("\nUE Routing Table:");
        Ptr<Ipv4> ueIpv4 = ueNodes.Get(0)->GetObject<Ipv4>();
        NS_LOG_UNCOND("UE has " << ueIpv4->GetNInterfaces() << " interfaces:");
        for (uint32_t i = 0; i < ueIpv4->GetNInterfaces(); ++i)
        {
            NS_LOG_UNCOND("  Interface " << i << ": " << ueIpv4->GetAddress(i, 0).GetLocal());
        }
        Ptr<Ipv4StaticRouting> ueRouting =
            ipv4RoutingHelper.GetStaticRouting(ueIpv4);
        ueRouting->PrintRoutingTable(Create<OutputStreamWrapper>(&std::cout));

        // Connect IP trace callbacks to UE for debugging
        ueIpv4->TraceConnectWithoutContext("Rx", MakeCallback(&UeIpRxTrace));
        ueIpv4->TraceConnectWithoutContext("Tx", MakeCallback(&UeIpTxTrace));
        ueIpv4->TraceConnectWithoutContext("Drop", MakeCallback(&UeIpDropTrace));
        NS_LOG_UNCOND("UE IP tracing enabled");
    }

    //--------------------------------------------------------------------------
    // Set up edge servers and connect them to gNBs via PGW
    // Using CSMA for tap bridge compatibility (P2P uses PPP which can't handle Ethernet frames)
    //
    // Ghost Node Architecture:
    //   PGW <--CSMA1--> EdgeServer <--CSMA2--> GhostNode <--TapBridge--> Docker
    //
    // This separates the PGW-facing interface from the tap-facing interface,
    // allowing EdgeTunnelApp to properly intercept packets in both directions.
    //--------------------------------------------------------------------------
    Ptr<Node> pgw = epcHelper->GetPgwNode();
    internet.Install(edgeServerNodes);
    // NOTE: Do NOT install internet stack on ghostNodes - they are pure L2 bridges
    // Installing IP stack causes ARP confusion and routing loops
    // internet.Install(ghostNodes);  // REMOVED - ghost nodes are L2 only

    // Store edge server devices for tunnel installation
    std::vector<Ptr<NetDevice>> edgeServerOuterDevices;  // EdgeServer's device on PGW-facing CSMA
    std::vector<Ptr<NetDevice>> edgeServerInnerDevices;  // EdgeServer's device on Ghost-facing CSMA
    std::vector<Ptr<NetDevice>> ghostDevices;            // GhostNode's device (for TapBridge)

    for (uint16_t i = 0; i < numGnbs; ++i)
    {
        CsmaHelper csma;
        csma.SetChannelAttribute("DataRate", DataRateValue(DataRate("10Gbps")));
        // Use small non-zero delay to avoid CSMA timing issues
        // MilliSeconds(1) is too high for real-time responsiveness
        csma.SetChannelAttribute("Delay", TimeValue(MicroSeconds(10)));

        //----------------------------------------------------------------------
        // CSMA1: PGW <-> EdgeServer (for tunnel packets from/to UE)
        //----------------------------------------------------------------------
        NodeContainer pgwEdgeLink;
        pgwEdgeLink.Add(pgw);
        pgwEdgeLink.Add(edgeServerNodes.Get(i));

        NetDeviceContainer pgwEdgeDevices = csma.Install(pgwEdgeLink);

        // Assign IP addresses on PGW-EdgeServer segment
        std::ostringstream subnet;
        subnet << "10." << (i + 1) << ".0.0";
        Ipv4AddressHelper ipv4Helper;
        ipv4Helper.SetBase(subnet.str().c_str(), "255.255.255.0");

        Ipv4InterfaceContainer pgwEdgeIpIfaces = ipv4Helper.Assign(pgwEdgeDevices);
        Ipv4Address edgeServerIp = pgwEdgeIpIfaces.GetAddress(1);

        // Store EdgeServer's outer device (PGW-facing)
        edgeServerOuterDevices.push_back(pgwEdgeDevices.Get(1));  // EdgeServer's device

        //----------------------------------------------------------------------
        // CSMA2: EdgeServer <-> GhostNode (for tap bridge to Docker)
        //----------------------------------------------------------------------
        NodeContainer edgeGhostLink;
        edgeGhostLink.Add(edgeServerNodes.Get(i));
        edgeGhostLink.Add(ghostNodes.Get(i));

        NetDeviceContainer edgeGhostDevices = csma.Install(edgeGhostLink);

        // Assign IP addresses on EdgeServer-GhostNode segment
        // Use 10.x.1.0/24 subnet (matching container setup: Docker router at 10.x.1.3)
        // Pattern: EdgeServer at 10.x.1.1, Docker (via tap) at 10.x.1.3
        // NOTE: Only assign IP to EdgeServer, NOT to GhostNode (it's a pure L2 bridge)
        std::ostringstream ghostSubnet;
        ghostSubnet << "10." << (i + 1) << ".1.0";
        Ipv4AddressHelper ghostIpHelper;
        ghostIpHelper.SetBase(ghostSubnet.str().c_str(), "255.255.255.0");

        // Only assign IP to EdgeServer's inner device (index 0), not GhostNode (index 1)
        Ipv4InterfaceContainer edgeGhostIpIfaces = ghostIpHelper.Assign(edgeGhostDevices.Get(0));

        // Store EdgeServer's inner device (Ghost-facing) and GhostNode's device
        edgeServerInnerDevices.push_back(edgeGhostDevices.Get(0));  // EdgeServer's device
        ghostDevices.push_back(edgeGhostDevices.Get(1));            // GhostNode's device (for TapBridge)

        // Get cell ID for this gNB
        Ptr<NrGnbNetDevice> gnbNetDevice = gnbNetDevs.Get(i)->GetObject<NrGnbNetDevice>();
        uint16_t cellId = gnbNetDevice->GetCellId();

        // Store mapping
        g_cellIdToEdgeServer[cellId] = edgeServerNodes.Get(i);
        g_cellIdToEdgeServerIp[cellId] = edgeServerIp;

        NS_LOG_INFO("Edge Server " << i << " (IP: " << edgeServerIp << ") connected to gNB "
                    << i << " (CellId: " << cellId << ")");
        NS_LOG_INFO("  Ghost segment: " << ghostSubnet.str() << "/24");

        // Set up routing from edge server to UE network (via PGW)
        Ipv4StaticRoutingHelper ipv4RoutingHelper;
        Ptr<Ipv4StaticRouting> edgeRouting =
            ipv4RoutingHelper.GetStaticRouting(edgeServerNodes.Get(i)->GetObject<Ipv4>());
        edgeRouting->SetDefaultRoute(pgwEdgeIpIfaces.GetAddress(0), 1);

        // Note: We intentionally do NOT add a route to 7.0.1.0/24 on EdgeServer
        // The default route would forward these packets via PGW, but EdgeTunnelApp handles them
        // via tunneling. The duplicate TX via IP stack is harmless but wastes some bandwidth.
        // A proper fix would require modifying ns-3's IP forwarding behavior.

        // NOTE: GhostNode has no IP stack - it's a pure L2 bridge, no routing needed

        // Add route on PGW for ghost segment (10.x.1.0/24 via EdgeServer)
        // This enables edge-to-edge communication (Docker Router 0 <-> Docker Router 1)
        Ptr<Ipv4StaticRouting> pgwRouting =
            ipv4RoutingHelper.GetStaticRouting(pgw->GetObject<Ipv4>());
        pgwRouting->AddNetworkRouteTo(
            Ipv4Address(ghostSubnet.str().c_str()),
            Ipv4Mask("255.255.255.0"),
            pgwEdgeIpIfaces.GetAddress(1),  // Via EdgeServer (10.x.0.2)
            pgw->GetObject<Ipv4>()->GetInterfaceForAddress(pgwEdgeIpIfaces.GetAddress(0))
        );
        NS_LOG_INFO("  PGW route added: " << ghostSubnet.str() << "/24 via " << pgwEdgeIpIfaces.GetAddress(1));
    }

    // Install tap bridges on GHOST NODES (not edge servers)
    // Ghost nodes act as pure L2 bridges between Docker and EdgeServer
    if (enableTap && numGnbs >= 2)
    {
        TapBridgeHelper tapBridge;
        tapBridge.SetAttribute("Mode", StringValue("UseBridge"));

        // Install tap bridge on ghost node 0
        tapBridge.SetAttribute("DeviceName", StringValue(tapEdge0Device));
        tapBridge.Install(ghostNodes.Get(0), ghostDevices[0]);
        NS_LOG_UNCOND("Tap bridge installed on Ghost Node 0");
        NS_LOG_UNCOND("  Edge Server 0 IP: " << g_cellIdToEdgeServerIp.begin()->second);
        NS_LOG_UNCOND("  Tap device: " << tapEdge0Device);

        // Install tap bridge on ghost node 1
        tapBridge.SetAttribute("DeviceName", StringValue(tapEdge1Device));
        tapBridge.Install(ghostNodes.Get(1), ghostDevices[1]);
        NS_LOG_UNCOND("Tap bridge installed on Ghost Node 1");
        NS_LOG_UNCOND("  Tap device: " << tapEdge1Device);

        // Add IP tracing on Edge Server 0 for debugging
        Ptr<Ipv4> edgeIpv4 = edgeServerNodes.Get(0)->GetObject<Ipv4>();
        edgeIpv4->TraceConnectWithoutContext("Rx", MakeCallback(&EdgeIpRxTrace));
        edgeIpv4->TraceConnectWithoutContext("Tx", MakeCallback(&EdgeIpTxTrace));
        NS_LOG_UNCOND("Edge Server IP tracing enabled");
    }

    //--------------------------------------------------------------------------
    // Set up UDP Tunnel Applications (if tap enabled)
    // These tunnel client packets through the 5G network using IP-in-UDP encapsulation
    // to bypass GTP filtering that drops packets with non-UE source IPs
    //--------------------------------------------------------------------------
    if (enableTap && ueInnerCsmaDevice)
    {
        // Install UeTunnelApp on UE node
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
        g_ueTunnelApp = ueTunnelApp;

        NS_LOG_UNCOND("\nUE Tunnel App installed:");
        NS_LOG_UNCOND("  Inner device: CSMA (7.0.1.1)");
        NS_LOG_UNCOND("  Client subnet: 7.0.1.0/24");
        NS_LOG_UNCOND("  Initial tunnel endpoint: " << edgeServer0Ip << ":5000");
        NS_LOG_UNCOND("  (Endpoint will switch dynamically on handover)");

        // Install EdgeTunnelApp on each edge server
        // Ghost Node Architecture:
        //   PGW <--CSMA1--> EdgeServer <--CSMA2--> GhostNode <--TapBridge--> Docker
        //                       |
        //                  EdgeTunnelApp
        //   - OuterDevice: EdgeServer's device on CSMA1 (receives tunnel packets from UE)
        //   - InnerDevice: EdgeServer's device on CSMA2 (forwards to GhostNode -> Docker)
        for (uint16_t i = 0; i < numGnbs && i < edgeServerInnerDevices.size(); ++i)
        {
            Ptr<EdgeTunnelApp> edgeTunnelApp = CreateObject<EdgeTunnelApp>();

            // Set both devices (both belong to EdgeServer node):
            // - Outer: EdgeServer's CSMA device facing PGW (tunnel packets arrive here)
            // - Inner: EdgeServer's CSMA device facing GhostNode (forwards to Docker)
            edgeTunnelApp->SetOuterDevice(edgeServerOuterDevices[i]);
            edgeTunnelApp->SetInnerDevice(edgeServerInnerDevices[i]);

            // Set inner subnet (10.x.1.0/24) - packets to other subnets will be dropped
            // This ensures connection breaks after handover instead of routing via PGW
            std::ostringstream innerSubnetStr;
            innerSubnetStr << "10." << (i + 1) << ".1.0";
            edgeTunnelApp->SetInnerSubnet(Ipv4Address(innerSubnetStr.str().c_str()),
                                          Ipv4Mask("255.255.255.0"));

            // Add tunnel mapping: packets to 7.0.1.x should go to UE NR IP (7.0.0.2)
            edgeTunnelApp->AddTunnelMapping(
                Ipv4Address("7.0.1.0"),
                Ipv4Mask("255.255.255.0"),
                ueIpAddr  // UE's NR interface IP (7.0.0.2)
            );
            edgeTunnelApp->SetLocalPort(5000);
            edgeTunnelApp->SetTunnelPort(5000);

            edgeServerNodes.Get(i)->AddApplication(edgeTunnelApp);
            edgeTunnelApp->SetStartTime(Seconds(1.0));
            edgeTunnelApp->SetStopTime(Seconds(simTime));

            NS_LOG_UNCOND("Edge Tunnel App " << i << " installed:");
            NS_LOG_UNCOND("  Outer device: " << edgeServerOuterDevices[i]->GetAddress());
            NS_LOG_UNCOND("  Inner device: " << edgeServerInnerDevices[i]->GetAddress());
            NS_LOG_UNCOND("  Mapping: 7.0.1.0/24 -> " << ueIpAddr);
        }
    }

    // Add route on PGW for UE's tap subnet (7.0.1.0/24) via UE (7.0.0.2)
    // This provides a more specific route for the client subnet
    if (enableTap)
    {
        Ptr<Ipv4StaticRouting> pgwRouting =
            ipv4RoutingHelper.GetStaticRouting(pgw->GetObject<Ipv4>());
        pgwRouting->AddNetworkRouteTo(Ipv4Address("7.0.1.0"),
                                       Ipv4Mask("255.255.255.0"),
                                       Ipv4Address("7.0.0.2"),
                                       1);  // Interface to SGW/UE network
        NS_LOG_UNCOND("Added route on PGW: 7.0.1.0/24 via UE (7.0.0.2)");
    }

    //--------------------------------------------------------------------------
    // Attach UE to the closest gNB initially
    //--------------------------------------------------------------------------
    nrHelper->AttachToClosestGnb(ueNetDevs, gnbNetDevs);

    //--------------------------------------------------------------------------
    // Add X2 interfaces between gNBs for handover
    //--------------------------------------------------------------------------
    for (uint16_t i = 0; i < numGnbs; ++i)
    {
        for (uint16_t j = i + 1; j < numGnbs; ++j)
        {
            epcHelper->AddX2Interface(gnbNodes.Get(i), gnbNodes.Get(j));
        }
    }

    //--------------------------------------------------------------------------
    // Configure measurement reporting for handover
    //--------------------------------------------------------------------------

    // Configure A3 event (neighbor better than serving by offset)
    NrRrcSap::ReportConfigEutra reportConfigA3;
    reportConfigA3.triggerType = NrRrcSap::ReportConfigEutra::EVENT;
    reportConfigA3.eventId = NrRrcSap::ReportConfigEutra::EVENT_A3;
    reportConfigA3.a3Offset = 0;
    reportConfigA3.hysteresis = 6;  // 3 dB (value is in 0.5 dB steps)
    reportConfigA3.timeToTrigger = 256;
    reportConfigA3.reportOnLeave = false;
    reportConfigA3.triggerQuantity = NrRrcSap::ReportConfigEutra::RSRP;
    reportConfigA3.reportQuantity = NrRrcSap::ReportConfigEutra::BOTH;
    reportConfigA3.maxReportCells = 8;
    reportConfigA3.reportInterval = NrRrcSap::ReportConfigEutra::MS480;

    // Add measurement config to all gNBs
    for (uint32_t i = 0; i < gnbNodes.GetN(); ++i)
    {
        Ptr<NrGnbRrc> gnbRrc = gnbNodes.Get(i)->GetDevice(0)->GetObject<NrGnbNetDevice>()->GetRrc();
        gnbRrc->AddUeMeasReportConfig(reportConfigA3);
    }

    //--------------------------------------------------------------------------
    // Connect trace sources for monitoring
    //--------------------------------------------------------------------------

    // Connect measurement report callback to all gNBs
    for (uint32_t i = 0; i < gnbNodes.GetN(); ++i)
    {
        std::ostringstream path;
        path << "/NodeList/" << gnbNodes.Get(i)->GetId()
             << "/DeviceList/0/NrGnbRrc/RecvMeasurementReport";
        Config::Connect(path.str(), MakeCallback(&MeasurementReportCallback));
    }

    // Connect handover callbacks to UE
    std::ostringstream uePath;
    uePath << "/NodeList/" << ueNodes.Get(0)->GetId() << "/DeviceList/0/NrUeRrc/";

    Config::Connect(uePath.str() + "HandoverStart", MakeCallback(&HandoverStartCallback));
    Config::Connect(uePath.str() + "HandoverEndOk", MakeCallback(&HandoverEndOkCallback));
    Config::Connect(uePath.str() + "HandoverEndError", MakeCallback(&HandoverEndErrorCallback));

    //--------------------------------------------------------------------------
    // Debug: Print PGW routing table
    //--------------------------------------------------------------------------
    Ptr<Ipv4> pgwIpv4 = pgw->GetObject<Ipv4>();
    NS_LOG_UNCOND("PGW has " << pgwIpv4->GetNInterfaces() << " interfaces:");
    for (uint32_t i = 0; i < pgwIpv4->GetNInterfaces(); ++i)
    {
        NS_LOG_UNCOND("  Interface " << i << ": " << pgwIpv4->GetAddress(i, 0).GetLocal());
    }

    Ptr<Ipv4StaticRouting> pgwRouting =
        ipv4RoutingHelper.GetStaticRouting(pgwIpv4);
    pgwRouting->PrintRoutingTable(Create<OutputStreamWrapper>(&std::cout));

    // Connect PGW IP trace callbacks for debugging
    if (enableTap)
    {
        pgwIpv4->TraceConnectWithoutContext("Rx", MakeCallback(&PgwIpRxTrace));
        pgwIpv4->TraceConnectWithoutContext("Tx", MakeCallback(&PgwIpTxTrace));
        NS_LOG_UNCOND("PGW IP tracing enabled");
    }

    //--------------------------------------------------------------------------
    // Schedule UE position printing
    //--------------------------------------------------------------------------
    Simulator::Schedule(Seconds(10.0), &PrintUePosition, ueNodes.Get(0));

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

    Simulator::Destroy();
    return 0;
}
