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
    if (predictedTarget > 0)
    {
        NS_LOG_INFO("  [PREDICTION] Handover likely to Cell " << predictedTarget);

        // Get the edge server for the predicted target
        if (g_cellIdToEdgeServerIp.count(predictedTarget) > 0)
        {
            NS_LOG_INFO("  [PREDICTION] Traffic will switch to Edge Server at "
                        << g_cellIdToEdgeServerIp[predictedTarget]);
        }
    }
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

    if (g_cellIdToEdgeServerIp.count(sourceCellId) > 0 &&
        g_cellIdToEdgeServerIp.count(targetCellId) > 0)
    {
        NS_LOG_UNCOND("  Edge Server switching: "
                      << g_cellIdToEdgeServerIp[sourceCellId] << " -> "
                      << g_cellIdToEdgeServerIp[targetCellId]);
    }
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

    g_currentServingCell = cellId;

    if (g_cellIdToEdgeServerIp.count(cellId) > 0)
    {
        NS_LOG_UNCOND("  Now connected to Edge Server at " << g_cellIdToEdgeServerIp[cellId]);
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
PrintUePosition(Ptr<Node> ueNode)
{
    Ptr<MobilityModel> mobility = ueNode->GetObject<MobilityModel>();
    Vector pos = mobility->GetPosition();
    Vector vel = mobility->GetVelocity();

    NS_LOG_UNCOND(Simulator::Now().GetSeconds()
                  << "s [UE POSITION] x=" << pos.x << "m, y=" << pos.y << "m"
                  << " | Velocity=" << vel.x << " m/s"
                  << " | ServingCell=" << g_currentServingCell);

    Simulator::Schedule(Seconds(1.0), &PrintUePosition, ueNode);
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

    // Traffic parameters
    uint32_t packetSize = 1024;              // UDP packet size
    double dataRate = 10.0;                  // Data rate in Mbps

    // Tap bridge parameters
    bool enableTap = false;                  // Enable tap bridge for external connectivity
    std::string tapUeDevice = "tap_z_pre_sub";   // Tap device for UE
    std::string tapEdge0Device = "tap_edge0";    // Tap device for edge server 0
    std::string tapEdge1Device = "tap_edge1";    // Tap device for edge server 1

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
    cmd.Parse(argc, argv);

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

        // Only assign IP to UE's CSMA interface, not ghost node
        // Ghost node acts as L2 bridge - external Linux host will have its own IP
        Ipv4AddressHelper ipv4Tap;
        ipv4Tap.SetBase("10.0.0.0", "255.255.255.0");
        Ipv4InterfaceContainer tapIpIfaces = ipv4Tap.Assign(tapDevices.Get(0));  // UE only
        Ipv4Address ueLanIp = tapIpIfaces.GetAddress(0);

        Ptr<Ipv4> ipv4 = ueNodes.Get(0)->GetObject<Ipv4>();
        ipv4->SetAttribute("IpForward", BooleanValue(true));

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
    }

    //--------------------------------------------------------------------------
    // Set up edge servers and connect them to gNBs via PGW
    // Using CSMA for tap bridge compatibility (P2P uses PPP which can't handle Ethernet frames)
    //--------------------------------------------------------------------------
    Ptr<Node> pgw = epcHelper->GetPgwNode();
    internet.Install(edgeServerNodes);

    // Store edge server devices for tap bridge installation
    std::vector<Ptr<NetDevice>> edgeServerDevices;

    for (uint16_t i = 0; i < numGnbs; ++i)
    {
        // Create a CSMA network for each edge server connection to PGW
        // This allows tap bridge to work properly (CSMA handles Ethernet frames)
        CsmaHelper csma;
        csma.SetChannelAttribute("DataRate", DataRateValue(DataRate("10Gbps")));
        csma.SetChannelAttribute("Delay", TimeValue(MilliSeconds(1)));

        NodeContainer edgeLink;
        edgeLink.Add(pgw);
        edgeLink.Add(edgeServerNodes.Get(i));

        NetDeviceContainer edgeDevices = csma.Install(edgeLink);

        // Assign IP addresses
        std::ostringstream subnet;
        subnet << "10." << (i + 1) << ".0.0";
        Ipv4AddressHelper ipv4Helper;
        ipv4Helper.SetBase(subnet.str().c_str(), "255.255.255.0");

        Ipv4InterfaceContainer edgeIpIfaces = ipv4Helper.Assign(edgeDevices);
        Ipv4Address edgeServerIp = edgeIpIfaces.GetAddress(1);

        // Store the edge server's CSMA device (index 1; index 0 is PGW side)
        edgeServerDevices.push_back(edgeDevices.Get(1));

        // Get cell ID for this gNB
        Ptr<NrGnbNetDevice> gnbNetDevice = gnbNetDevs.Get(i)->GetObject<NrGnbNetDevice>();
        uint16_t cellId = gnbNetDevice->GetCellId();

        // Store mapping
        g_cellIdToEdgeServer[cellId] = edgeServerNodes.Get(i);
        g_cellIdToEdgeServerIp[cellId] = edgeServerIp;

        NS_LOG_INFO("Edge Server " << i << " (IP: " << edgeServerIp << ") connected to gNB "
                    << i << " (CellId: " << cellId << ")");

        // Set up routing from edge server to UE network
        // Ptr<Ipv4StaticRouting> edgeRouting =
        //     ipv4RoutingHelper.GetStaticRouting(edgeServerNodes.Get(i)->GetObject<Ipv4>());
        // edgeRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);
        Ipv4StaticRoutingHelper ipv4RoutingHelper;
        Ptr<Ipv4StaticRouting> edgeRouting =
            ipv4RoutingHelper.GetStaticRouting(edgeServerNodes.Get(i)->GetObject<Ipv4>());
        edgeRouting->SetDefaultRoute(edgeIpIfaces.GetAddress(0), 1);
    }

    // Install tap bridges on edge servers (if enabled)
    if (enableTap && numGnbs >= 2)
    {
        TapBridgeHelper tapBridge;
        tapBridge.SetAttribute("Mode", StringValue("UseBridge"));

        // Install tap bridge on edge server 0
        tapBridge.SetAttribute("DeviceName", StringValue(tapEdge0Device));
        tapBridge.Install(edgeServerNodes.Get(0), edgeServerDevices[0]);
        NS_LOG_UNCOND("Tap bridge installed on Edge Server 0");
        NS_LOG_UNCOND("  Edge Server 0 IP: " << g_cellIdToEdgeServerIp.begin()->second);
        NS_LOG_UNCOND("  Tap device: " << tapEdge0Device);

        // Install tap bridge on edge server 1
        tapBridge.SetAttribute("DeviceName", StringValue(tapEdge1Device));
        tapBridge.Install(edgeServerNodes.Get(1), edgeServerDevices[1]);
        NS_LOG_UNCOND("Tap bridge installed on Edge Server 1");
        NS_LOG_UNCOND("  Tap device: " << tapEdge1Device);
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
    // Set up traffic applications (DL direction: Edge Server -> UE)
    // NOTE: Using DL traffic to avoid scheduler crash bug with UL during handover
    //--------------------------------------------------------------------------

    uint16_t dlPort = 5000;

    // Install UDP server (sink) on UE to receive DL traffic
    UdpServerHelper ueServer(dlPort);
    ApplicationContainer ueServerApp = ueServer.Install(ueNodes.Get(0));
    ueServerApp.Start(Seconds(0.5));
    ueServerApp.Stop(Seconds(simTime));

    // Install UDP client on each edge server - they will send DL traffic to UE
    // In a real MEC scenario, the active edge server would be determined by the serving gNB
    ApplicationContainer edgeClientApps;
    for (uint16_t i = 0; i < numGnbs; ++i)
    {
        UdpClientHelper edgeClient(ueIpAddr, dlPort);
        edgeClient.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
        edgeClient.SetAttribute("Interval", TimeValue(Seconds(packetSize * 8.0 / (dataRate * 1e6))));
        edgeClient.SetAttribute("PacketSize", UintegerValue(packetSize));
        edgeClientApps.Add(edgeClient.Install(edgeServerNodes.Get(i)));
    }
    // Start traffic after network stabilization
    edgeClientApps.Start(Seconds(2.0));
    edgeClientApps.Stop(Seconds(simTime - 1.0));

    //--------------------------------------------------------------------------
    // Schedule UE position printing
    //--------------------------------------------------------------------------
    Simulator::Schedule(Seconds(1.0), &PrintUePosition, ueNodes.Get(0));

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

    // Print received packets at UE (DL traffic)
    Ptr<UdpServer> ueUdpServer = ueServerApp.Get(0)->GetObject<UdpServer>();
    NS_LOG_UNCOND("UE received " << ueUdpServer->GetReceived() << " packets from edge servers");

    Simulator::Destroy();
    return 0;
}
