// Copyright (c) 2024
// SPDX-License-Identifier: GPL-2.0-only

#include "edge-tunnel-app.h"
#include "zenoh-latency-measurement.h"

#include "ns3/arp-cache.h"
#include "ns3/inet-socket-address.h"
#include "ns3/ipv4-interface.h"
#include "ns3/ipv4-l3-protocol.h"
#include "ns3/ipv4-header.h"
#include "ns3/ipv4.h"
#include "ns3/log.h"
#include "ns3/mac48-address.h"
#include "ns3/simulator.h"
#include "ns3/socket-factory.h"
#include "ns3/udp-header.h"
#include "ns3/udp-socket-factory.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("EdgeTunnelApp");
NS_OBJECT_ENSURE_REGISTERED(EdgeTunnelApp);

TypeId
EdgeTunnelApp::GetTypeId()
{
    static TypeId tid = TypeId("ns3::EdgeTunnelApp")
                            .SetParent<Application>()
                            .SetGroupName("Applications")
                            .AddConstructor<EdgeTunnelApp>();
    return tid;
}

EdgeTunnelApp::EdgeTunnelApp()
    : m_tunnelSocket(nullptr),
      m_outerDevice(nullptr),
      m_innerDevice(nullptr),
      m_localPort(5000),
      m_tunnelPort(5000),
      m_running(false),
      m_txPackets(0),
      m_rxPackets(0),
      m_innerSubnet(Ipv4Address("0.0.0.0")),
      m_innerMask(Ipv4Mask("0.0.0.0")),
      m_innerSubnetSet(false),
      m_edgeNodeId(0)
{
    NS_LOG_FUNCTION(this);
}

EdgeTunnelApp::~EdgeTunnelApp()
{
    NS_LOG_FUNCTION(this);
}

void
EdgeTunnelApp::DoDispose()
{
    NS_LOG_FUNCTION(this);
    m_tunnelSocket = nullptr;
    m_outerDevice = nullptr;
    m_innerDevice = nullptr;
    m_tunnelMappings.clear();
    Application::DoDispose();
}

void
EdgeTunnelApp::SetOuterDevice(Ptr<NetDevice> device)
{
    NS_LOG_FUNCTION(this << device);
    m_outerDevice = device;
}

void
EdgeTunnelApp::SetInnerDevice(Ptr<NetDevice> device)
{
    NS_LOG_FUNCTION(this << device);
    m_innerDevice = device;
}

void
EdgeTunnelApp::AddTunnelMapping(Ipv4Address clientSubnet, Ipv4Mask clientMask, Ipv4Address ueNrIp)
{
    NS_LOG_FUNCTION(this << clientSubnet << clientMask << ueNrIp);
    TunnelEntry entry;
    entry.subnet = clientSubnet;
    entry.mask = clientMask;
    entry.ueNrIp = ueNrIp;
    m_tunnelMappings.push_back(entry);
}

void
EdgeTunnelApp::SetLocalPort(uint16_t port)
{
    NS_LOG_FUNCTION(this << port);
    m_localPort = port;
}

void
EdgeTunnelApp::SetTunnelPort(uint16_t port)
{
    NS_LOG_FUNCTION(this << port);
    m_tunnelPort = port;
}

void
EdgeTunnelApp::SetInnerSubnet(Ipv4Address subnet, Ipv4Mask mask)
{
    NS_LOG_FUNCTION(this << subnet << mask);
    m_innerSubnet = subnet;
    m_innerMask = mask;
    m_innerSubnetSet = true;
}

void
EdgeTunnelApp::SetEdgeNodeId(uint16_t nodeId)
{
    NS_LOG_FUNCTION(this << nodeId);
    m_edgeNodeId = nodeId;
}

void
EdgeTunnelApp::StartApplication()
{
    NS_LOG_FUNCTION(this);
    m_running = true;

    // Create UDP socket for tunnel communication (for sending)
    if (!m_tunnelSocket)
    {
        TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
        m_tunnelSocket = Socket::CreateSocket(GetNode(), tid);

        // Bind to local port
        InetSocketAddress local = InetSocketAddress(Ipv4Address::GetAny(), m_localPort);
        if (m_tunnelSocket->Bind(local) == -1)
        {
            NS_LOG_ERROR("Failed to bind tunnel socket to port " << m_localPort);
            return;
        }
        // Note: We use promiscuous callback on outer device to capture tunnel packets
        // Socket callback is not used because we need L2 interception
    }

    NS_LOG_UNCOND(Simulator::Now().GetSeconds() << "s [EDGE_TUNNEL] Started");

    // Register promiscuous callback on OUTER device (PGW side)
    // This captures incoming tunnel packets from UE
    if (m_outerDevice)
    {
        m_outerDevice->SetPromiscReceiveCallback(
            MakeCallback(&EdgeTunnelApp::ReceiveFromOuter, this));
        NS_LOG_UNCOND("  Outer device (PGW side): " << m_outerDevice->GetAddress());
    }
    else
    {
        NS_LOG_ERROR("No outer device set for EdgeTunnelApp");
    }

    // Register promiscuous callback on INNER device (Ghost/tap side)
    // This captures packets from Docker destined for client subnet
    if (m_innerDevice)
    {
        m_innerDevice->SetPromiscReceiveCallback(
            MakeCallback(&EdgeTunnelApp::ReceiveFromInner, this));
        NS_LOG_UNCOND("  Inner device (Ghost/tap side): " << m_innerDevice->GetAddress());
    }
    else
    {
        NS_LOG_ERROR("No inner device set for EdgeTunnelApp");
    }

    NS_LOG_UNCOND("  Local port: " << m_localPort);
    NS_LOG_UNCOND("  Tunnel mappings:");
    for (const auto& entry : m_tunnelMappings)
    {
        NS_LOG_UNCOND("    " << entry.subnet << "/" << entry.mask << " -> " << entry.ueNrIp);
    }
}

void
EdgeTunnelApp::StopApplication()
{
    NS_LOG_FUNCTION(this);
    m_running = false;

    NS_LOG_UNCOND("[EDGE_TUNNEL] Stopped. TX=" << m_txPackets << " RX=" << m_rxPackets);

    if (m_tunnelSocket)
    {
        m_tunnelSocket->Close();
        m_tunnelSocket->SetRecvCallback(MakeNullCallback<void, Ptr<Socket>>());
    }
}

Ipv4Address
EdgeTunnelApp::LookupTunnelEndpoint(Ipv4Address dstAddr)
{
    for (const auto& entry : m_tunnelMappings)
    {
        uint32_t dstVal = dstAddr.Get();
        uint32_t subnetVal = entry.subnet.Get();
        uint32_t maskVal = entry.mask.Get();

        if ((dstVal & maskVal) == (subnetVal & maskVal))
        {
            return entry.ueNrIp;
        }
    }
    return Ipv4Address("0.0.0.0");
}

bool
EdgeTunnelApp::ReceiveFromOuter(Ptr<NetDevice> device,
                                 Ptr<const Packet> packet,
                                 uint16_t protocol,
                                 const Address& source,
                                 const Address& destination,
                                 NetDevice::PacketType packetType)
{
    // This callback handles packets arriving from PGW (tunnel packets from UE)
    NS_LOG_FUNCTION(this << packet->GetSize() << protocol);

    if (!m_running)
    {
        return false;
    }

    // Only handle IP packets
    if (protocol != 0x0800)
    {
        return false;
    }

    // Extract IP header
    Ptr<Packet> pktCopy = packet->Copy();
    Ipv4Header ipHeader;
    pktCopy->RemoveHeader(ipHeader);

    Ipv4Address srcAddr = ipHeader.GetSource();
    Ipv4Address dstAddr = ipHeader.GetDestination();

    // Record INGRESS for ALL Zenoh packets arriving from WAN/PGW side
    // This captures packets at intermediate edges (e.g., Edge 5) before they go to Docker
    auto seq = ExtractZenohPayloadSeq(pktCopy, false);
    if (seq)
    {
        ZenohLatencyTracker::GetInstance().RecordHopIngress(*seq, m_edgeNodeId);
        NS_LOG_UNCOND("[EDGE_TUNNEL] Zenoh seq=" << *seq << " ingress at Edge " << m_edgeNodeId
                      << " (from " << srcAddr << ")");
    }

    // Check if this is a tunnel packet (UDP to our IP on tunnel port)
    Ptr<Ipv4> ipv4 = GetNode()->GetObject<Ipv4>();
    bool isLocalDst = false;
    for (uint32_t i = 0; i < ipv4->GetNInterfaces(); ++i)
    {
        for (uint32_t j = 0; j < ipv4->GetNAddresses(i); ++j)
        {
            if (ipv4->GetAddress(i, j).GetLocal() == dstAddr)
            {
                isLocalDst = true;
                break;
            }
        }
        if (isLocalDst) break;
    }

    if (isLocalDst && ipHeader.GetProtocol() == 17)  // UDP
    {
        // Check UDP port
        UdpHeader udpHeader;
        pktCopy->RemoveHeader(udpHeader);

        if (udpHeader.GetDestinationPort() == m_localPort)
        {
            // This is an incoming tunnel packet from UE!
            NS_LOG_UNCOND(Simulator::Now().GetSeconds() << "s [EDGE_TUNNEL] Received from tunnel: "
                          << pktCopy->GetSize() << " bytes from " << srcAddr);

            m_rxPackets++;

            // Schedule ForwardToInner to run after this callback completes
            // This avoids issues with CSMA channel state during promiscuous callback
            Simulator::ScheduleNow(&EdgeTunnelApp::ForwardToInner, this, pktCopy);

            return true;  // We handled this packet
        }
    }

    // Not a tunnel packet, let it pass through
    return false;
}

bool
EdgeTunnelApp::ReceiveFromInner(Ptr<NetDevice> device,
                                 Ptr<const Packet> packet,
                                 uint16_t protocol,
                                 const Address& source,
                                 const Address& destination,
                                 NetDevice::PacketType packetType)
{
    // This callback handles packets from Docker (Ghost/tap side) going to client subnet
    NS_LOG_FUNCTION(this << packet->GetSize() << protocol);

    if (!m_running)
    {
        return false;
    }

    // Only handle IP packets
    if (protocol != 0x0800)
    {
        return false;
    }

    // Extract IP header
    Ptr<Packet> pktCopy = packet->Copy();
    Ipv4Header ipHeader;
    pktCopy->PeekHeader(ipHeader);

    Ipv4Address srcAddr = ipHeader.GetSource();
    Ipv4Address dstAddr = ipHeader.GetDestination();

    // Learn the source MAC address for later use in ForwardToInner
    // This learns Docker container MACs for when we forward tunnel responses
    if (Mac48Address::IsMatchingType(source))
    {
        Mac48Address srcMac = Mac48Address::ConvertFrom(source);
        if (m_learnedMacs.find(srcAddr.Get()) == m_learnedMacs.end())
        {
            m_learnedMacs[srcAddr.Get()] = srcMac;
            NS_LOG_UNCOND(Simulator::Now().GetSeconds() << "s [EDGE_TUNNEL] Learned MAC: "
                          << srcAddr << " -> " << srcMac);
        }
    }

    // Look up if this destination should be tunneled to a UE (7.0.1.x subnet)
    Ipv4Address ueAddr = LookupTunnelEndpoint(dstAddr);
    if (ueAddr == Ipv4Address("0.0.0.0"))
    {
        // No tunnel mapping for this destination
        return false;
    }

    // Don't tunnel packets from local addresses (prevent loops)
    Ptr<Ipv4> ipv4 = GetNode()->GetObject<Ipv4>();
    for (uint32_t i = 0; i < ipv4->GetNInterfaces(); ++i)
    {
        for (uint32_t j = 0; j < ipv4->GetNAddresses(i); ++j)
        {
            if (ipv4->GetAddress(i, j).GetLocal() == srcAddr)
            {
                return false;  // Don't tunnel our own packets
            }
        }
    }

    NS_LOG_UNCOND(Simulator::Now().GetSeconds() << "s [EDGE_TUNNEL] Downstream: "
                  << srcAddr << " -> " << dstAddr << " via UE " << ueAddr
                  << " (size=" << packet->GetSize() << ")");

    // Record edge egress - packet leaving edge to UE via 5G tunnel
    // This is the EDGE_EGRESS point for this edge server
    auto seq = ExtractZenohPayloadSeq(packet, false);
    if (seq)
    {
        ZenohLatencyTracker::GetInstance().RecordHopEgress(*seq, m_edgeNodeId);
        NS_LOG_UNCOND("[EDGE_TUNNEL] Zenoh seq=" << *seq << " egress from Edge " << m_edgeNodeId);
    }

    // Tunnel this packet to the UE
    SendToTunnel(packet, ueAddr);

    return true;
}

void
EdgeTunnelApp::SendToTunnel(Ptr<const Packet> innerPacket, Ipv4Address ueAddr)
{
    NS_LOG_FUNCTION(this << innerPacket->GetSize() << ueAddr);

    if (!m_tunnelSocket)
    {
        NS_LOG_ERROR("No tunnel socket");
        return;
    }

    // Create a copy of the inner packet as the tunnel payload
    Ptr<Packet> tunnelPacket = innerPacket->Copy();

    // Send to UE via tunnel (goes through outer device to PGW)
    InetSocketAddress remote = InetSocketAddress(ueAddr, m_tunnelPort);
    int ret = m_tunnelSocket->SendTo(tunnelPacket, 0, remote);

    if (ret > 0)
    {
        m_txPackets++;
        NS_LOG_UNCOND(Simulator::Now().GetSeconds() << "s [EDGE_TUNNEL] Sent to tunnel: "
                      << tunnelPacket->GetSize() << " bytes -> " << ueAddr);
    }
    else
    {
        NS_LOG_UNCOND("[EDGE_TUNNEL] ERROR: Failed to send to tunnel, ret=" << ret);
    }
}

void
EdgeTunnelApp::ReceiveFromTunnel(Ptr<Socket> socket)
{
    // This method is kept for potential future use but is not currently used
    // We use promiscuous callback on outer device instead
    NS_LOG_FUNCTION(this << socket);

    Ptr<Packet> packet;
    Address from;

    while ((packet = socket->RecvFrom(from)))
    {
        if (!m_running)
        {
            return;
        }

        if (InetSocketAddress::IsMatchingType(from))
        {
            InetSocketAddress address = InetSocketAddress::ConvertFrom(from);
            NS_LOG_UNCOND(Simulator::Now().GetSeconds() << "s [EDGE_TUNNEL] Received from tunnel: "
                          << packet->GetSize() << " bytes from " << address.GetIpv4());

            m_rxPackets++;

            // The packet payload is the original IP packet - forward to inner device
            ForwardToInner(packet);
        }
    }
}

void
EdgeTunnelApp::ForwardToInner(Ptr<Packet> packet)
{
    NS_LOG_FUNCTION(this << packet->GetSize());

    if (!m_innerDevice)
    {
        NS_LOG_ERROR("No inner device");
        return;
    }

    // Extract destination from IP header for logging
    Ipv4Header ipHeader;
    uint32_t headerSize = packet->PeekHeader(ipHeader);
    Ipv4Address srcAddr = ipHeader.GetSource();
    Ipv4Address dstAddr = ipHeader.GetDestination();

    // Check if destination is on our inner subnet
    // If not, drop the packet (it would be broadcast and routed via PGW otherwise)
    if (m_innerSubnetSet)
    {
        uint32_t dstVal = dstAddr.Get();
        uint32_t subnetVal = m_innerSubnet.Get();
        uint32_t maskVal = m_innerMask.Get();

        if ((dstVal & maskVal) != (subnetVal & maskVal))
        {
            NS_LOG_UNCOND(Simulator::Now().GetSeconds() << "s [EDGE_TUNNEL] DROP: "
                          << dstAddr << " not on inner subnet " << m_innerSubnet << "/"
                          << m_innerMask << " - connection should break after handover");
            return;
        }
    }

    // Try to resolve destination MAC - first check learned MACs, then ARP cache
    Mac48Address dstMac = Mac48Address::GetBroadcast();  // Default to broadcast

    // First check learned MAC table (for Docker containers behind TapBridge)
    auto it = m_learnedMacs.find(dstAddr.Get());
    if (it != m_learnedMacs.end())
    {
        dstMac = it->second;
        NS_LOG_UNCOND(Simulator::Now().GetSeconds() << "s [EDGE_TUNNEL] Using learned MAC: "
                      << dstAddr << " -> " << dstMac);
    }
    else
    {
        // Fall back to ARP cache lookup
        Ptr<Ipv4L3Protocol> ipv4l3 = GetNode()->GetObject<Ipv4L3Protocol>();
        if (ipv4l3)
        {
            int32_t ifIndex = ipv4l3->GetInterfaceForDevice(m_innerDevice);
            if (ifIndex >= 0)
            {
                Ptr<Ipv4Interface> iface = ipv4l3->GetInterface(ifIndex);
                if (iface)
                {
                    Ptr<ArpCache> arpCache = iface->GetArpCache();
                    if (arpCache)
                    {
                        ArpCache::Entry* entry = arpCache->Lookup(dstAddr);
                        if (entry && entry->IsAlive())
                        {
                            dstMac = Mac48Address::ConvertFrom(entry->GetMacAddress());
                            NS_LOG_UNCOND(Simulator::Now().GetSeconds() << "s [EDGE_TUNNEL] ARP resolved: "
                                          << dstAddr << " -> " << dstMac);
                        }
                    }
                }
            }
        }
    }

    NS_LOG_UNCOND(Simulator::Now().GetSeconds() << "s [EDGE_TUNNEL] Forward to inner: "
                  << srcAddr << " -> " << dstAddr << " (size=" << packet->GetSize() << ")"
                  << " IP hdr=" << headerSize << " bytes, proto=" << (int)ipHeader.GetProtocol()
                  << " dstMac=" << dstMac << " via device " << m_innerDevice->GetAddress());

    // Send to inner device (CSMA -> GhostNode -> tap bridge -> Docker)
    bool success = m_innerDevice->Send(packet, dstMac, 0x0800);  // IP protocol

    if (success)
    {
        NS_LOG_UNCOND(Simulator::Now().GetSeconds() << "s [EDGE_TUNNEL] Send success on inner device");
    }
    else
    {
        NS_LOG_UNCOND(Simulator::Now().GetSeconds() << "s [EDGE_TUNNEL] ERROR: Send failed on inner device!");
    }
}

} // namespace ns3
