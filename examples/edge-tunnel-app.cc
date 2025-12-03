// Copyright (c) 2024
// SPDX-License-Identifier: GPL-2.0-only

#include "edge-tunnel-app.h"

#include "ns3/inet-socket-address.h"
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
      m_innerDevice(nullptr),
      m_localPort(5000),
      m_tunnelPort(5000),
      m_running(false),
      m_txPackets(0),
      m_rxPackets(0)
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
    m_innerDevice = nullptr;
    m_tunnelMappings.clear();
    Application::DoDispose();
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
EdgeTunnelApp::StartApplication()
{
    NS_LOG_FUNCTION(this);
    m_running = true;

    // Create UDP socket for tunnel communication
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

        // Set receive callback for incoming tunnel packets
        m_tunnelSocket->SetRecvCallback(MakeCallback(&EdgeTunnelApp::ReceiveFromTunnel, this));
    }

    // Register promiscuous callback on inner device to capture packets for tunneling
    if (m_innerDevice)
    {
        m_innerDevice->SetPromiscReceiveCallback(
            MakeCallback(&EdgeTunnelApp::ReceiveFromInner, this));

        NS_LOG_UNCOND(Simulator::Now().GetSeconds() << "s [EDGE_TUNNEL] Started");
        NS_LOG_UNCOND("  Inner device: " << m_innerDevice->GetAddress());
        NS_LOG_UNCOND("  Local port: " << m_localPort);
        NS_LOG_UNCOND("  Tunnel mappings:");
        for (const auto& entry : m_tunnelMappings)
        {
            NS_LOG_UNCOND("    " << entry.subnet << "/" << entry.mask << " -> " << entry.ueNrIp);
        }
    }
    else
    {
        NS_LOG_ERROR("No inner device set for EdgeTunnelApp");
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
EdgeTunnelApp::ReceiveFromInner(Ptr<NetDevice> device,
                                 Ptr<const Packet> packet,
                                 uint16_t protocol,
                                 const Address& source,
                                 const Address& destination,
                                 NetDevice::PacketType packetType)
{
    NS_LOG_FUNCTION(this << packet->GetSize() << protocol);

    if (!m_running)
    {
        return false;
    }

    // Log all packets received on inner device for debugging
    NS_LOG_DEBUG(Simulator::Now().GetSeconds() << "s [EDGE_TUNNEL] Inner packet: proto=0x"
                 << std::hex << protocol << std::dec << " size=" << packet->GetSize());

    // Only handle IP packets
    if (protocol != 0x0800)
    {
        return false;
    }

    // Extract IP header to check destination
    Ptr<Packet> pktCopy = packet->Copy();
    Ipv4Header ipHeader;
    pktCopy->PeekHeader(ipHeader);

    Ipv4Address srcAddr = ipHeader.GetSource();
    Ipv4Address dstAddr = ipHeader.GetDestination();

    // Look up if this destination should be tunneled to a UE
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
        if (ipv4->GetAddress(i, 0).GetLocal() == srcAddr)
        {
            return false;  // Don't tunnel our own packets
        }
    }

    NS_LOG_UNCOND(Simulator::Now().GetSeconds() << "s [EDGE_TUNNEL] Downstream: "
                  << srcAddr << " -> " << dstAddr << " via UE " << ueAddr
                  << " (size=" << packet->GetSize() << ")");

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

    // Send to UE via tunnel
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
    packet->PeekHeader(ipHeader);
    Ipv4Address srcAddr = ipHeader.GetSource();
    Ipv4Address dstAddr = ipHeader.GetDestination();

    NS_LOG_UNCOND(Simulator::Now().GetSeconds() << "s [EDGE_TUNNEL] Forward to inner: "
                  << srcAddr << " -> " << dstAddr << " (size=" << packet->GetSize() << ")");

    // Send to inner device (CSMA -> tap bridge -> external router)
    // Use broadcast MAC since we don't have ARP resolution here
    Mac48Address dstMac = Mac48Address::GetBroadcast();

    bool success = m_innerDevice->Send(packet, dstMac, 0x0800);  // IP protocol

    if (!success)
    {
        NS_LOG_ERROR("Failed to forward packet to inner device");
    }
}

} // namespace ns3
