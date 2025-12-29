// Copyright (c) 2024
// SPDX-License-Identifier: GPL-2.0-only

#include "ue-tunnel-app.h"
#include "zenoh-latency-measurement.h"

#include "ns3/arp-cache.h"
#include "ns3/inet-socket-address.h"
#include "ns3/ipv4-header.h"
#include "ns3/ipv4-interface.h"
#include "ns3/ipv4-l3-protocol.h"
#include "ns3/ipv4.h"
#include "ns3/log.h"
#include "ns3/mac48-address.h"
#include "ns3/simulator.h"
#include "ns3/socket-factory.h"
#include "ns3/udp-socket-factory.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("UeTunnelApp");
NS_OBJECT_ENSURE_REGISTERED(UeTunnelApp);

TypeId
UeTunnelApp::GetTypeId()
{
    static TypeId tid = TypeId("ns3::UeTunnelApp")
                            .SetParent<Application>()
                            .SetGroupName("Applications")
                            .AddConstructor<UeTunnelApp>();
    return tid;
}

UeTunnelApp::UeTunnelApp()
    : m_tunnelSocket(nullptr),
      m_innerDevice(nullptr),
      m_edgeServerIp(Ipv4Address("10.1.0.2")),
      m_tunnelPort(5000),
      m_localPort(5000),
      m_clientSubnet(Ipv4Address("7.0.1.0")),
      m_clientMask(Ipv4Mask("255.255.255.0")),
      m_running(false),
      m_txPackets(0),
      m_rxPackets(0),
      m_handoverActive(false)
{
    NS_LOG_FUNCTION(this);
}

UeTunnelApp::~UeTunnelApp()
{
    NS_LOG_FUNCTION(this);
}

void
UeTunnelApp::DoDispose()
{
    NS_LOG_FUNCTION(this);
    m_tunnelSocket = nullptr;
    m_innerDevice = nullptr;
    Application::DoDispose();
}

void
UeTunnelApp::SetInnerDevice(Ptr<NetDevice> device)
{
    NS_LOG_FUNCTION(this << device);
    m_innerDevice = device;
}

void
UeTunnelApp::SetTunnelEndpoint(Ipv4Address edgeServerIp, uint16_t port)
{
    NS_LOG_FUNCTION(this << edgeServerIp << port);
    m_edgeServerIp = edgeServerIp;
    m_tunnelPort = port;
}

void
UeTunnelApp::SetLocalPort(uint16_t port)
{
    NS_LOG_FUNCTION(this << port);
    m_localPort = port;
}

void
UeTunnelApp::SetClientSubnet(Ipv4Address subnet, Ipv4Mask mask)
{
    NS_LOG_FUNCTION(this << subnet << mask);
    m_clientSubnet = subnet;
    m_clientMask = mask;
}

void
UeTunnelApp::UpdateTunnelEndpoint(Ipv4Address newEdgeServerIp)
{
    NS_LOG_FUNCTION(this << newEdgeServerIp);

    if (m_edgeServerIp != newEdgeServerIp)
    {
        NS_LOG_DEBUG(Simulator::Now().GetSeconds()
                      << "s [UE_TUNNEL] Switching tunnel endpoint: "
                      << m_edgeServerIp << " -> " << newEdgeServerIp);
        m_edgeServerIp = newEdgeServerIp;
    }
}

Ipv4Address
UeTunnelApp::GetTunnelEndpoint() const
{
    return m_edgeServerIp;
}

void
UeTunnelApp::SetHandoverActive(bool active)
{
    NS_LOG_FUNCTION(this << active);
    m_handoverActive = active;
}

void
UeTunnelApp::StartApplication()
{
    NS_LOG_FUNCTION(this);
    m_running = true;

    // Create UDP socket for tunnel communication
    if (!m_tunnelSocket)
    {
        TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
        m_tunnelSocket = Socket::CreateSocket(GetNode(), tid);

        // Bind to local port on NR interface specifically
        // Get UE's NR interface IP (should be 7.0.0.x)
        Ptr<Ipv4> ipv4 = GetNode()->GetObject<Ipv4>();
        Ipv4Address localIp = Ipv4Address::GetAny();

        // Find the NR interface (not loopback, not CSMA/inner)
        for (uint32_t i = 1; i < ipv4->GetNInterfaces(); ++i)
        {
            Ipv4Address addr = ipv4->GetAddress(i, 0).GetLocal();
            // Skip if it's in the client subnet (inner CSMA interface)
            uint32_t addrVal = addr.Get();
            uint32_t subnetVal = m_clientSubnet.Get();
            uint32_t maskVal = m_clientMask.Get();
            if ((addrVal & maskVal) != (subnetVal & maskVal))
            {
                localIp = addr;  // This should be the NR interface (7.0.0.x)
                break;
            }
        }

        InetSocketAddress local = InetSocketAddress(localIp, m_localPort);
        if (m_tunnelSocket->Bind(local) == -1)
        {
            NS_LOG_ERROR("Failed to bind tunnel socket to " << localIp << ":" << m_localPort);
            return;
        }

        // Set receive callback for incoming tunnel packets
        m_tunnelSocket->SetRecvCallback(MakeCallback(&UeTunnelApp::ReceiveFromTunnel, this));
    }

    if (m_innerDevice)
    {
        // Use promiscuous callback to capture packets at L2 level
        // This happens BEFORE IP layer processing
        m_innerDevice->SetPromiscReceiveCallback(
            MakeCallback(&UeTunnelApp::ReceiveFromInner, this));

        NS_LOG_DEBUG(Simulator::Now().GetSeconds() << "s [UE_TUNNEL] Started");
        NS_LOG_DEBUG("  Inner device: " << m_innerDevice->GetAddress());
        NS_LOG_DEBUG("  Tunnel endpoint: " << m_edgeServerIp << ":" << m_tunnelPort);
        NS_LOG_DEBUG("  Local port: " << m_localPort);
        NS_LOG_DEBUG("  Client subnet: " << m_clientSubnet << "/" << m_clientMask);
    }
    else
    {
        NS_LOG_ERROR("No inner device set for UeTunnelApp");
    }
}

void
UeTunnelApp::StopApplication()
{
    NS_LOG_FUNCTION(this);
    m_running = false;

    NS_LOG_DEBUG("[UE_TUNNEL] Stopped. TX=" << m_txPackets << " RX=" << m_rxPackets);

    if (m_tunnelSocket)
    {
        m_tunnelSocket->Close();
        m_tunnelSocket->SetRecvCallback(MakeNullCallback<void, Ptr<Socket>>());
    }
}

bool
UeTunnelApp::ShouldTunnel(Ipv4Address srcAddr, Ipv4Address dstAddr)
{
    // Tunnel if source is from client subnet (upstream)
    uint32_t srcVal = srcAddr.Get();
    uint32_t subnetVal = m_clientSubnet.Get();
    uint32_t maskVal = m_clientMask.Get();

    return (srcVal & maskVal) == (subnetVal & maskVal);
}

void
UeTunnelApp::InterceptForward(const Ipv4Header& header, Ptr<const Packet> packet, uint32_t interface)
{
    // Not used in this approach
}

bool
UeTunnelApp::ReceiveFromInner(Ptr<NetDevice> device,
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

    // Only handle IP packets
    if (protocol != 0x0800)
    {
        return false;
    }

    // Extract IP header to check source/destination
    Ptr<Packet> pktCopy = packet->Copy();
    Ipv4Header ipHeader;
    pktCopy->PeekHeader(ipHeader);

    Ipv4Address srcAddr = ipHeader.GetSource();
    Ipv4Address dstAddr = ipHeader.GetDestination();

    // Check if this packet should be tunneled (from client subnet)
    if (!ShouldTunnel(srcAddr, dstAddr))
    {
        return false;
    }

    // Don't tunnel packets destined to local addresses (7.0.1.1)
    Ptr<Ipv4> ipv4 = GetNode()->GetObject<Ipv4>();
    for (uint32_t i = 0; i < ipv4->GetNInterfaces(); ++i)
    {
        for (uint32_t j = 0; j < ipv4->GetNAddresses(i); ++j)
        {
            if (ipv4->GetAddress(i, j).GetLocal() == dstAddr)
            {
                return false;  // Local delivery, don't tunnel
            }
        }
    }

    // Learn the source MAC address for later use in ForwardToInner
    // TapBridge packets come from external hosts, so we can't use ARP
    if (Mac48Address::IsMatchingType(source))
    {
        Mac48Address srcMac = Mac48Address::ConvertFrom(source);
        m_learnedMacs[srcAddr.Get()] = srcMac;
        NS_LOG_DEBUG(Simulator::Now().GetSeconds() << "s [UE_TUNNEL] Learned MAC: "
                      << srcAddr << " -> " << srcMac);
    }

    NS_LOG_DEBUG(Simulator::Now().GetSeconds() << "s [UE_TUNNEL] Upstream: "
                  << srcAddr << " -> " << dstAddr << " (size=" << packet->GetSize() << ")");

    // Tunnel this packet - send the complete IP packet
    SendToTunnel(packet);

    // Return true to indicate we've handled it
    // Note: This doesn't prevent IP layer from also processing it
    // We rely on the fact that we've tunneled it, even if IP also forwards it
    // The tunnel copy will arrive properly; the direct forward will be dropped by GTP
    return true;
}

void
UeTunnelApp::SendToTunnel(Ptr<const Packet> innerPacket)
{
    NS_LOG_FUNCTION(this << innerPacket->GetSize());

    if (!m_tunnelSocket)
    {
        NS_LOG_ERROR("No tunnel socket");
        return;
    }

    // Create a copy of the inner packet as the tunnel payload
    Ptr<Packet> tunnelPacket = innerPacket->Copy();

    // Send to Edge Server via tunnel
    InetSocketAddress remote = InetSocketAddress(m_edgeServerIp, m_tunnelPort);
    int ret = m_tunnelSocket->SendTo(tunnelPacket, 0, remote);

    if (ret > 0)
    {
        m_txPackets++;
        NS_LOG_DEBUG(Simulator::Now().GetSeconds() << "s [UE_TUNNEL] Sent to tunnel: "
                      << tunnelPacket->GetSize() << " bytes -> " << m_edgeServerIp);
    }
    else
    {
        NS_LOG_ERROR("Failed to send to tunnel, ret=" << ret);
    }
}

void
UeTunnelApp::ReceiveFromTunnel(Ptr<Socket> socket)
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
            NS_LOG_DEBUG(Simulator::Now().GetSeconds() << "s [UE_TUNNEL] Received from tunnel: "
                          << packet->GetSize() << " bytes from " << address.GetIpv4());

            m_rxPackets++;

            // Extract Zenoh sequence number from payload pattern "[XXXX]" and record latency
            // Enable debug for first 20 packets to diagnose parsing
            static uint32_t debugCount = 0;
            bool enableDebug = (debugCount < 20);
            debugCount++;

            auto seq = ExtractZenohPayloadSeq(packet, enableDebug);
            if (seq)
            {
                double latencyMs = ZenohLatencyTracker::GetInstance().RecordReceive(
                    *seq, m_handoverActive);

                if (latencyMs >= 0)
                {
                    NS_LOG_DEBUG("[UE_TUNNEL] Zenoh seq=" << *seq
                                << " latency=" << latencyMs << "ms");

                    if (latencyMs > 50.0)
                    {
                        NS_LOG_WARN("[SLA VIOLATION] seq=" << *seq
                                    << " latency=" << latencyMs << "ms > 50ms");
                    }
                }
            }

            // The packet payload is the original IP packet - forward to inner device
            ForwardToInner(packet);
        }
    }
}

void
UeTunnelApp::ForwardToInner(Ptr<Packet> packet)
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

    // Try to resolve destination MAC - first check learned MACs, then ARP cache
    Mac48Address dstMac = Mac48Address::GetBroadcast();  // Default to broadcast

    // First check learned MAC table (for TapBridge clients)
    auto it = m_learnedMacs.find(dstAddr.Get());
    if (it != m_learnedMacs.end())
    {
        dstMac = it->second;
        NS_LOG_DEBUG(Simulator::Now().GetSeconds() << "s [UE_TUNNEL] Using learned MAC: "
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
                            NS_LOG_DEBUG(Simulator::Now().GetSeconds() << "s [UE_TUNNEL] ARP resolved: "
                                          << dstAddr << " -> " << dstMac);
                        }
                    }
                }
            }
        }
    }

    NS_LOG_DEBUG(Simulator::Now().GetSeconds() << "s [UE_TUNNEL] Forward to inner: "
                  << srcAddr << " -> " << dstAddr << " (size=" << packet->GetSize() << ")"
                  << " dstMac=" << dstMac);

    // Send to inner device (CSMA -> tap bridge -> external client)
    bool success = m_innerDevice->Send(packet, dstMac, 0x0800);  // IP protocol

    if (!success)
    {
        NS_LOG_ERROR("Failed to forward packet to inner device");
    }
}

} // namespace ns3
