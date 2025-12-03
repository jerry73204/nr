// Copyright (c) 2024
// SPDX-License-Identifier: GPL-2.0-only

/**
 * @file ue-tunnel-app.h
 * @brief UDP Tunnel Application for UE Node
 *
 * This application creates a UDP tunnel on the UE that:
 * - Upstream: Captures packets from CSMA (tap) interface, encapsulates in UDP,
 *   and sends through 5G NR interface to Edge Server
 * - Downstream: Receives UDP tunnel packets, decapsulates, and forwards to
 *   CSMA interface for delivery to external client
 */

#ifndef UE_TUNNEL_APP_H
#define UE_TUNNEL_APP_H

#include "ns3/application.h"
#include "ns3/socket.h"
#include "ns3/ipv4-address.h"
#include "ns3/ipv4-header.h"
#include "ns3/net-device.h"
#include "ns3/packet.h"
#include "ns3/packet-socket.h"
#include "ns3/mac48-address.h"

#include <map>

namespace ns3
{

/**
 * @brief UE Tunnel Application
 *
 * Tunnels packets between external client (via CSMA/tap) and Edge Server (via 5G).
 * - Captures raw IP packets from CSMA interface
 * - Encapsulates in UDP and sends to Edge Server
 * - Receives UDP packets from Edge Server and forwards to CSMA interface
 */
class UeTunnelApp : public Application
{
  public:
    static TypeId GetTypeId();

    UeTunnelApp();
    ~UeTunnelApp() override;

    /**
     * @brief Set the CSMA device (inner interface, connected to tap/client)
     */
    void SetInnerDevice(Ptr<NetDevice> device);

    /**
     * @brief Set the tunnel endpoint (Edge Server address)
     */
    void SetTunnelEndpoint(Ipv4Address edgeServerIp, uint16_t port = 5000);

    /**
     * @brief Set the local tunnel port
     */
    void SetLocalPort(uint16_t port);

    /**
     * @brief Set the client subnet to capture (e.g., 7.0.1.0/24)
     */
    void SetClientSubnet(Ipv4Address subnet, Ipv4Mask mask);

  protected:
    void DoDispose() override;

  private:
    void StartApplication() override;
    void StopApplication() override;

    /**
     * @brief Callback for packets received on inner (CSMA) interface
     * Captures packets from external client, encapsulates, and tunnels to Edge
     */
    bool ReceiveFromInner(Ptr<NetDevice> device,
                          Ptr<const Packet> packet,
                          uint16_t protocol,
                          const Address& source,
                          const Address& destination,
                          NetDevice::PacketType packetType);

    /**
     * @brief Callback for UDP packets received from tunnel (from Edge Server)
     * Decapsulates and forwards to CSMA interface
     */
    void ReceiveFromTunnel(Ptr<Socket> socket);

    /**
     * @brief Send encapsulated packet through tunnel to Edge Server
     */
    void SendToTunnel(Ptr<const Packet> innerPacket);

    /**
     * @brief Forward decapsulated packet to inner (CSMA) interface
     */
    void ForwardToInner(Ptr<Packet> packet);

    /**
     * @brief Check if packet should be tunneled (matches client subnet)
     */
    bool ShouldTunnel(Ipv4Address srcAddr, Ipv4Address dstAddr);

    /**
     * @brief Intercept packets being forwarded by Ipv4L3Protocol
     * This is called via UnicastForward trace before packet is forwarded
     */
    void InterceptForward(const Ipv4Header& header, Ptr<const Packet> packet, uint32_t interface);

    // Tunnel UDP socket
    Ptr<Socket> m_tunnelSocket;

    // Inner device (CSMA, connected to tap/client)
    Ptr<NetDevice> m_innerDevice;

    // Tunnel endpoint (Edge Server)
    Ipv4Address m_edgeServerIp;
    uint16_t m_tunnelPort;
    uint16_t m_localPort;

    // Client subnet to capture
    Ipv4Address m_clientSubnet;
    Ipv4Mask m_clientMask;

    bool m_running;
    uint64_t m_txPackets;
    uint64_t m_rxPackets;

    // Learned MAC addresses (IP -> MAC) for clients behind tap bridge
    std::map<uint32_t, Mac48Address> m_learnedMacs;
};

} // namespace ns3

#endif // UE_TUNNEL_APP_H
