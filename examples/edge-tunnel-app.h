// Copyright (c) 2024
// SPDX-License-Identifier: GPL-2.0-only

/**
 * @file edge-tunnel-app.h
 * @brief UDP Tunnel Application for Edge Server Node
 *
 * This application creates a UDP tunnel on the Edge Server that:
 * - Upstream: Receives UDP tunnel packets from UE on outer device (PGW side),
 *   decapsulates, and forwards to inner device (Ghost/tap side)
 * - Downstream: Captures packets from inner device destined for client subnet,
 *   encapsulates in UDP, and sends through tunnel to UE via outer device
 *
 * Architecture (Ghost Node approach):
 *   PGW <--CSMA--> EdgeServer <--CSMA--> GhostNode <--TapBridge--> Docker
 *                      |
 *                EdgeTunnelApp
 *                - outerDevice: CSMA to PGW (tunnel packets arrive here)
 *                - innerDevice: CSMA to GhostNode (forwards to Docker)
 */

#ifndef EDGE_TUNNEL_APP_H
#define EDGE_TUNNEL_APP_H

#include "ns3/application.h"
#include "ns3/ipv4-address.h"
#include "ns3/net-device.h"
#include "ns3/packet.h"
#include "ns3/socket.h"

#include <map>

namespace ns3
{

/**
 * @brief Edge Server Tunnel Application
 *
 * Tunnels packets between External Router (via Ghost/tap) and UE (via PGW).
 * Uses two separate CSMA devices for clean packet flow.
 */
class EdgeTunnelApp : public Application
{
  public:
    static TypeId GetTypeId();

    EdgeTunnelApp();
    ~EdgeTunnelApp() override;

    /**
     * @brief Set the outer (PGW side) network device
     * @param device The CSMA NetDevice connected to PGW (tunnel packets arrive here)
     */
    void SetOuterDevice(Ptr<NetDevice> device);

    /**
     * @brief Set the inner (Ghost/tap side) network device
     * @param device The CSMA NetDevice connected to GhostNode (forwards to Docker)
     */
    void SetInnerDevice(Ptr<NetDevice> device);

    /**
     * @brief Add a tunnel mapping for a client subnet to UE endpoint
     * @param clientSubnet The client subnet (e.g., 7.0.1.0)
     * @param clientMask The subnet mask (e.g., 255.255.255.0)
     * @param ueNrIp The UE's NR interface IP (e.g., 7.0.0.2)
     */
    void AddTunnelMapping(Ipv4Address clientSubnet, Ipv4Mask clientMask, Ipv4Address ueNrIp);

    /**
     * @brief Set the local tunnel port
     * @param port UDP port to listen on (default 5000)
     */
    void SetLocalPort(uint16_t port);

    /**
     * @brief Set the tunnel port on UE side
     * @param port UDP port to send to (default 5000)
     */
    void SetTunnelPort(uint16_t port);

    /**
     * @brief Set the local inner subnet (for filtering forwarded packets)
     * @param subnet The inner subnet (e.g., 10.1.1.0)
     * @param mask The subnet mask (e.g., 255.255.255.0)
     *
     * Only packets destined for this subnet will be forwarded to inner device.
     * This prevents packets from being broadcast when destination is unreachable.
     */
    void SetInnerSubnet(Ipv4Address subnet, Ipv4Mask mask);

    /**
     * @brief Set the edge node ID for latency measurement
     * @param nodeId The edge node ID (e.g., site ID)
     */
    void SetEdgeNodeId(uint16_t nodeId);

  protected:
    void DoDispose() override;

  private:
    void StartApplication() override;
    void StopApplication() override;

    /**
     * @brief Callback for packets received on outer (PGW) device
     * Captures incoming tunnel packets (UDP to local port), decapsulates,
     * and forwards to inner device
     */
    bool ReceiveFromOuter(Ptr<NetDevice> device,
                          Ptr<const Packet> packet,
                          uint16_t protocol,
                          const Address& source,
                          const Address& destination,
                          NetDevice::PacketType packetType);

    /**
     * @brief Callback for packets received on inner (Ghost/tap) device
     * Captures packets destined for client subnet, encapsulates,
     * and tunnels to UE via outer device
     */
    bool ReceiveFromInner(Ptr<NetDevice> device,
                          Ptr<const Packet> packet,
                          uint16_t protocol,
                          const Address& source,
                          const Address& destination,
                          NetDevice::PacketType packetType);

    /**
     * @brief Callback for UDP packets received from tunnel (from UE)
     * Decapsulates and forwards to inner device
     */
    void ReceiveFromTunnel(Ptr<Socket> socket);

    /**
     * @brief Send encapsulated packet through tunnel to UE
     * @param innerPacket The original IP packet to tunnel
     * @param ueAddr The UE's NR IP address to send to
     */
    void SendToTunnel(Ptr<const Packet> innerPacket, Ipv4Address ueAddr);

    /**
     * @brief Forward decapsulated packet to inner device (toward Ghost/tap)
     * @param packet The decapsulated IP packet
     */
    void ForwardToInner(Ptr<Packet> packet);

    /**
     * @brief Lookup UE tunnel endpoint for a destination IP
     * @param dstAddr The destination IP address
     * @return The UE's NR IP if found, 0.0.0.0 otherwise
     */
    Ipv4Address LookupTunnelEndpoint(Ipv4Address dstAddr);

    // Tunnel mapping: client subnet/mask -> UE NR IP
    struct TunnelEntry
    {
        Ipv4Address subnet;
        Ipv4Mask mask;
        Ipv4Address ueNrIp;
    };
    std::vector<TunnelEntry> m_tunnelMappings;

    // Tunnel UDP socket
    Ptr<Socket> m_tunnelSocket;

    // Outer device (CSMA to PGW - where tunnel packets arrive)
    Ptr<NetDevice> m_outerDevice;

    // Inner device (CSMA to GhostNode - forwards to Docker via TapBridge)
    Ptr<NetDevice> m_innerDevice;

    // Tunnel ports
    uint16_t m_localPort;
    uint16_t m_tunnelPort;

    bool m_running;
    uint64_t m_txPackets;
    uint64_t m_rxPackets;

    // Learned MAC addresses (IP -> MAC) for Docker containers behind tap bridge
    std::map<uint32_t, Mac48Address> m_learnedMacs;

    // Inner subnet (for filtering - only forward packets destined for this subnet)
    Ipv4Address m_innerSubnet;
    Ipv4Mask m_innerMask;
    bool m_innerSubnetSet;

    // Edge node ID for latency measurement
    uint16_t m_edgeNodeId;
};

} // namespace ns3

#endif // EDGE_TUNNEL_APP_H
