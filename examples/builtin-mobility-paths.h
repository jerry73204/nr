/*
 * Copyright (c) 2024
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#ifndef BUILTIN_MOBILITY_PATHS_H
#define BUILTIN_MOBILITY_PATHS_H

#include "ns3/log.h"
#include "ns3/mobility-module.h"

#include <cmath>
#include <string>
#include <vector>

namespace ns3
{

/**
 * Apply a built-in waypoint path to a WaypointMobilityModel.
 *
 * Available paths: hexagonal, linear-y, zigzag, highway, urban-grid, kaohsiung.
 * Falls back to linear-y if an unknown path name is given.
 *
 * @param builtinPath  Name of the built-in path to apply
 * @param waypointMm   The WaypointMobilityModel to add waypoints to
 * @param ueSpeed      UE speed in m/s (used by linear-y, kaohsiung, and fallback)
 */
inline void
ApplyBuiltinPath(const std::string& builtinPath,
                 Ptr<WaypointMobilityModel> waypointMm,
                 double ueSpeed)
{
    if (builtinPath == "hexagonal")
    {
        // Circular path visiting all 7 sites (triggers handovers to each site)
        // Site positions with ISD=500m:
        //   Site 0: (0, 0), Site 1: (433, 250), Site 2: (0, 500)
        //   Site 3: (-433, 250), Site 4: (-433, -250), Site 5: (0, -500)
        //   Site 6: (433, -250)
        // Path goes directly to each site center to ensure handover
        double hexSpeed = 12.0; // m/s for this path
        double t = 0.0;

        // Start at center (Site 0)
        waypointMm->AddWaypoint(Waypoint(Seconds(t), Vector(0, 0, 1.5)));

        // Go directly to Site 1 center (433, 250)
        t += 500.0 / hexSpeed;
        waypointMm->AddWaypoint(Waypoint(Seconds(t), Vector(433, 250, 1.5)));

        // Go directly to Site 2 center (0, 500)
        t += 500.0 / hexSpeed;
        waypointMm->AddWaypoint(Waypoint(Seconds(t), Vector(0, 500, 1.5)));

        // Go directly to Site 3 center (-433, 250)
        t += 500.0 / hexSpeed;
        waypointMm->AddWaypoint(Waypoint(Seconds(t), Vector(-433, 250, 1.5)));

        // Go directly to Site 4 center (-433, -250)
        t += 500.0 / hexSpeed;
        waypointMm->AddWaypoint(Waypoint(Seconds(t), Vector(-433, -250, 1.5)));

        // Go directly to Site 5 center (0, -500)
        t += 500.0 / hexSpeed;
        waypointMm->AddWaypoint(Waypoint(Seconds(t), Vector(0, -500, 1.5)));

        // Go directly to Site 6 center (433, -250)
        t += 500.0 / hexSpeed;
        waypointMm->AddWaypoint(Waypoint(Seconds(t), Vector(433, -250, 1.5)));

        // Return to center (Site 0)
        t += 500.0 / hexSpeed;
        waypointMm->AddWaypoint(Waypoint(Seconds(t), Vector(0, 0, 1.5)));

        NS_LOG_INFO("Waypoint mobility: hexagonal path visiting all 7 sites (total time=" << t
                                                                                         << "s)");
    }
    else if (builtinPath == "linear-y")
    {
        double totalDist = 450.0;
        double travelTime = totalDist / ueSpeed;
        waypointMm->AddWaypoint(Waypoint(Seconds(0), Vector(0, 50, 1.5)));
        waypointMm->AddWaypoint(Waypoint(Seconds(travelTime), Vector(0, 500, 1.5)));
        NS_LOG_INFO("Waypoint mobility: linear-y path toward Site 2 (time=" << travelTime << "s)");
    }
    else if (builtinPath == "zigzag")
    {
        // Zigzag path that crosses multiple site boundaries
        double zigzagSpeed = 15.0; // m/s
        double t = 0.0;

        // Start at center (Site 0)
        waypointMm->AddWaypoint(Waypoint(Seconds(t), Vector(0, 0, 1.5)));

        // Zig to Site 1 area (433, 250)
        t += 500.0 / zigzagSpeed;
        waypointMm->AddWaypoint(Waypoint(Seconds(t), Vector(400, 220, 1.5)));

        // Zag to Site 3 area (-433, 250)
        t += 850.0 / zigzagSpeed;
        waypointMm->AddWaypoint(Waypoint(Seconds(t), Vector(-400, 280, 1.5)));

        // Zig to Site 6 area (433, -250)
        t += 950.0 / zigzagSpeed;
        waypointMm->AddWaypoint(Waypoint(Seconds(t), Vector(400, -220, 1.5)));

        // Zag to Site 4 area (-433, -250)
        t += 800.0 / zigzagSpeed;
        waypointMm->AddWaypoint(Waypoint(Seconds(t), Vector(-400, -280, 1.5)));

        // Return to center (Site 0)
        t += 500.0 / zigzagSpeed;
        waypointMm->AddWaypoint(Waypoint(Seconds(t), Vector(0, 0, 1.5)));

        NS_LOG_INFO("Waypoint mobility: zigzag path crossing sites (total time=" << t << "s)");
    }
    else if (builtinPath == "highway")
    {
        // Highway path: Site 5 -> Site 0 -> Site 2 (along Y-axis)
        double highwaySpeed = 20.0; // 72 km/h highway speed

        Vector start(0, -400, 1.5); // 100m from Site 5
        Vector end(0, 400, 1.5);    // 100m from Site 2
        double distance = (end - start).GetLength();
        double travelTime = distance / highwaySpeed;

        waypointMm->AddWaypoint(Waypoint(Seconds(0), start));
        waypointMm->AddWaypoint(Waypoint(Seconds(travelTime), end));
        NS_LOG_INFO("Waypoint mobility: highway path Site5->Site0->Site2 (speed="
                    << highwaySpeed << " m/s, distance=" << distance
                    << "m, travel time=" << travelTime << "s)");
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

        NS_LOG_INFO("Waypoint mobility: urban-grid path with "
                    << stopTime << "s stops at intersections (total time=" << t << "s)");
    }
    else if (builtinPath == "kaohsiung")
    {
        // Real-world path from Kaohsiung, Taiwan GPX data
        // Converted from lat/lon to local X/Y coordinates (meters)
        // Reference point: lat=22.615131, lon=120.303932
        // Total path distance: ~482m, spans ~200m x 284m
        struct WaypointData
        {
            double x;
            double y;
        };

        std::vector<WaypointData> gpxPoints = {
            {0.00, 0.00},        {16.13, -49.09},     {22.20, -67.46},
            {26.31, -80.04},     {25.90, -80.48},     {23.53, -82.49},
            {20.24, -85.16},     {16.54, -89.17},     {13.98, -92.06},
            {12.13, -94.84},     {7.81, -101.97},     {5.86, -104.97},
            {2.67, -112.21},     {0.82, -116.55},     {0.00, -124.34},
            {-1.23, -136.70},    {-1.23, -140.37},    {-1.23, -146.05},
            {1.54, -155.51},     {32.37, -145.16},    {37.20, -143.38},
            {46.14, -140.04},    {67.62, -132.47},    {103.48, -153.18},
            {107.49, -154.73},   {143.35, -172.32},   {173.97, -187.13},
            {179.93, -190.13},   {180.55, -190.36},   {175.21, -201.82},
            {156.61, -241.68},   {156.71, -241.90},   {161.54, -246.69},
            {174.28, -258.37},   {198.43, -283.75},
        };

        double t = 0.0;
        double ueHeight = 1.5;
        waypointMm->AddWaypoint(
            Waypoint(Seconds(t), Vector(gpxPoints[0].x, gpxPoints[0].y, ueHeight)));

        for (size_t i = 1; i < gpxPoints.size(); ++i)
        {
            double dx = gpxPoints[i].x - gpxPoints[i - 1].x;
            double dy = gpxPoints[i].y - gpxPoints[i - 1].y;
            double dist = std::sqrt(dx * dx + dy * dy);
            t += dist / ueSpeed;
            waypointMm->AddWaypoint(
                Waypoint(Seconds(t), Vector(gpxPoints[i].x, gpxPoints[i].y, ueHeight)));
        }

        NS_LOG_INFO("Waypoint mobility: kaohsiung real-world path (speed=" << ueSpeed
                                                                           << " m/s, total time="
                                                                           << t << "s)");
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

} // namespace ns3

#endif // BUILTIN_MOBILITY_PATHS_H
