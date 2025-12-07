// Copyright (c) 2024
// SPDX-License-Identifier: GPL-2.0-only

/**
 * @file handover-prediction-file.h
 * @brief Low-overhead file-based handover prediction notification
 *
 * Writes prediction state to a file that external applications (Docker containers)
 * can read. Uses atomic rename to prevent partial reads.
 *
 * Design choices to minimize real-time simulation overhead:
 * 1. Only writes when prediction state CHANGES (not every measurement)
 * 2. Uses atomic rename (write to .tmp, rename to final) - single syscall
 * 3. Batches writes with a minimum interval to avoid excessive I/O
 * 4. Pre-formats strings to minimize work during callback
 */

#ifndef HANDOVER_PREDICTION_FILE_H
#define HANDOVER_PREDICTION_FILE_H

#include "ns3/nstime.h"
#include "ns3/simulator.h"
#include "ns3/ipv4-address.h"

#include <cstdio>
#include <cstring>
#include <string>
#include <unistd.h>

namespace ns3 {

class HandoverPredictionFile
{
public:
    static HandoverPredictionFile& GetInstance()
    {
        static HandoverPredictionFile instance;
        return instance;
    }

    /**
     * @brief Initialize the prediction file system
     * @param basePath Base path for prediction files (e.g., "/tmp/ns3_handover")
     * @param minWriteInterval Minimum time between writes to reduce I/O
     */
    void Initialize(const std::string& basePath = "/tmp/ns3_handover",
                    Time minWriteInterval = MilliSeconds(100))
    {
        m_basePath = basePath;
        m_filePath = basePath + ".json";
        m_tmpPath = basePath + ".tmp";
        m_minWriteInterval = minWriteInterval;
        m_lastWriteTime = Time(0);
        m_lastPredictedTarget = 0;
        m_initialized = true;

        // Write initial empty state
        WriteState(0, 0, Ipv4Address("0.0.0.0"), 0, 0.0);
    }

    /**
     * @brief Update prediction state (only writes if changed or interval elapsed)
     * @param imsi UE identifier
     * @param currentCell Current serving cell
     * @param predictedTarget Predicted handover target cell (0 = no prediction)
     * @param targetEdgeIp Edge server IP for predicted target
     * @param currentRsrp Current serving cell RSRP
     * @param rsrpTrend RSRP trend of predicted target
     * @return true if file was written, false if skipped
     */
    bool UpdatePrediction(uint64_t imsi,
                          uint16_t currentCell,
                          uint16_t predictedTarget,
                          Ipv4Address targetEdgeIp,
                          int currentRsrp,
                          double rsrpTrend)
    {
        if (!m_initialized)
        {
            return false;
        }

        Time now = Simulator::Now();

        // Skip if prediction hasn't changed AND minimum interval hasn't elapsed
        bool predictionChanged = (predictedTarget != m_lastPredictedTarget);
        bool intervalElapsed = (now - m_lastWriteTime) >= m_minWriteInterval;

        if (!predictionChanged && !intervalElapsed)
        {
            return false;
        }

        // Update state and write
        m_lastPredictedTarget = predictedTarget;
        m_lastWriteTime = now;

        WriteState(imsi, currentCell, targetEdgeIp, predictedTarget,
                   currentRsrp, rsrpTrend);
        return true;
    }

    /**
     * @brief Force write current handover event (for actual handover, not prediction)
     */
    void WriteHandoverEvent(uint64_t imsi,
                            uint16_t sourceCell,
                            uint16_t targetCell,
                            Ipv4Address sourceEdgeIp,
                            Ipv4Address targetEdgeIp,
                            const std::string& eventType)
    {
        if (!m_initialized)
        {
            return;
        }

        std::string eventPath = m_basePath + "_event.json";
        std::string eventTmpPath = m_basePath + "_event.tmp";

        // Format JSON (pre-sized buffer, no dynamic allocation)
        char buf[512];
        int len = snprintf(buf, sizeof(buf),
            "{"
            "\"event\":\"%s\","
            "\"imsi\":%lu,"
            "\"source_cell\":%u,"
            "\"target_cell\":%u,"
            "\"source_edge_ip\":\"%s\","
            "\"target_edge_ip\":\"%s\","
            "\"timestamp_ms\":%ld"
            "}\n",
            eventType.c_str(),
            imsi,
            sourceCell,
            targetCell,
            FormatIp(sourceEdgeIp),
            FormatIp(targetEdgeIp),
            Simulator::Now().GetMilliSeconds());

        AtomicWrite(eventPath, eventTmpPath, buf, len);
    }

private:
    HandoverPredictionFile() = default;

    void WriteState(uint64_t imsi,
                    uint16_t currentCell,
                    Ipv4Address targetEdgeIp,
                    uint16_t predictedTarget,
                    int currentRsrp = 0,
                    double rsrpTrend = 0.0)
    {
        // Format JSON (pre-sized buffer avoids heap allocation)
        char buf[512];
        int len = snprintf(buf, sizeof(buf),
            "{"
            "\"imsi\":%lu,"
            "\"current_cell\":%u,"
            "\"predicted_target\":%u,"
            "\"target_edge_ip\":\"%s\","
            "\"current_rsrp\":%d,"
            "\"rsrp_trend\":%.3f,"
            "\"timestamp_ms\":%ld,"
            "\"has_prediction\":%s"
            "}\n",
            imsi,
            currentCell,
            predictedTarget,
            FormatIp(targetEdgeIp),
            currentRsrp,
            rsrpTrend,
            Simulator::Now().GetMilliSeconds(),
            predictedTarget > 0 ? "true" : "false");

        AtomicWrite(m_filePath, m_tmpPath, buf, len);
    }

    /**
     * @brief Atomic write using rename (prevents partial reads)
     */
    void AtomicWrite(const std::string& finalPath,
                     const std::string& tmpPath,
                     const char* data,
                     size_t len)
    {
        // Write to temp file
        FILE* f = fopen(tmpPath.c_str(), "w");
        if (f)
        {
            fwrite(data, 1, len, f);
            fclose(f);
            // Atomic rename (single syscall, very fast)
            rename(tmpPath.c_str(), finalPath.c_str());
        }
    }

    /**
     * @brief Format IPv4 address to static buffer (avoids allocation)
     */
    const char* FormatIp(Ipv4Address addr)
    {
        static char ipBuf[16];
        uint32_t ip = addr.Get();
        snprintf(ipBuf, sizeof(ipBuf), "%u.%u.%u.%u",
                 (ip >> 24) & 0xFF,
                 (ip >> 16) & 0xFF,
                 (ip >> 8) & 0xFF,
                 ip & 0xFF);
        return ipBuf;
    }

    std::string m_basePath;
    std::string m_filePath;
    std::string m_tmpPath;
    Time m_minWriteInterval;
    Time m_lastWriteTime;
    uint16_t m_lastPredictedTarget;
    bool m_initialized = false;
};

}  // namespace ns3

#endif // HANDOVER_PREDICTION_FILE_H
