#ifndef WINDSTATS_H
#define WINDSTATS_H

/**
 * WindStats.h  --  rolling wind history + windowed stability   (STANDALONE)
 * ========================================================================
 * Buffers the aggregate (unweighted mean) wind on the receiver and reports how
 * much it varied over the last 10 / 30 / 60 s. Runs continuously on the box, so
 * the record is intact regardless of whether a phone is connected -- and it's
 * the rolling-buffer foundation the Sentinel/lag-time work will build on.
 *
 * Pure data/math (no WiFi, no radio) so it can be tested in isolation.
 *
 * "Stability" here = variability of the mean wind over a window:
 *   speed SD (mph) and circular direction SD (deg, Yamartino). Lower = steadier.
 * The current/instantaneous tier uses the nodes' own 2 s SDs (computed upstream);
 * this module supplies the longer-window view.
 */

#include <Arduino.h>
#include <math.h>

#define WINDSTATS_CAP     160     // ring capacity (2 Hz * 60 s = 120, + margin)
#define WINDSTATS_PERIOD  500UL   // sample every 500 ms (2 Hz)

class WindStats {
private:
    struct Sample { uint32_t t; float spd; float dir; };
    Sample   buf[WINDSTATS_CAP];
    int      head = 0, count = 0;
    uint32_t lastSample = 0;

public:
    // Feed the current aggregate mean wind each loop; throttled internally.
    void update(float aggSpeed, float aggDir, bool valid) {
        if (!valid) return;
        uint32_t now = millis();
        if (now - lastSample < WINDSTATS_PERIOD) return;
        lastSample = now;
        buf[head] = { now, aggSpeed, aggDir };
        head = (head + 1) % WINDSTATS_CAP;
        if (count < WINDSTATS_CAP) count++;
    }

    // Variability over the last `sec` seconds. Returns sample count.
    int window(int sec, float& spdSd, float& dirSd) {
        spdSd = 0; dirSd = 0;
        uint32_t now = millis();
        uint32_t cutoff = now - (uint32_t)sec * 1000UL;
        float sSum = 0, sSq = 0, cx = 0, cy = 0;
        int n = 0;
        for (int i = 0; i < count; i++) {
            int idx = (head - 1 - i + WINDSTATS_CAP) % WINDSTATS_CAP;
            if (buf[idx].t < cutoff) break;              // older than window
            sSum += buf[idx].spd;
            sSq  += buf[idx].spd * buf[idx].spd;
            float r = buf[idx].dir * (float)DEG_TO_RAD;
            cx += cosf(r); cy += sinf(r);
            n++;
        }
        if (n < 2) return n;
        float mean = sSum / n;
        float var  = sSq / n - mean * mean; if (var < 0) var = 0;
        spdSd = sqrtf(var);
        float R = sqrtf(cx*cx + cy*cy) / n; if (R > 1) R = 1;
        float eps = sqrtf(1 - R*R);
        dirSd = asinf(eps) * (1 + 0.1547f*eps*eps*eps) * (float)RAD_TO_DEG;
        return n;
    }
};

#endif // WINDSTATS_H
