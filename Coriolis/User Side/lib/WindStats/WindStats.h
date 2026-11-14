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
 * The nodes' own SDs (rolling 10 s window on the ATtiny) are the short-term
 * view; this class supplies the 10/30/60 s view of the field average.
 *
 * Kept for older phones (the /data.json "stability" block). It samples the
 * field average at 2 Hz, but each node only reports every 2-3 s, so most
 * samples repeat, and averaging the meters cancels their wobble. The newer
 * phone page uses NodeWindHistory below instead.
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
        if ((uint32_t)(now - lastSample) < WINDSTATS_PERIOD) return;   // rollover-safe
        lastSample = now;
        buf[head] = { now, aggSpeed, aggDir };
        head = (head + 1) % WINDSTATS_CAP;
        if (count < WINDSTATS_CAP) count++;
    }

    // Variability over the last `sec` seconds. Returns sample count.
    //
    // FIX: this used to compute an absolute `cutoff = now - sec*1000` and test
    // `buf[idx].t < cutoff`. That is wrong twice:
    //   1) For the first `sec` seconds of uptime, now < sec*1000, so the
    //      unsigned subtraction wraps to ~4.29e9 and EVERY sample looks older
    //      than the window -> the loop breaks immediately and stability
    //      reported 0 samples until uptime exceeded the window length.
    //   2) At the 49.7-day millis() rollover the same comparison inverts and
    //      the buffer looks empty again. That one matters now: months-long
    //      solar deployment is the whole point of the deep-sleep work.
    // Comparing the unsigned DIFFERENCE is correct in both cases -- modular
    // arithmetic makes (now - t) the true elapsed time straight through a wrap.
    int window(int sec, float& spdSd, float& dirSd) {
        spdSd = 0; dirSd = 0;
        uint32_t now = millis();
        uint32_t winMs = (uint32_t)sec * 1000UL;
        float sSum = 0, sSq = 0, cx = 0, cy = 0;
        int n = 0;
        for (int i = 0; i < count; i++) {
            int idx = (head - 1 - i + WINDSTATS_CAP) % WINDSTATS_CAP;
            if ((uint32_t)(now - buf[idx].t) > winMs) break;   // older than window
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

// ============================================================================
// NodeWindHistory  --  per-node wind vectors, logged as each packet arrives
// ============================================================================
// The phone splits field variability into three parts (gusting, spread,
// drift). Two of them need history the phone does not keep:
//
//   smooth()  the node's average wind vector over its last 10 s of packets.
//             Used to compare meters (spread). A node's own reading can be a
//             short (3 s) average that jumps with every gust; comparing those
//             would count gusting a second time as "the meters disagree".
//   drift()   how much that 10 s average moved over the last 60 s or 5 min,
//             as the covariance of its north/east parts. The base does not
//             know the line of fire, so the phone projects this onto the shot:
//                 var(cross) = cos^2(az)*yy + sin^2(az)*xx - 2*sin(az)*cos(az)*xy
//
// Vector convention matches vectorMeanWind(): x = speed*cos(dir),
// y = speed*sin(dir), dir = the direction the wind blows FROM. Crosswind for a
// shot at azimuth az is then y*cos(az) - x*sin(az) (+ = from the right).
//
// Samples are logged per packet (not on a clock), so a node that stops
// talking adds nothing instead of repeating its last value. All times are
// compared as unsigned differences, safe through the millis() rollover.
#define NWH_NODES      16          // must cover node ids 1..MAX_NODES-1
#define NWH_CAP        160         // per node: 5 min at one packet per ~1.9 s
#define NWH_SMOOTH_MS  10000UL     // the "10 s average" used for spread/drift

class NodeWindHistory {
private:
    struct S { uint32_t t; float x, y; };
    S       buf[NWH_NODES][NWH_CAP];
    uint8_t head[NWH_NODES]  = {0};
    uint8_t count[NWH_NODES] = {0};

    // i = 0 newest, 1 next older, ...
    const S& at(uint8_t id, int i) const {
        return buf[id][(head[id] + NWH_CAP - 1 - i) % NWH_CAP];
    }
    // Mean of the samples no more than NWH_SMOOTH_MS older than sample j.
    void meanAround(uint8_t id, int j, float& x, float& y) const {
        uint32_t tj = at(id, j).t;
        float sx = 0, sy = 0; int n = 0;
        for (int k = j; k < count[id]; k++) {
            const S& s = at(id, k);
            if ((uint32_t)(tj - s.t) > NWH_SMOOTH_MS) break;
            sx += s.x; sy += s.y; n++;
        }
        x = sx / n; y = sy / n;               // n >= 1 (sample j itself)
    }

public:
    void add(uint8_t id, float speed, float dirDeg) {
        if (id >= NWH_NODES) return;
        float r = dirDeg * (float)DEG_TO_RAD;
        buf[id][head[id]] = { millis(), speed * cosf(r), speed * sinf(r) };
        head[id] = (uint8_t)((head[id] + 1) % NWH_CAP);
        if (count[id] < NWH_CAP) count[id]++;
    }

    // 10 s average vector, ending at the node's newest packet (so a node that
    // has gone quiet keeps its last average; the base's timeout drops it from
    // the wind call). Returns how many packets it averaged; 0 = no history.
    int smooth(uint8_t id, float& x, float& y) const {
        x = 0; y = 0;
        if (id >= NWH_NODES || count[id] == 0) return 0;
        uint32_t t0 = at(id, 0).t;
        int n = 0;
        for (int k = 0; k < count[id]; k++) {
            if ((uint32_t)(t0 - at(id, k).t) > NWH_SMOOTH_MS) break;
            n++;
        }
        meanAround(id, 0, x, y);
        return n;
    }

    // Covariance of the node's 10 s averages over the last winMs (ending now).
    // Population covariance (divide by n): it describes this window, it is not
    // an estimate of a wider population. Returns the number of averages used;
    // under 3 the outputs are 0 and the phone shows drift as unknown.
    int drift(uint8_t id, uint32_t winMs, float& xx, float& yy, float& xy) const {
        xx = yy = xy = 0;
        if (id >= NWH_NODES) return 0;
        uint32_t now = millis();
        float mx[NWH_CAP], my[NWH_CAP];
        int n = 0;
        for (int j = 0; j < count[id]; j++) {
            if ((uint32_t)(now - at(id, j).t) > winMs) break;
            meanAround(id, j, mx[n], my[n]);
            n++;
        }
        if (n < 3) return n;
        float ax = 0, ay = 0;
        for (int i = 0; i < n; i++) { ax += mx[i]; ay += my[i]; }
        ax /= n; ay /= n;
        for (int i = 0; i < n; i++) {
            float dx = mx[i] - ax, dy = my[i] - ay;
            xx += dx * dx; yy += dy * dy; xy += dx * dy;
        }
        xx /= n; yy /= n; xy /= n;
        return n;
    }
};

#endif // WINDSTATS_H
