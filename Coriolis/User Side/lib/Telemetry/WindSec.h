#ifndef WIND_SEC_H
#define WIND_SEC_H

/**
 * Base side of the timed 1-second wind readings (see WindExt.h)
 * =============================================================
 * BeaconLog remembers when each recent beacon finished transmitting, on the
 * base's millis() clock. WindSecStore turns each packet's WindExt block into
 * readings on that same clock and keeps the last WSEC_CAP of them per node, so
 * the phone can ask for everything since a given time (/windsec).
 *
 * No Arduino calls in here: the host test (tests/firmware) compiles it with g++.
 */

#include <stdint.h>
#include <stddef.h>
#include "WindExt.h"

#ifndef WSEC_CAP
#define WSEC_CAP 300          // readings kept per node (about 5 minutes): the phone's wind
                              // log catches up from here after its screen was off a while
#endif
#define WSEC_DEDUP_MS   500   // a reading within this of one we have is the same publish
#define BEACON_LOG_N    16    // beacons remembered (43 s at a 2.7 s cycle)
#define BEACON_MAX_AGE  60000UL

// When each recent beacon finished going out. Keyed by the seq low byte the
// node echoes back.
class BeaconLog {
    uint8_t  seqLow[BEACON_LOG_N] = {0};
    uint32_t doneMs[BEACON_LOG_N] = {0};
    bool     used[BEACON_LOG_N]   = {false};
    uint8_t  head = 0;
public:
    void record(uint32_t seq, uint32_t ms) {
        seqLow[head] = (uint8_t)seq; doneMs[head] = ms; used[head] = true;
        head = (uint8_t)((head + 1) % BEACON_LOG_N);
    }
    // Newest beacon with this seq low byte, if it's recent enough to be the one.
    bool find(uint8_t s, uint32_t now, uint32_t& ms) const {
        for (int k = 1; k <= BEACON_LOG_N; k++) {
            int i = (head - k + BEACON_LOG_N) % BEACON_LOG_N;
            if (!used[i]) return false;
            if (seqLow[i] == s) {
                if ((uint32_t)(now - doneMs[i]) > BEACON_MAX_AGE) return false;
                ms = doneMs[i];
                return true;
            }
        }
        return false;
    }
};

// How a node's readings were timed, newest packet. For /windsec and the log.
enum WindTimeSrc : uint8_t { WT_NONE = 0, WT_BEACON = 1, WT_ARRIVAL = 2 };

// Base-clock time of the newest reading in a block. arrivalMs = when the
// packet finished arriving, len = its length in bytes. False when the block
// can't be placed (unknown beacon, or a time that makes no sense).
static inline bool windExtResolve(const WindExt& e, const BeaconLog& log, uint32_t arrivalMs,
                                  uint32_t airtimeMs, uint32_t& tNewest, WindTimeSrc& src) {
    if (windExtSynced(e)) {
        uint32_t b;
        if (!log.find(e.beacon, arrivalMs, b)) return false;
        tNewest = b + (uint32_t)(int32_t)e.t_ms;
        src = WT_BEACON;
    } else {
        if (e.t_ms > 0) return false;                 // unsynced times are before TX
        tNewest = arrivalMs - airtimeMs + (uint32_t)(int32_t)e.t_ms;
        src = WT_ARRIVAL;
    }
    // A reading can't be newer than the packet carrying it (a little slack for
    // the two clocks' few-ms difference), and a block 30 s old is a bad match.
    int32_t lead = (int32_t)(tNewest - arrivalMs);
    if (lead > 50 || lead < -30000) return false;
    return true;
}

#pragma pack(push, 1)
struct WindSec {
    uint32_t t;        // base millis() when this publish appeared on the node
    uint8_t  spd;      // 0.25 mph
    uint8_t  dir;      // 360/256 deg
};
#pragma pack(pop)

template <int NODES>
class WindSecStore {
    WindSec  buf[NODES][WSEC_CAP];
    uint16_t head[NODES];
    uint16_t count[NODES];
    uint32_t lastNewest[NODES];
    float    period[NODES];         // ms between ATtiny publishes, learned per node
    WindTimeSrc src_[NODES];
public:
    WindSecStore() { clear(); }
    void clear() {
        for (int n = 0; n < NODES; n++) { head[n] = 0; count[n] = 0; lastNewest[n] = 0; period[n] = WIND_EXT_SPACING_MS; src_[n] = WT_NONE; }
    }
    int  size(int node) const { return (node >= 0 && node < NODES) ? count[node] : 0; }
    WindTimeSrc source(int node) const { return (node >= 0 && node < NODES) ? src_[node] : WT_NONE; }
    float publishPeriod(int node) const { return (node >= 0 && node < NODES) ? period[node] : WIND_EXT_SPACING_MS; }

    // i = 0 oldest .. size-1 newest
    const WindSec& at(int node, int i) const {
        int idx = (head[node] - count[node] + i + WSEC_CAP) % WSEC_CAP;
        return buf[node][idx];
    }

    // Add a packet's readings. Returns how many were new.
    int ingest(int node, const WindExt& e, uint32_t tNewest, WindTimeSrc src) {
        if (node < 0 || node >= NODES) return 0;
        uint8_t n = windExtCount(e);
        if (n < 1 || n > WIND_EXT_MAX) return 0;
        src_[node] = src;

        // The ATtiny runs on its own RC clock, so its "1 s" can be a few
        // percent off. Two packets' newest readings are a whole number of
        // publishes apart; learn the real spacing from that, slowly.
        if (count[node]) {
            int32_t dt = (int32_t)(tNewest - lastNewest[node]);
            if (dt >= 700 && dt <= 12000) {
                int k = (int)((dt + 500) / 1000);
                if (k >= 1) {
                    float p = (float)dt / k;
                    if (p >= 900 && p <= 1100) period[node] += 0.2f * (p - period[node]);
                }
            }
        }
        lastNewest[node] = tNewest;

        int added = 0;
        for (int k = n - 1; k >= 0; k--) {            // oldest first, so the ring stays in order
            uint32_t t = tNewest - (uint32_t)(k * period[node] + 0.5f);
            if (count[node]) {
                const WindSec& last = at(node, count[node] - 1);
                if ((int32_t)(t - last.t) < WSEC_DEDUP_MS) continue;   // have it (or older)
            }
            WindSec& w = buf[node][head[node]];
            w.t = t; w.spd = e.r[k][0]; w.dir = e.r[k][1];
            head[node] = (uint16_t)((head[node] + 1) % WSEC_CAP);
            if (count[node] < WSEC_CAP) count[node]++;
            added++;
        }
        return added;
    }
};

#endif // WIND_SEC_H
