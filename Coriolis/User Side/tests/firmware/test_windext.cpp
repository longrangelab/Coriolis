// Host test for the timed 1 s wind readings (WindExt.h, WindSec.h).
//   g++ -std=c++17 -O2 -I../../lib/Telemetry test_windext.cpp -o test_windext && ./test_windext
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <vector>
#include "WindExt.h"
#include "WindSec.h"

static int fails = 0, passes = 0;
#define CHECK(c, ...) do { if (c) passes++; else { fails++; printf("FAIL %s:%d  ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

static uint32_t rng = 12345;
static double urand() { rng = rng * 1664525u + 1013904223u; return rng / 4294967296.0; }

int main() {
    // ---- wire format
    CHECK(sizeof(WindExt) == 10, "size %zu", sizeof(WindExt));
    CHECK(windExtSpeed(windExtSpeedByte(12.37f)) == 12.25f || windExtSpeed(windExtSpeedByte(12.37f)) == 12.5f, "speed rounding");
    CHECK(windExtSpeedByte(-1) == 0 && windExtSpeedByte(100) == 255, "speed clamps");
    CHECK(windExtDirByte(359.9f) == 0, "359.9 deg wraps to 0, got %d", windExtDirByte(359.9f));
    CHECK(fabsf(windExtDir(windExtDirByte(271.0f)) - 271.0f) < 0.71f, "dir round trip");

    // ---- airtime, SF9 BW125 CR4/7 preamble 8 (numbers in the design notes)
    CHECK(loraAirtimeMs(36, 9, 125.0f, 7, 8) == 341, "36 B %u", loraAirtimeMs(36, 9, 125.0f, 7, 8));
    CHECK(loraAirtimeMs(46, 9, 125.0f, 7, 8) == 398, "46 B %u", loraAirtimeMs(46, 9, 125.0f, 7, 8));
    CHECK(loraAirtimeMs(48, 9, 125.0f, 7, 8) == 398, "48 B %u", loraAirtimeMs(48, 9, 125.0f, 7, 8));
    CHECK(loraAirtimeMs(49, 9, 125.0f, 7, 8) == 427, "49 B %u", loraAirtimeMs(49, 9, 125.0f, 7, 8));
    CHECK(loraAirtimeMs(46, 9, 125.0f, 7, 8) + 50 <= 450, "46 B fits the 450 ms slot with a guard");

    // ---- parse after a 36-byte packet (sizeof(NodePacket))
    uint8_t buf[64] = {0};
    WindExt e{}; e.tag = WIND_EXT_TAG | WIND_EXT_SYNCED | 3; e.beacon = 7; e.t_ms = 1234;
    memcpy(buf + 36, &e, sizeof e);
    WindExt got;
    CHECK(!windExtParse(buf, 36, 36, got), "plain packet has no block");
    CHECK(windExtParse(buf, 46, 36, got) && got.t_ms == 1234 && got.beacon == 7 && windExtCount(got) == 3, "block parses");
    buf[36] = 0x13; CHECK(!windExtParse(buf, 46, 36, got), "wrong tag rejected");
    buf[36] = WIND_EXT_TAG; CHECK(!windExtParse(buf, 46, 36, got), "count 0 rejected");

    // ---- resolve: beacon-synced and arrival-timed
    BeaconLog log;
    for (uint32_t s = 1; s <= 20; s++) log.record(s, 1000 + s * 2700);   // beacon s done at 1000+2700s
    uint32_t t; WindTimeSrc src;
    WindExt s1{}; s1.tag = WIND_EXT_TAG | WIND_EXT_SYNCED | 1; s1.beacon = 20; s1.t_ms = -300;
    CHECK(windExtResolve(s1, log, 1000 + 20 * 2700 + 900, 398, t, src) && src == WT_BEACON && t == 1000 + 20 * 2700 - 300, "synced time %u", t);
    s1.beacon = 2;      // overwritten long ago
    CHECK(!windExtResolve(s1, log, 1000 + 20 * 2700 + 900, 398, t, src), "unknown beacon is not placed");
    s1.beacon = 20; s1.t_ms = 2000;   // claims to be newer than its own packet
    CHECK(!windExtResolve(s1, log, 1000 + 20 * 2700 + 900, 398, t, src), "reading after arrival rejected");
    WindExt u{}; u.tag = WIND_EXT_TAG | 2; u.t_ms = -450;
    CHECK(windExtResolve(u, log, 90000, 398, t, src) && src == WT_ARRIVAL && t == 90000 - 398 - 450, "arrival time %u", t);
    u.t_ms = 10; CHECK(!windExtResolve(u, log, 90000, 398, t, src), "unsynced time after TX rejected");

    // ---- store: a node whose ATtiny really publishes every 1020 ms, sending
    // every 2.7 s with the newest 3 publishes, stamped with +/-50 ms poll noise.
    WindSecStore<16> st;
    const double P = 1020.0, t0 = 5000.0;
    int published = 0, lastSent = -1;
    double worst = 0;
    for (int cyc = 0; cyc < 80; cyc++) {
        double tx = 7000.0 + cyc * 2700.0;
        int newest = (int)floor((tx - t0) / P);               // publishes 0..newest exist
        WindExt pk{};
        int n = newest - lastSent; if (n > 3) n = 3; if (n < 1) n = 1;
        pk.tag = WIND_EXT_TAG | WIND_EXT_SYNCED | n; pk.beacon = 0;
        double stamp = t0 + newest * P + (urand() - 0.5) * 100.0;
        for (int k = 0; k < n; k++) { pk.r[k][0] = (uint8_t)((newest - k) & 0xFF); pk.r[k][1] = 0; }
        st.ingest(3, pk, (uint32_t)lround(stamp), WT_BEACON);
        lastSent = newest; published = newest + 1;
    }
    // every publish from the first packet on should be there once, in order
    int n = st.size(3);
    bool ordered = true, unique = true;
    for (int i = 0; i < n; i++) {
        const WindSec& w = st.at(3, i);
        int id = w.spd;                                        // we stored the publish number
        double truth = t0 + id * P;
        worst = fmax(worst, fabs((double)w.t - truth));
        if (i && (int32_t)(w.t - st.at(3, i - 1).t) <= 0) ordered = false;
        if (i && (uint8_t)(st.at(3, i - 1).spd + 1) != (uint8_t)id) unique = false;
    }
    CHECK(n == WSEC_CAP || n >= published - 3, "kept %d of %d publishes", n, published);
    CHECK(ordered && unique, "every publish once, in time order");
    CHECK(fabs(st.publishPeriod(3) - P) < 8, "learned ATtiny period %.1f (true %.0f)", st.publishPeriod(3), P);
    CHECK(worst < 110, "worst time error %.0f ms", worst);
    printf("store: %d readings, period %.1f ms, worst time error %.0f ms\n", n, st.publishPeriod(3), worst);

    printf("%d passed, %d failed\n", passes, fails);
    return fails ? 1 : 0;
}
