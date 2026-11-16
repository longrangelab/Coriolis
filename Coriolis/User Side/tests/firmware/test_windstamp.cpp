// Host test for the node's publish timing (Node/lib/WindStamp).
// Simulates an ATtiny publishing every ~990 ms on its own clock, a node polling
// it every 100 ms that stops polling 250 ms before its TDMA slot and while it
// transmits (~400 ms), and polls that fail now and then. Checks every publish
// gets a time and how close that time is to when the publish really appeared.
//   g++ -std=c++17 -O2 -I../../../Node/lib/WindStamp -I../../../Node/lib/Telemetry test_windstamp.cpp -o test_windstamp && ./test_windstamp
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cmath>
#include <vector>
#include <algorithm>
#include "WindStamp.h"

static int fails = 0, passes = 0;
#define CHECK(c, ...) do { if (c) passes++; else { fails++; printf("FAIL line %d: ", __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

static uint32_t rng = 12345;
static double urand() { rng = rng * 1664525u + 1013904223u; return (rng >> 8) / 16777216.0; }

struct Result { int publishes = 0, stamped = 0, lost = 0, within60 = 0, within100 = 0; int worst = 0; double rms = 0;
                float period = 0; };

// One run. period: ATtiny publish period (ms). failP: chance a poll fails.
// slot: our TDMA slot (1..), cycle: TDMA cycle (ms). Runs `seconds` of time.
// jumpAt/jumpMs: the ATtiny's rhythm shifts by jumpMs at jumpAt ms (a stalled loop).
static Result run(double period, double failP, int slot, uint32_t cycle, int seconds, uint32_t seed,
                  double jumpAt = 0, double jumpMs = 0, bool verbose = false) {
    rng = seed;
    WindStamper st;
    // ATtiny publishes: time each one appears and its sequence.
    std::vector<double> pubT; std::vector<uint8_t> pubSeq;
    double t0 = 37.0 + urand() * 900;
    uint8_t seq = 0;
    bool jumped = false;
    for (double t = t0; t < seconds * 1000.0 + 2000; t += period + (urand() - 0.5) * 4) {
        if (jumpMs != 0 && !jumped && t >= jumpAt) { t += jumpMs; jumped = true; }
        seq = (uint8_t)(seq % 63 + 1);
        pubT.push_back(t); pubSeq.push_back(seq);
    }
    auto frameAt = [&](double t, uint8_t& s, size_t& idx) -> bool {   // newest publish visible at t
        auto it = std::upper_bound(pubT.begin(), pubT.end(), t);
        if (it == pubT.begin()) { s = 0; return false; }
        idx = (size_t)(it - pubT.begin()) - 1; s = pubSeq[idx]; return true;
    };
    std::vector<int> stampedErr(pubT.size(), 1 << 30);

    auto observe = [&](uint32_t now) {
        uint8_t s; size_t idx = 0;
        // the frame is read a millisecond or two after `now`
        bool have = frameAt(now + 1.5, s, idx);
        uint8_t before = st.count; uint8_t headBefore = st.head;
        st.observe(have ? s : 0, (uint16_t)(have ? idx % 500 : 0), 0, now);
        if (st.count != before || st.head != headBefore) {
            const StampedReading& r = st.fromNewest(0);
            // find which publish it was: the newest one with that seq at or before now
            for (size_t j = idx + 1; j-- > 0;) if (pubSeq[j] == r.seq) {
                stampedErr[j] = (int)lround((double)r.ms - pubT[j]); break;
            }
        }
    };

    uint32_t beacon0 = 500;                   // a beacon at the start of each cycle
    uint32_t now = 0, nextPoll = 0;
    const uint32_t guard = 250, slotMs = 450;
    while (now < (uint32_t)seconds * 1000) {
        uint32_t into = (now - beacon0) % cycle;
        uint32_t myOff = (uint32_t)slot * slotMs;
        uint32_t until = myOff >= into ? myOff - into : myOff + cycle - into;
        if (until == 0) {
            observe(now);                         // readSensors() right before transmit
            now += 395 + (uint32_t)(urand() * 25); // transmit (blocking)
            continue;
        }
        if (until < guard) { now++; continue; }
        if ((int32_t)(now - nextPoll) >= 0) {
            nextPoll = now + 100;
            if (urand() < failP) st.failedPoll();
            else observe(now);
            now += 2;                             // the read itself
            continue;
        }
        now += 1 + (urand() < 0.05 ? (uint32_t)(urand() * 15) : 0);   // other loop work
    }

    Result res; double sq = 0;
    for (size_t j = 0; j < pubT.size(); j++) {
        if (pubT[j] < 8000 || pubT[j] > seconds * 1000.0 - 1500) continue;   // warm-up / tail
        res.publishes++;
        if (stampedErr[j] == (1 << 30)) { res.lost++; if (verbose) printf("  lost publish at %.0f\n", pubT[j]); continue; }
        int e = std::abs(stampedErr[j]);
        res.stamped++;
        if (e <= 60) res.within60++;
        if (e <= 100) res.within100++;
        res.worst = std::max(res.worst, e);
        sq += (double)e * e;
    }
    res.rms = res.stamped ? sqrt(sq / res.stamped) : 0;
    res.period = st.period;
    return res;
}

static void report(const char* name, const Result& r) {
    printf("%-34s %5d publishes, %d lost, %.1f%% within 60 ms, %.1f%% within 100 ms, rms %.0f ms, worst %d ms\n",
           name, r.publishes, r.lost, 100.0 * r.within60 / std::max(1, r.stamped),
           100.0 * r.within100 / std::max(1, r.stamped), r.rms, r.worst);
}

int main() {
    // 1) the bench case: node in slot 3 of a 2.7 s cycle, ~2 failed reads a minute
    for (double p : {980.0, 990.0, 1000.0}) {
        char name[64]; snprintf(name, sizeof name, "slot 3, period %.0f, 2 fails/min", p);
        Result r = run(p, 2.0 / 600, 3, 2700, 1800, 1000 + (uint32_t)p);
        report(name, r);
        CHECK(r.lost == 0, "%s: %d publishes lost", name, r.lost);
        CHECK(r.within100 * 1.0 >= 0.995 * r.stamped, "%s: only %d/%d within 100 ms", name, r.within100, r.stamped);
        CHECK(r.within60 * 1.0 >= 0.97 * r.stamped, "%s: only %d/%d within 60 ms", name, r.within60, r.stamped);
        CHECK(r.worst <= 100, "%s: worst %d ms", name, r.worst);
        CHECK(fabs(r.period - p) < 3, "%s: fitted period %.1f", name, r.period);
    }
    // 2) other slots and cycle lengths, more failed reads
    for (int slot = 1; slot <= 4; slot++) {
        char name[64]; snprintf(name, sizeof name, "slot %d, cycle 2700, 1%% fails", slot);
        Result r = run(991.3, 0.01, slot, 2700, 1800, 77 + slot);
        report(name, r);
        CHECK(r.lost == 0, "%s: %d lost", name, r.lost);
        CHECK(r.within100 * 1.0 >= 0.99 * r.stamped, "%s: %d/%d within 100 ms", name, r.within100, r.stamped);
    }
    {
        Result r = run(986.0, 2.0 / 600, 5, 3150, 1800, 9);
        report("slot 5, cycle 3150", r);
        CHECK(r.lost == 0 && r.within100 * 1.0 >= 0.99 * r.stamped, "slot 5 / 3150 ms cycle");
    }
    // 3) a bad patch: 20% of reads fail. Nothing should be placed badly; a few may be lost.
    {
        Result r = run(990.0, 0.20, 3, 2700, 600, 4242);
        report("slot 3, 20% fails", r);
        CHECK(r.lost * 1.0 <= 0.05 * r.publishes, "20%% fails: %d/%d lost", r.lost, r.publishes);
        CHECK(r.worst <= 200, "20%% fails: worst %d ms", r.worst);
    }

    // 4) the ATtiny's rhythm jumps (its loop stalled): the fit is rebuilt within seconds
    for (double j : {300.0, -450.0, 60.0}) {
        char name[64]; snprintf(name, sizeof name, "rhythm jumps %+.0f ms", j);
        Result r = run(990.0, 2.0 / 600, 3, 2700, 600, 31, 300000, j);
        report(name, r);
        CHECK(r.lost <= 3, "%s: %d lost", name, r.lost);
        CHECK(r.stamped - r.within100 <= 3, "%s: %d off by more than 100 ms", name, r.stamped - r.within100);
    }

    // 5) build(): newest three consecutive publishes, synced and unsynced times
    {
        WindStamper st;
        uint32_t t = 1000;
        for (; t < 7000; t += 100) {               // publishes at 1050, 2050, ... seen by 100 ms polls
            int n = t >= 1050 ? (int)(t - 1050) / 1000 + 1 : 0;
            st.observe((uint8_t)n, (uint16_t)(1000 + n), (uint16_t)(900 * n), t);
        }
        WindExt e;
        CHECK(st.build(e, t, true, true, t - 300, 0x5A), "build ok");
        CHECK(windExtCount(e) == 3 && windExtSynced(e) && e.beacon == 0x5A, "3 readings, synced, beacon byte");
        CHECK(e.t_ms == (int16_t)(st.fromNewest(0).ms - (t - 300)), "t_ms after beacon (%d)", e.t_ms);
        CHECK(e.r[0][0] == windExtSpeedByte(10.06f) && e.r[1][0] == windExtSpeedByte(10.05f), "newest first");
        CHECK(st.build(e, t, true, false, 0, 0) && !windExtSynced(e) && e.t_ms < 0 && e.t_ms > -1100, "unsynced: ms before now (%d)", e.t_ms);
        CHECK(!st.build(e, t, false, true, t - 300, 1), "no block when wind invalid");
        CHECK(!st.build(e, t + 6000, true, true, t - 300, 1), "no block when the ATtiny went quiet");
        // ATtiny restarts: seq 0, then 1 again; the history before the restart isn't linked
        st.observe(0, 0, 0, t); t += 100;
        for (int k = 0; k < 5; k++, t += 100) st.observe(1, 0, 0, t);
        st.observe(2, 5, 0, t);
        CHECK(st.build(e, t, true, false, 0, 0) && windExtCount(e) == 1, "after a restart only the new publish (%d)", windExtCount(e));
    }

    printf("%d passed, %d failed\n", passes, fails);
    return fails ? 1 : 0;
}
