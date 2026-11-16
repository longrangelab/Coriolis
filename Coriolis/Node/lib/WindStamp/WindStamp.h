#ifndef WIND_STAMP_H
#define WIND_STAMP_H

/**
 * Node side of the timed 1 s wind readings: when did each ATtiny publish appear?
 * =============================================================================
 * The ATtiny publishes a new frame about once a second (its own RC clock, so
 * 980-1000 ms on the bench) with a 6-bit sequence number (1..63, 0 = no data
 * yet). The node reads it every 100 ms and calls observe() with each frame.
 *
 * Each publish is known to fall between the poll that last saw the old
 * sequence and the poll that first sees the new one. With polls 100 ms apart
 * that alone is +/-50 ms. Some gaps are longer: the node stops polling just
 * before its transmit slot and while transmitting (~650 ms), and a poll can
 * fail. Halfway in a 650 ms gap is up to +/-325 ms off (bench 2026-11-15).
 *
 * So the stamper also fits a straight line (publish number -> time) through
 * the midpoints of the last STAMP_FIT_N publishes seen in short gaps. The
 * ATtiny's clock is steady over that ~20 s, so the line says where each
 * publish fell to about +/-20 ms, better than any single midpoint. Every
 * publish is placed on the line, kept inside the gap it was seen in. When the
 * line disagrees with the gaps twice running (the ATtiny restarted or its
 * clock moved), it is thrown away and rebuilt. A failed poll is simply skipped
 * (it used to throw away the next publish's time).
 *
 * Plain C++ (no Arduino calls): the host test in tests/firmware runs it
 * against simulated publishes, polls, transmit gaps and failed reads.
 */

#include <stdint.h>
#include <string.h>
#include "WindExt.h"

#define STAMP_RING          4
#define STAMP_PRECISE_MS    150      // a gap this short goes into the fit
#define STAMP_MAX_GAP_MS    1500     // longer than this: can't place a publish
#define STAMP_MID_ONLY_MS   600      // without a fit, midpoint only up to here
#define STAMP_RESYNC_MS     30000UL  // can't count publishes across a longer gap
#define STAMP_FIT_N         16       // publishes in the fit
#define STAMP_FIT_MIN       4        // fewest publishes to trust the fit
#define STAMP_FIT_AGE_MS    30000UL  // fit points older than this are dropped
#define STAMP_SLACK_MS      40       // fit may land this far outside the gap

struct StampedReading { uint32_t ms; uint16_t spd; uint16_t dir; uint8_t seq; };

// Steps from sequence a to sequence b in the ATtiny's 1..63 cycle.
static inline uint8_t stampSeqSteps(uint8_t a, uint8_t b) {
    return (uint8_t)((b + 63 - a) % 63);
}

class WindStamper {
public:
    StampedReading ring[STAMP_RING];
    uint8_t head = 0, count = 0;
    float   period = 0;                  // fitted ATtiny publish period, ms (0 = no fit)

    // A good frame read at `now` (millis() just before the read started).
    // spd = mph x 100, dir = deg x 10, as the ATtiny sends them.
    void observe(uint8_t seq, uint16_t spd, uint16_t dir, uint32_t now) {
        if (seq == 0) { havePoll = false; fitN = 0; return; }   // ATtiny (re)started
        if (havePoll && seq != lastSeq) {
            uint32_t gap = now - lastPollMs;
            uint8_t steps = stampSeqSteps(lastSeq, seq);
            if (gap > STAMP_RESYNC_MS) fitN = 0;
            pubIdx += steps;
            // steps > 1: publishes went by unseen; don't guess which is which
            if (steps == 1 && gap <= STAMP_MAX_GAP_MS) place(seq, spd, dir, now, gap);
        }
        lastSeq = seq; lastPollMs = now; havePoll = true;
    }

    // A poll that failed: nothing to do. The next good read is bracketed by
    // the last good one, so its publish still gets a time.
    void failedPoll() {}

    const StampedReading& fromNewest(uint8_t k) const {
        return ring[(head + STAMP_RING - 1 - k) % STAMP_RING];
    }

    // Fill e with the newest publish and up to two consecutive ones before it.
    // Synced: t_ms after the beacon heard at beaconRxMs (seq low byte
    // beaconSeqLow). Otherwise: t_ms before `now` (the transmit starts within a
    // couple of ms). False when there's nothing recent to send.
    bool build(WindExt& e, uint32_t now, bool windValid, bool synced, uint32_t beaconRxMs,
               uint8_t beaconSeqLow) const {
        if (!windValid || count == 0) return false;
        const StampedReading& newest = fromNewest(0);
        if ((uint32_t)(now - newest.ms) > 5000) return false;            // ATtiny went quiet
        uint8_t n = 1;
        while (n < WIND_EXT_MAX && n < count) {
            const StampedReading& r = fromNewest(n);
            if (stampSeqSteps(r.seq, fromNewest(n - 1).seq) != 1) break;   // a publish is missing
            if ((uint32_t)(newest.ms - r.ms) > (uint32_t)n * 1500) break;
            n++;
        }
        memset(&e, 0, sizeof(e));
        int32_t tb = (int32_t)(newest.ms - beaconRxMs);
        if (synced && (uint32_t)(now - beaconRxMs) < 30000UL && tb > -30000 && tb < 30000) {
            e.tag = (uint8_t)(WIND_EXT_TAG | WIND_EXT_SYNCED | n);
            e.beacon = beaconSeqLow;
            e.t_ms = (int16_t)tb;
        } else {
            e.tag = (uint8_t)(WIND_EXT_TAG | n);
            e.t_ms = (int16_t)-(int32_t)(now - newest.ms);
        }
        for (uint8_t k = 0; k < n; k++) {
            const StampedReading& r = fromNewest(k);
            e.r[k][0] = windExtSpeedByte(r.spd / 100.0f);
            e.r[k][1] = windExtDirByte(r.dir / 10.0f);
        }
        return true;
    }

private:
    uint8_t  lastSeq = 0;
    uint32_t lastPollMs = 0;
    bool     havePoll = false;
    int32_t  pubIdx = 0;                 // publishes counted since start (unwrapped seq)
    int32_t  fitIdx[STAMP_FIT_N];        // fit points: publish number, midpoint time
    uint32_t fitMs[STAMP_FIT_N];
    uint8_t  fitN = 0, fitHead = 0, misfits = 0;

    // Where the fit puts publish number idx. False without a usable fit.
    bool predict(int32_t idx, uint32_t now, uint32_t& out) {
        if (fitN < STAMP_FIT_MIN) return false;
        const uint8_t last = (uint8_t)((fitHead + STAMP_FIT_N - 1) % STAMP_FIT_N);
        const int32_t x0 = fitIdx[last];
        const uint32_t t0 = fitMs[last];
        double sx = 0, sy = 0, sxx = 0, sxy = 0;
        int n = 0, xmin = 0;
        for (uint8_t i = 0; i < fitN; i++) {
            uint8_t j = (uint8_t)((fitHead + STAMP_FIT_N - 1 - i) % STAMP_FIT_N);
            if ((uint32_t)(now - fitMs[j]) > STAMP_FIT_AGE_MS) break;
            double x = fitIdx[j] - x0, y = (double)(int32_t)(fitMs[j] - t0);
            sx += x; sy += y; sxx += x * x; sxy += x * y; n++;
            if (x < xmin) xmin = (int)x;
        }
        if (n < STAMP_FIT_MIN || xmin > -3) return false;
        double d = n * sxx - sx * sx;
        if (d <= 0) return false;
        double b = (n * sxy - sx * sy) / d;               // ms per publish
        if (b < 900.0 || b > 1100.0) return false;
        double a = (sy - b * sx) / n;
        double p = a + b * (double)(idx - x0);
        if (p < -60000.0 || p > 60000.0) return false;
        period = (float)b;
        out = t0 + (uint32_t)(int32_t)(p >= 0 ? p + 0.5 : p - 0.5);
        return true;
    }

    void place(uint8_t seq, uint16_t spd, uint16_t dir, uint32_t now, uint32_t gap) {
        uint32_t mid = lastPollMs + gap / 2;
        uint32_t t = mid;
        bool placed = false;
        uint32_t pred;
        if (predict(pubIdx, now, pred)) {
            int32_t afterStart = (int32_t)(pred - lastPollMs), beforeEnd = (int32_t)(now - pred);
            if (afterStart >= -STAMP_SLACK_MS && beforeEnd >= -STAMP_SLACK_MS) {
                t = afterStart < 0 ? lastPollMs : (beforeEnd < 0 ? now : pred);
                placed = true; misfits = 0;
            } else if (++misfits >= 2) {
                fitN = 0; misfits = 0; period = 0;               // the rhythm moved: rebuild
            }
        }
        if (!placed && gap > STAMP_MID_ONLY_MS) return;
        if (gap <= STAMP_PRECISE_MS) {
            fitIdx[fitHead] = pubIdx; fitMs[fitHead] = mid;
            fitHead = (uint8_t)((fitHead + 1) % STAMP_FIT_N);
            if (fitN < STAMP_FIT_N) fitN++;
        }
        StampedReading& r = ring[head];
        r.ms = t; r.spd = spd; r.dir = dir; r.seq = seq;
        head = (uint8_t)((head + 1) % STAMP_RING);
        if (count < STAMP_RING) count++;
    }
};

#endif // WIND_STAMP_H
