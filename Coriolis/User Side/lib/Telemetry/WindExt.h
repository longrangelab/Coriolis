#ifndef WIND_EXT_H
#define WIND_EXT_H

/**
 * Timed 1-second wind readings, appended after NodePacket
 * =======================================================
 * >>> Identical copy in Coriolis/Node/lib/Telemetry and
 *     Coriolis/User Side/lib/Telemetry. CI checks they match. <<<
 *
 * The ATtiny publishes a new wind figure every second (speed_avg / dir_avg =
 * the newest 2 s). The node used to send only the one figure it read at its
 * transmit slot, every 2.7 s or more, with no time on it. Now the node polls
 * the ATtiny every 100 ms, notes when each new publish appears, and sends the
 * newest 1..3 publishes with the time the newest one appeared.
 *
 * Wire format: 10 bytes after the 36-byte NodePacket = 46 bytes, 341 ms at
 * SF11/BW500/CR4-7 (398 ms on the old SF9/BW125; 450 ms slot). The NodePacket CRC
 * still covers only its own 36 bytes and the base has always accepted
 * len >= 36, so an older base reads the packet exactly as before and ignores
 * the extra bytes. A newer base reads an older node's 36-byte packet with no
 * timed readings. Nothing else in the packet or beacon changes, so
 * PACKET_VERSION stays 5 and mixed fleets keep working.
 *
 * Timing: with no GPS running on the nodes (they power GPS down after the
 * first fix), the shared clock is the base's beacon. The node records the
 * moment each beacon finishes arriving (radio interrupt) and stamps readings in
 * ms after that beacon; the base records when it finished sending the same
 * beacon, so it can place every reading on its own clock to a few ms, plus the
 * +/-50 ms of the 100 ms poll. The base then maps its clock to GPS UTC.
 * If the node isn't synced to a beacon, it stamps readings in ms before the
 * start of its own transmission, and the base uses the arrival time minus the
 * packet's airtime instead.
 */

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#define WIND_EXT_TAG         0xA0   // high nibble marks the block
#define WIND_EXT_TAG_MASK    0xF0
#define WIND_EXT_SYNCED      0x08   // t_ms is ms after the beacon whose seq low byte is `beacon`
#define WIND_EXT_FAST_OK     0x04   // reserved: node can switch data rate (adaptive speed, later)
#define WIND_EXT_COUNT_MASK  0x03   // number of readings, 1..3
#define WIND_EXT_MAX         3
#define WIND_EXT_SPACING_MS  1000   // nominal ATtiny publish period

#pragma pack(push, 1)
struct WindExt {
    uint8_t tag;        // WIND_EXT_TAG | flags | count
    uint8_t beacon;     // low byte of the reference beacon's seq (synced only)
    int16_t t_ms;       // time the NEWEST reading appeared: ms after the beacon
                        // (synced) or ms relative to TX start, negative (unsynced)
    uint8_t r[WIND_EXT_MAX][2];   // newest first, one ATtiny publish apart:
                                  // [0] speed in 0.25 mph (0..63.75), [1] dir in 360/256 deg
};
#pragma pack(pop)
static_assert(sizeof(WindExt) == 10, "WindExt is a 10-byte wire format");

static inline uint8_t windExtCount(const WindExt& e) { return e.tag & WIND_EXT_COUNT_MASK; }
static inline bool    windExtSynced(const WindExt& e) { return (e.tag & WIND_EXT_SYNCED) != 0; }

static inline uint8_t windExtSpeedByte(float mph) {
    float v = mph * 4.0f + 0.5f;
    if (v <= 0) return 0;
    if (v >= 255) return 255;
    return (uint8_t)v;
}
static inline float windExtSpeed(uint8_t b) { return b * 0.25f; }
static inline uint8_t windExtDirByte(float deg) {
    while (deg < 0) deg += 360.0f;
    long v = (long)(deg * (256.0f / 360.0f) + 0.5f);
    return (uint8_t)(v & 0xFF);
}
static inline float windExtDir(uint8_t b) { return b * (360.0f / 256.0f); }

// Read the block that follows a NodePacket of `pktLen` bytes. False if the
// packet is a plain 36-byte one or the bytes don't look like a block.
static inline bool windExtParse(const uint8_t* buf, size_t len, size_t pktLen, WindExt& e) {
    if (len < pktLen + sizeof(WindExt)) return false;
    memcpy(&e, buf + pktLen, sizeof(WindExt));
    if ((e.tag & WIND_EXT_TAG_MASK) != WIND_EXT_TAG) return false;
    uint8_t n = windExtCount(e);
    return n >= 1 && n <= WIND_EXT_MAX;
}

// LoRa time on air in ms (explicit header, CRC on). cr is the denominator
// (5..8 for 4/5..4/8), as passed to RadioLib.
static inline uint32_t loraAirtimeMs(size_t len, uint8_t sf, float bwKhz, uint8_t cr, uint16_t preamble) {
    float tSym = (float)(1UL << sf) / bwKhz;                 // ms
    int de = (tSym >= 16.0f) ? 1 : 0;                        // low data rate optimize
    long num = 8L * (long)len - 4L * sf + 28 + 16;           // CRC on, explicit header (H=0)
    long den = 4L * (sf - 2 * de);
    long blocks = num > 0 ? (num + den - 1) / den : 0;
    float nSym = 8.0f + (float)(blocks * (cr));              // (CR+4) with CR=1..4 == cr 5..8
    float t = (preamble + 4.25f) * tSym + nSym * tSym;
    return (uint32_t)(t + 0.5f);
}

#endif // WIND_EXT_H
