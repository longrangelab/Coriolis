// Host test for the node's bit-banged ATtiny read (Node/lib/SoftI2CRead).
// A simulated I2C slave, shaped like the ATtiny's USI + TinyWireS (ACKs its
// address, stretches SCL after each byte, sends a 12-byte frame), sits on a
// simulated open-drain bus.
//   g++ -std=c++17 -O2 -DSI2C_HOST_TEST -I../../../Node/lib/SoftI2CRead test_softi2c.cpp -o test_softi2c && ./test_softi2c
#include <cstdio>
#include <cstdint>
#include <cstring>

static uint32_t simUs = 0;
static int mSda = 1, mScl = 1;          // master drive (1 = released)
struct Slave;
static Slave* slave = nullptr;
static int busSda();
static int busScl();
static void onLines();
static void tickSlave();

#define SDA_PIN 21
#define SCL_PIN 22
#define SI2C_SET(pin, lvl)  do { if ((pin) == SDA_PIN) mSda = (lvl); else mScl = (lvl); onLines(); } while (0)
#define SI2C_GET(pin)       ((pin) == SDA_PIN ? busSda() : busScl())
#define SI2C_DELAY_US(us)   do { simUs += (us); tickSlave(); } while (0)
#define SI2C_MICROS()       (simUs)
#include "SoftI2CRead.h"

struct Slave {
    uint8_t addr = 0x42;
    uint8_t frame[12];
    int sda = 1, scl = 1;               // slave drive
    enum { IDLE, ADDR, ACKING, TX, TXACK, WAITSTOP } st = IDLE;
    int bits = 0, bitn = 0, idx = 0;
    uint8_t shift = 0;
    bool masterAck = false;
    uint32_t stretchUs = 40;            // USI interrupt latency per byte
    uint32_t releaseAt = 0;
    bool hangOnByte = false; int hangAt = -1;   // hold SCL forever at this byte
    int prevSda = 1, prevScl = 1;
    int starts = 0, stops = 0;

    void stretch() { scl = 0; releaseAt = simUs + stretchUs; }
    void tick() { if (scl == 0 && !(hangOnByte && idx == hangAt) && (int32_t)(simUs - releaseAt) >= 0) scl = 1; }
    void driveBit() { sda = (frame[idx] >> bitn) & 1; }

    void lines(int s, int c) {
        if (c && prevScl && s != prevSda) {                 // SDA moved with SCL high
            if (!s) { st = ADDR; bits = 0; shift = 0; sda = 1; starts++; }
            else    { st = IDLE; sda = 1; stops++; }
        } else if (c && !prevScl) {                          // SCL rising
            if (st == ADDR) { shift = (uint8_t)((shift << 1) | s); bits++; }
            else if (st == TXACK) masterAck = (s == 0);
        } else if (!c && prevScl) {                          // SCL falling
            switch (st) {
            case ADDR:
                if (bits == 8) {
                    if ((shift >> 1) == addr && (shift & 1)) { sda = 0; st = ACKING; }
                    else { st = WAITSTOP; sda = 1; }
                }
                break;
            case ACKING:
                idx = 0; bitn = 7; st = TX; driveBit(); stretch(); break;
            case TX:
                if (--bitn >= 0) driveBit(); else { sda = 1; st = TXACK; }
                break;
            case TXACK:
                if (masterAck && idx + 1 < 12) { idx++; bitn = 7; st = TX; driveBit(); stretch(); }
                else { sda = 1; st = WAITSTOP; }
                break;
            default: break;
            }
        }
        prevSda = s; prevScl = c;
    }
};

static int busSda() { return mSda & (slave ? slave->sda : 1); }
static int busScl() { return mScl & (slave ? slave->scl : 1); }
static void onLines() { if (slave) slave->lines(busSda(), busScl()); }
static void tickSlave() { if (slave) { int before = busScl(); slave->tick(); if (busScl() != before) onLines(); } }

static int fails = 0, passes = 0;
#define CHECK(c, ...) do { if (c) passes++; else { fails++; printf("FAIL line %d: ", __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

static void reset(Slave& s) { slave = &s; mSda = mScl = 1; s.prevSda = s.prevScl = 1; }

int main() {
    SoftI2CRead bus(SDA_PIN, SCL_PIN);
    uint8_t buf[12];

    // 1) normal read, with the ATtiny stretching 40 us after every byte
    Slave s; for (int i = 0; i < 12; i++) s.frame[i] = (uint8_t)(0xA5 ^ (i * 37));
    reset(s);
    uint32_t t0 = simUs;
    SoftI2CResult r = bus.read(0x42, buf, 12);
    CHECK(r == SI2C_OK, "result %d", r);
    CHECK(memcmp(buf, s.frame, 12) == 0, "frame mismatch");
    CHECK(s.starts == 1 && s.stops == 1 && busSda() && busScl(), "one START, one STOP, bus released");
    printf("normal read: %u us\n", simUs - t0);
    CHECK(simUs - t0 < 3000, "read took %u us", simUs - t0);

    // 2) back-to-back reads stay in step
    for (int k = 0; k < 50; k++) {
        for (int i = 0; i < 12; i++) s.frame[i] = (uint8_t)(k * 13 + i);
        r = bus.read(0x42, buf, 12);
        if (r != SI2C_OK || memcmp(buf, s.frame, 12)) { CHECK(false, "read %d failed (%d)", k, r); break; }
    }
    passes++;

    // 3) nobody at the address -> quick NACK
    Slave other; other.addr = 0x10; reset(other);
    t0 = simUs;
    r = bus.read(0x42, buf, 12);
    CHECK(r == SI2C_NACK, "absent device gives NACK (%d)", r);
    CHECK(simUs - t0 < 500, "NACK took %u us", simUs - t0);
    CHECK(busSda() && busScl(), "bus released after NACK");

    // 4) ATtiny holds SCL low mid-frame and never lets go -> gives up in ~3 ms, not 1 s
    Slave h; memset(h.frame, 0x55, 12); h.hangOnByte = true; h.hangAt = 5; reset(h);
    t0 = simUs;
    r = bus.read(0x42, buf, 12);
    uint32_t took = simUs - t0;
    CHECK(r == SI2C_STRETCH, "stuck clock gives SI2C_STRETCH (%d)", r);
    CHECK(took < 3 * SI2C_STRETCH_US, "stuck read took %u us", took);
    printf("stuck read gave up after %u us\n", took);

    // 5) once it lets go, the next read works again
    h.hangOnByte = false; h.scl = 1; onLines();      // it carries on mid-byte, as a USI would
    r = bus.read(0x42, buf, 12);
    CHECK(r == SI2C_OK && memcmp(buf, h.frame, 12) == 0, "recovers after a stuck read (%d)", r);

    // 6) slave left driving SDA low (read cut short) -> clearBus frees it
    Slave c; for (int i = 0; i < 12; i++) c.frame[i] = 0x00; reset(c);
    c.st = Slave::TX; c.idx = 3; c.bitn = 7; c.sda = 0;          // mid-byte, driving a 0
    r = bus.read(0x42, buf, 12);
    CHECK(r == SI2C_OK, "read after a cut-short frame (%d)", r);

    printf("%d passed, %d failed\n", passes, fails);
    return fails ? 1 : 0;
}
