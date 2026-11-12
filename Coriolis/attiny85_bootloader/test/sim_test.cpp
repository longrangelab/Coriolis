/*
 * End-to-end test of the ATtiny85 I2C update path, off the hardware.
 *
 * Runs the real bootloader and the real wind sensor sketch on simavr's
 * ATtiny85 and drives them from the real Node/lib/AttinyFlasher code through a
 * bit-level I2C master (100 kHz, ESP32's 13 ms clock-stretch limit).
 *
 * simavr 1.6 (Ubuntu/Debian package) has no USI, so a two-wire USI model is
 * included here (start/stop detect, 4-bit edge counter, SCL hold after start
 * and on overflow, open-drain SDA from the USIDR MSB latch, interrupts 13/14).
 * It also flags bus contention: a pin driven push-pull high while the bus is
 * pulled low. Its ATtiny85 core has no self-programming either, so SPM page
 * fill/erase/write is modelled too, with the datasheet's 4.5 ms busy time.
 *
 * Build/run: see Makefile in this folder (make test).
 */
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <simavr/sim_avr.h>
#include <simavr/sim_io.h>
#include <simavr/sim_interrupts.h>

#include "../../Node/lib/AttinyFlasher/AttinyFlasher.h"

// ---- ATtiny85 data-space addresses ----------------------------------------
enum : uint16_t { R_USICR = 0x2D, R_USISR = 0x2E, R_USIDR = 0x2F,
                  R_PINB = 0x36, R_DDRB = 0x37, R_PORTB = 0x38 };
enum { SIF = 7, OIF = 6, PF = 5 };
enum : uint16_t { R_SPMCSR = 0x57 };
#define AVR_IOCTL_FLASH_SPM AVR_IOCTL_DEF('f','s','p','m')

static avr_t* avr;
static avr_int_vector_t vStart, vOvf;

// bus state
static int  mSda = 1, mScl = 1;       // master: 1 = released
static int  bSda = 1, bScl = 1;       // resolved bus
static int  latch = 1;                // USI data-out latch (USIDR MSB)
static bool holdStart = false, holdOvf = false;
static long contention = 0;
static long stretchFails = 0;
static uint64_t maxStretch = 0;

static inline uint8_t& D(uint16_t a) { return avr->data[a]; }

// ---- SPM (self-programming) model -----------------------------------------
static uint8_t  spmBuf[64];
static uint64_t spmBusyUntil = 0;
static long     spmErases = 0, spmWrites = 0;

static int spmIoctl(avr_io_t*, uint32_t ctl, void*) {
    if (ctl != AVR_IOCTL_FLASH_SPM) return -1;
    uint8_t  cs = D(R_SPMCSR);
    uint16_t z  = D(30) | (D(31) << 8);
    uint16_t page = z & ~63;
    if (!(cs & 1)) return 0;
    if ((cs & 0x3F) == 0x01) {                       // fill temporary buffer
        spmBuf[z & 62] = D(0); spmBuf[(z & 62) + 1] = D(1);
        D(R_SPMCSR) &= ~1;
    } else if ((cs & 0x3F) == 0x03) {                // page erase
        memset(avr->flash + page, 0xFF, 64); spmErases++;
        spmBusyUntil = avr->cycle + 36000;
    } else if ((cs & 0x3F) == 0x05) {                // page write
        for (int i = 0; i < 64; i++) avr->flash[page + i] &= spmBuf[i];
        memset(spmBuf, 0xFF, 64); spmWrites++;
        spmBusyUntil = avr->cycle + 36000;
    } else if ((cs & 0x3F) == 0x11) {                // clear temporary buffer
        memset(spmBuf, 0xFF, 64);
        D(R_SPMCSR) &= ~0x11;
    } else {
        D(R_SPMCSR) &= ~0x3F;
    }
    return 0;
}
static avr_io_t spmIo;

// -1 released, 0 low, 1 push-pull high
static int slaveSda() {
    uint8_t ddr = D(R_DDRB), port = D(R_PORTB);
    bool tw = ((D(R_USICR) >> 4) & 3) >= 2;
    if (!(ddr & 1)) return -1;
    if (tw) return (!(port & 1) || !latch) ? 0 : -1;
    return (port & 1) ? 1 : 0;
}
static int slaveScl() {
    uint8_t ddr = D(R_DDRB), port = D(R_PORTB);
    bool tw = ((D(R_USICR) >> 4) & 3) >= 2;
    if (!(ddr & 4)) return -1;
    if (tw) {
        bool hold = (holdStart && (D(R_USISR) & (1 << SIF))) ||
                    (holdOvf && (D(R_USISR) & (1 << OIF)));
        return (!(port & 4) || hold) ? 0 : -1;
    }
    return (port & 4) ? 1 : 0;
}

static void counterTick() {
    uint8_t sr = D(R_USISR);
    uint8_t cnt = (sr & 0x0F) + 1;
    if (cnt > 15) {
        cnt = 0;
        sr |= (1 << OIF);
        D(R_USISR) = (sr & 0xF0) | cnt;
        avr_raise_interrupt(avr, &vOvf);
        return;
    }
    D(R_USISR) = (sr & 0xF0) | cnt;
}

static void update() {
    if (spmBusyUntil && avr->cycle >= spmBusyUntil) { spmBusyUntil = 0; D(R_SPMCSR) &= ~0x07; }
    uint8_t cr = D(R_USICR);
    int wm = (cr >> 4) & 3, cs = (cr >> 2) & 3;
    bool tw = wm >= 2;
    if (!(D(R_USISR) & (1 << SIF))) holdStart = false;
    if (!(D(R_USISR) & (1 << OIF))) holdOvf = false;

    for (int settle = 0; settle < 3; settle++) {
        int ss = slaveSda(), sc = slaveScl();
        int nSda = (mSda == 0 || ss == 0) ? 0 : 1;
        int nScl = (mScl == 0 || sc == 0) ? 0 : 1;
        if ((ss == 1 && mSda == 0) || (sc == 1 && mScl == 0)) { if (getenv("SIMDBG") && contention < 6) printf("  contention %s pc=0x%04x DDRB=%02x PORTB=%02x USICR=%02x\n", ss == 1 ? "SDA" : "SCL", avr->pc, D(R_DDRB), D(R_PORTB), D(R_USICR)); contention++; }
        if (nSda == bSda && nScl == bScl) break;

        int oSda = bSda, oScl = bScl;
        bSda = nSda; bScl = nScl;

        if (tw && oScl && nScl && oSda != nSda) {
            if (!nSda) { D(R_USISR) |= (1 << SIF); avr_raise_interrupt(avr, &vStart); }
            else       { D(R_USISR) |= (1 << PF); }
        }
        if (wm && cs >= 2 && oScl != nScl) {
            if (nScl) D(R_USIDR) = (uint8_t)((D(R_USIDR) << 1) | bSda);
            else      latch = D(R_USIDR) >> 7;
            counterTick();
        }
        if (tw && oScl && !nScl) {
            if (D(R_USISR) & (1 << SIF)) holdStart = true;
            if (wm == 3 && (D(R_USISR) & (1 << OIF))) holdOvf = true;
        }
    }
    // level-triggered interrupt flags
    if ((D(R_USICR) & 0x80) && (D(R_USISR) & 0x80) && !vStart.pending) avr_raise_interrupt(avr, &vStart);
    if ((D(R_USICR) & 0x40) && (D(R_USISR) & 0x40) && !vOvf.pending)   avr_raise_interrupt(avr, &vOvf);
}

static uint8_t pinbRead(avr_t*, avr_io_addr_t, void*) {
    uint8_t ddr = D(R_DDRB), port = D(R_PORTB);
    uint8_t v = 0x3A;                         // PB1 LED, PB3 vane, PB4 anemo (pull-up), PB5 reset
    v = (v & ~ddr) | (port & ddr);
    v = (v & ~0x05) | (bSda ? 1 : 0) | (bScl ? 4 : 0);
    D(R_PINB) = v;
    return v;
}
static void usicrWrite(avr_t*, avr_io_addr_t, uint8_t v, void*) {
    D(R_USICR) = v & 0xFC;                    // strobes not modelled
}
static void usisrWrite(avr_t*, avr_io_addr_t, uint8_t v, void*) {
    uint8_t sr = D(R_USISR);
    if (v & (1 << SIF)) { sr &= ~(1 << SIF); holdStart = false; avr_clear_interrupt(avr, &vStart); }
    if (v & (1 << OIF)) { sr &= ~(1 << OIF); holdOvf = false;   avr_clear_interrupt(avr, &vOvf); }
    if (v & (1 << PF))  sr &= ~(1 << PF);
    D(R_USISR) = (sr & 0xE0) | (v & 0x0F);
}
static void usidrWrite(avr_t*, avr_io_addr_t, uint8_t v, void*) {
    D(R_USIDR) = v;
    if (!bScl) latch = v >> 7;                // latch is transparent while SCL is low
}

static void step(uint64_t cycles) {
    uint64_t end = avr->cycle + cycles;
    while (avr->cycle < end) {
        int st = avr_run(avr);
        if (st == cpu_Crashed || st == cpu_Done) { printf("  !! AVR stopped (state %d) at pc 0x%04x\n", st, avr->pc); exit(2); }
        update();
    }
}

// ---- bit-level I2C master ---------------------------------------------------
static const uint64_t HALF = 40;              // 100 kHz at 8 MHz
static const uint64_t STRETCH_LIMIT = 104857; // ESP32: 0xFFFFF APB cycles = 13.1 ms

static bool releaseScl() {
    mScl = 1; update();
    uint64_t t0 = avr->cycle;
    while (!bScl) {
        step(1);
        if (avr->cycle - t0 > STRETCH_LIMIT) { if (getenv("SIMDBG") && stretchFails < 3) printf("  stretch stuck pc=0x%04x SPMCSR=%02x USISR=%02x USICR=%02x DDRB=%02x PORTB=%02x\n", avr->pc, D(0x57), D(R_USISR), D(R_USICR), D(R_DDRB), D(R_PORTB)); stretchFails++; return false; }
    }
    if (avr->cycle - t0 > maxStretch) maxStretch = avr->cycle - t0;
    return true;
}
static bool sendBit(int b) { mSda = b; step(HALF); if (!releaseScl()) return false; step(HALF); mScl = 0; step(HALF); return true; }
static int recvBit() {
    mSda = 1; step(HALF);
    if (!releaseScl()) return -1;
    step(HALF / 2); int b = bSda; step(HALF / 2);
    mScl = 0; step(HALF);
    return b;
}
static void startCond() {
    if (!mScl) { mSda = 1; step(HALF); releaseScl(); step(HALF); }
    mSda = 0; step(HALF); mScl = 0; step(HALF);
}
static void stopCond() { mSda = 0; step(HALF); releaseScl(); step(HALF); mSda = 1; step(HALF * 4); }
static bool sendByte(uint8_t v) {
    for (int i = 7; i >= 0; i--) if (!sendBit((v >> i) & 1)) return false;
    return recvBit() == 0;
}
static uint8_t recvByte(bool ack) {
    uint8_t v = 0;
    for (int i = 0; i < 8; i++) v = (uint8_t)((v << 1) | (recvBit() & 1));
    sendBit(ack ? 0 : 1);
    return v;
}

struct SimBus {
    long failAfterPages = -1;                // simulate the link dying mid-update
    long pages = 0;
    bool write(uint8_t a, const uint8_t* d, uint16_t n) {
        if (n == 4 + attiny::PAGE_SIZE && failAfterPages >= 0 && pages++ >= failAfterPages) return false;
        startCond();
        bool ok = sendByte(a << 1);
        for (uint16_t i = 0; ok && i < n; i++) ok = sendByte(d[i]);
        stopCond();
        return ok;
    }
    bool writeRead(uint8_t a, const uint8_t* w, uint16_t wn, uint8_t* r, uint16_t rn) {
        if (failAfterPages >= 0 && pages >= failAfterPages) return false;
        startCond();
        bool ok = sendByte(a << 1);
        for (uint16_t i = 0; ok && i < wn; i++) ok = sendByte(w[i]);
        if (ok) { startCond(); ok = sendByte((a << 1) | 1); }
        for (uint16_t i = 0; ok && i < rn; i++) r[i] = recvByte(i + 1 < rn);
        stopCond();
        return ok;
    }
    bool read(uint8_t a, uint8_t* r, uint16_t rn) {
        startCond();
        bool ok = sendByte((a << 1) | 1);
        for (uint16_t i = 0; ok && i < rn; i++) r[i] = recvByte(i + 1 < rn);
        stopCond();
        return ok;
    }
    void delayMs(uint32_t ms) { step((uint64_t)ms * 8000); }
};

// ---- helpers ------------------------------------------------------------------
static std::vector<uint8_t> loadHex(const char* path, uint16_t* used) {
    std::vector<uint8_t> img(8192, 0xFF);
    FILE* f = fopen(path, "r");
    if (!f) { printf("cannot open %s\n", path); exit(1); }
    char line[600];
    *used = 0;
    while (fgets(line, sizeof line, f)) {
        auto h = attiny::parseIntelHexLine(line, img.data(), 8192, used);
        if (h == attiny::HEX_BAD || h == attiny::HEX_RANGE) { printf("bad hex %s\n", path); exit(1); }
        if (h == attiny::HEX_EOF) break;
    }
    fclose(f);
    return img;
}

static int fails = 0;
static void check(bool ok, const char* what) {
    printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) fails++;
}

static void powerCycle() {
    avr_reset(avr);
    mSda = mScl = 1; bSda = bScl = 1; latch = 1; holdStart = holdOvf = false;
}

static bool appAnswers(SimBus& bus, uint8_t* out12 = nullptr) {
    uint8_t b[12];
    bool ok = bus.read(attiny::APP_ADDR, b, 12);
    if (ok && out12) memcpy(out12, b, 12);
    return ok;
}
static bool bootAnswers(SimBus& bus) {
    uint8_t info[8];
    attiny::Flasher<SimBus> f(bus);
    return f.readChipInfo(info);
}
static bool flashMatches(const std::vector<uint8_t>& img, uint16_t len) {
    // Compare the simulated chip's flash with the image, skipping the two
    // vectors twiboot patches (reset and EE_RDY), which are checked separately.
    for (uint16_t i = 0; i < len; i++) {
        if (i < 2 || (i >= 12 && i < 14)) continue;
        if (avr->flash[i] != img[i]) return false;
    }
    return true;
}
// Wait while the node keeps talking to other chips (PMU at 0x34) every 50 ms,
// as the real node does, so a pin left driven high shows up as contention.
static void busyWait(SimBus& bus, uint32_t ms) {
    for (uint32_t t = 0; t < ms; t += 50) {
        uint8_t x[2];
        bus.read(0x34, x, 2);
        bus.delayMs(50);
    }
}
static uint16_t word(uint16_t byteAddr) { return avr->flash[byteAddr] | (avr->flash[byteAddr + 1] << 8); }

int main(int argc, char** argv) {
    if (argc < 4) { printf("usage: %s twiboot.hex app_v1.hex app_v2.hex\n", argv[0]); return 1; }
    uint16_t bootLen, v1Len, v2Len;
    auto boot = loadHex(argv[1], &bootLen);
    auto v1 = loadHex(argv[2], &v1Len);
    auto v2 = loadHex(argv[3], &v2Len);
    printf("bootloader %u bytes (ends 0x%04x), app v1 %u bytes, app v2 %u bytes\n",
           bootLen - attiny::BOOT_START, bootLen, v1Len, v2Len);

    avr = avr_make_mcu_by_name("attiny85");
    if (!avr) { printf("no attiny85 core\n"); return 1; }
    avr_init(avr);
    avr->frequency = 8000000;
    avr->log = 0;

    // USI model hooks
    avr->io[AVR_DATA_TO_IO(R_PINB)].r.c = pinbRead;  avr->io[AVR_DATA_TO_IO(R_PINB)].r.param = nullptr;
    avr->io[AVR_DATA_TO_IO(R_USICR)].w.c = usicrWrite; avr->io[AVR_DATA_TO_IO(R_USICR)].w.param = nullptr;
    avr->io[AVR_DATA_TO_IO(R_USISR)].w.c = usisrWrite; avr->io[AVR_DATA_TO_IO(R_USISR)].w.param = nullptr;
    avr->io[AVR_DATA_TO_IO(R_USIDR)].w.c = usidrWrite; avr->io[AVR_DATA_TO_IO(R_USIDR)].w.param = nullptr;
    memset(&vStart, 0, sizeof vStart); memset(&vOvf, 0, sizeof vOvf);
    vStart.vector = 13; vStart.enable = AVR_IO_REGBIT(R_USICR, 7); vStart.raised = AVR_IO_REGBIT(R_USISR, SIF); vStart.raise_sticky = 1;
    vOvf.vector   = 14; vOvf.enable   = AVR_IO_REGBIT(R_USICR, 6); vOvf.raised   = AVR_IO_REGBIT(R_USISR, OIF); vOvf.raise_sticky = 1;
    avr_register_vector(avr, &vStart);
    avr_register_vector(avr, &vOvf);
    memset(&spmIo, 0, sizeof spmIo);
    spmIo.kind = "spm-model"; spmIo.ioctl = spmIoctl;
    avr_register_io(avr, &spmIo);
    memset(spmBuf, 0xFF, 64);

    // One-time ISP step: bootloader only, rest of flash erased.
    memcpy(avr->flash + attiny::BOOT_START, boot.data() + attiny::BOOT_START, bootLen - attiny::BOOT_START);
    powerCycle();

    SimBus bus;
    attiny::Flasher<SimBus> fl(bus);

    printf("\n1) Fresh chip, bootloader only\n");
    step(8000 * 200);
    check(bootAnswers(bus), "bootloader answers at 0x43 after power-on");
    {   // the node talks to other chips on the same bus; the bootloader must not answer for them
        uint8_t x[2];
        check(!bus.read(0x34, x, 2), "bootloader ignores another address (0x34, the PMU)");
    }

    printf("\n2) First sketch over I2C\n");
    check(fl.enterBootloader(), "enterBootloader");
    attiny::Result r = fl.program(v1.data(), v1Len);
    printf("     program: %s, longest SCL stretch %.2f ms\n", attiny::resultText(r), maxStretch / 8000.0);
    check(r == attiny::OK, "program v1");
    check(flashMatches(v1, v1Len), "flash holds v1");
    check(word(0) == attiny::rjmpOpcode(0, attiny::BOOT_START), "reset vector jumps to the bootloader at 0x1C00");
    check(attiny::rjmpTarget(6, word(12)) == attiny::rjmpTarget(0, v1[0] | (v1[1] << 8)),
          "EE_RDY slot jumps to the sketch's own start");
    check(fl.startApp(), "start sketch");
    busyWait(bus, 2500);                       // sketch blinks ~1.7 s before starting I2C
    uint8_t wd[12];
    check(appAnswers(bus, wd), "sketch answers at 0x42 with 12 bytes");
    check(!bootAnswers(bus), "bootloader is gone from 0x43");

    printf("\n3) Power cycle runs the bootloader 1 s, then the sketch\n");
    powerCycle();
    step(8000 * 500);
    {   // a bare read does not send a command, so it does not stop the timeout
        uint8_t x;
        check(bus.read(attiny::BOOT_ADDR, &x, 1), "0x43 answers during the boot window");
    }
    busyWait(bus, 3500);
    check(appAnswers(bus), "sketch runs after the window");

    printf("\n4) Update a running sketch to v2 (enter command + watchdog reset)\n");
    uint64_t t0 = avr->cycle;
    bool entered = fl.enterBootloader();
    printf("     sketch -> bootloader took %.0f ms\n", (avr->cycle - t0) / 8000.0);
    t0 = avr->cycle;
    r = entered ? fl.program(v2.data(), v2Len) : attiny::NO_BOOTLOADER;
    if (r == attiny::OK) fl.startApp();
    printf("     program + verify took %.2f s\n", (avr->cycle - t0) / 8e6);
    check(r == attiny::OK, "update to v2");
    check(flashMatches(v2, v2Len), "flash holds v2");
    busyWait(bus, 2500);
    check(appAnswers(bus), "v2 answers at 0x42");

    printf("\n5) Interrupted update, then power loss\n");
    bus.failAfterPages = 40; bus.pages = 0;   // link dies after 40 pages
    r = fl.update(v1.data(), v1Len);
    bus.failAfterPages = -1;
    printf("     interrupted update: %s\n", attiny::resultText(r));
    check(r != attiny::OK, "update reports failure");
    powerCycle();
    busyWait(bus, 5000);                      // well past the 1 s timeout
    check(!appAnswers(bus), "half-written sketch is NOT started");
    check(bootAnswers(bus), "chip sits in the bootloader at 0x43");
    r = fl.update(v1.data(), v1Len);
    check(r == attiny::OK, "retry succeeds");
    busyWait(bus, 2500);
    check(appAnswers(bus), "v1 runs after the retry");

    printf("\n6) Image checks\n");
    std::vector<uint8_t> big(8192, 0x00);
    check(fl.enterBootloader() && fl.program(big.data(), 7200) == attiny::IMAGE_TOO_BIG, "rejects a 7200-byte image");
    std::vector<uint8_t> ee(v1);
    uint16_t vec = attiny::rjmpOpcode(6, 0x100);   // EE_RDY pointing at real code
    ee[12] = vec & 0xFF; ee[13] = vec >> 8;
    check(fl.program(ee.data(), v1Len) == attiny::IMAGE_USES_EE_RDY, "rejects a sketch that uses EE_RDY");
    check(fl.matches(v1.data(), v1Len), "flash unchanged by the rejected images");
    fl.startApp();
    busyWait(bus, 2500);

    printf("\nflash page erases %ld, page writes %ld\n", spmErases, spmWrites);
    printf("bus contention events: %ld, ESP32 stretch timeouts: %ld\n", contention, stretchFails);
    check(contention == 0, "no push-pull fights on SDA/SCL");
    check(stretchFails == 0, "no clock stretch over 13.1 ms");
    printf("\n%s (%d failed)\n", fails ? "FAILED" : "ALL PASSED", fails);
    return fails ? 1 : 0;
}
