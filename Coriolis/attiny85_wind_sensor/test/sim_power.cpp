/*
 * Off-target test of the wind sketch's power saving (sleep on the node's
 * command, wake on I2C), on simavr's ATtiny85.
 *
 * Runs the real sketch (a .hex built from ../attiny85_wind_sensor.ino) and
 * drives it like the node does: a bit-level I2C master at 100 kHz that reads
 * the 12-byte frame at 0x42, writes {0x02, 0x00} before the node deep-sleeps,
 * and talks to the power chip (0x34) when it wakes. Anemometer pulses go in on
 * PB4 and a fixed vane voltage on PB3.
 *
 * The two-wire USI model is the one in ../../attiny85_bootloader/test
 * (simavr 1.6 has no USI). simavr also does not stop the timers in power-down,
 * which the real chip does, so while the sketch sleeps in power-down this test
 * masks the timer 0 interrupt the way the hardware would; everything that can
 * still wake the real chip (USI start condition, pin change, watchdog) stays.
 *
 * Build/run (Debian/Ubuntu: apt install libsimavr-dev libelf-dev):
 *   make test HEX=sketch.hex [MPH_PER_HZ=1.492]
 */
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <simavr/sim_avr.h>
#include <simavr/sim_io.h>
#include <simavr/sim_irq.h>
#include <simavr/sim_interrupts.h>
#include <simavr/sim_hex.h>
#include <simavr/sim_cycle_timers.h>
#include <simavr/avr_ioport.h>
#include <simavr/avr_adc.h>

// ---- ATtiny85 data-space addresses ----------------------------------------
enum : uint16_t { R_USICR = 0x2D, R_USISR = 0x2E, R_USIDR = 0x2F,
                  R_PINB = 0x36, R_DDRB = 0x37, R_PORTB = 0x38,
                  R_PCMSK = 0x35, R_WDTCR = 0x41, R_ADCSRA = 0x26,
                  R_MCUCR = 0x55, R_TIMSK = 0x59 };
enum { SIF = 7, OIF = 6, PF = 5 };

static avr_t* avr;
static avr_int_vector_t vStart, vOvf;

static int  mSda = 1, mScl = 1;       // master: 1 = released
static int  bSda = 1, bScl = 1;       // resolved bus
static int  latch = 1;                // USI data-out latch (USIDR MSB)
static bool holdStart = false, holdOvf = false;
static long contention = 0;
static int  anemoLevel = 1;           // PB4: reed open = pulled high

static inline uint8_t& D(uint16_t a) { return avr->data[a]; }

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
    uint8_t v = 0x2A | (anemoLevel ? 0x10 : 0);   // PB1 LED, PB3 vane, PB4 anemo, PB5 reset
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


// ---- time keeping ------------------------------------------------------------
static const uint64_t CPS = 8000000;           // cycles per second
static uint64_t runCycles = 0, sleepCycles = 0, ledCycles = 0;
static long     pdEntries = 0, pdWakes = 0;     // power-down sleeps entered / left
static bool     inPowerDown = false;
static uint8_t  savedTimsk = 0;
static double   pulseHz = 0;                   // anemometer rate, 0 = calm
static uint64_t pulseCount = 0;
static avr_irq_t* pb4Irq;

static bool powerDownMode() { return ((D(R_MCUCR) >> 3) & 3) == 2; }

static void anemometer() {
    int lvl = 1;
    if (pulseHz > 0) {
        uint64_t period = (uint64_t)(CPS / pulseHz);
        lvl = (avr->cycle % period) < period / 10 ? 0 : 1;   // closed 10% of a turn
    }
    if (lvl != anemoLevel) {
        anemoLevel = lvl;
        if (!lvl) pulseCount++;
        avr_raise_irq(pb4Irq, lvl);
    }
}

static void step(uint64_t cycles) {
    uint64_t end = avr->cycle + cycles;
    while (avr->cycle < end) {
        uint64_t c0 = avr->cycle;
        bool wasSleeping = avr->state == cpu_Sleeping;
        int st = avr_run(avr);
        if (st == cpu_Crashed || st == cpu_Done) { printf("  !! AVR stopped (state %d) at pc 0x%04x\n", st, avr->pc); exit(2); }
        uint64_t dc = avr->cycle - c0;
        (wasSleeping ? sleepCycles : runCycles) += dc;
        if ((D(R_DDRB) & 2) && (D(R_PORTB) & 2)) ledCycles += dc;
        // power-down: the real chip's timers stop, so timer 0 can't wake it
        bool pd = avr->state == cpu_Sleeping && powerDownMode();
        if (pd && !inPowerDown) { inPowerDown = true; pdEntries++; savedTimsk = D(R_TIMSK); D(R_TIMSK) = 0; }
        if (!pd && inPowerDown && avr->state != cpu_Sleeping) { inPowerDown = false; pdWakes++; D(R_TIMSK) = savedTimsk; }
        anemometer();
        update();
    }
}

// ---- bit-level I2C master ---------------------------------------------------
static const uint64_t HALF = 40;              // 100 kHz at 8 MHz
static const uint64_t STRETCH_LIMIT = 104857; // ESP32: 13.1 ms
static uint64_t maxStretch = 0;
static bool releaseScl() {
    mScl = 1; update();
    uint64_t t0 = avr->cycle;
    while (!bScl) { step(1); if (avr->cycle - t0 > STRETCH_LIMIT) return false; }
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
static bool i2cWrite(uint8_t a, const uint8_t* d, int n) {
    startCond();
    bool ok = sendByte(a << 1);
    for (int i = 0; ok && i < n; i++) ok = sendByte(d[i]);
    stopCond();
    return ok;
}
static bool i2cRead(uint8_t a, uint8_t* r, int n) {
    startCond();
    bool ok = sendByte((a << 1) | 1);
    for (int i = 0; ok && i < n; i++) r[i] = recvByte(i + 1 < n);
    stopCond();
    return ok;
}

struct Frame { bool ok; float mph; float dir; float dirSd; uint8_t seq; };
// A read is retried like the node does. Misses are counted: TinyWireS takes a
// START for a STOP when another interrupt (the wind pulse ISR, the millis
// timer) delays its start ISR past the first address bit, which is a 1 for
// 0x42. That race is in the library and predates this sketch's sleep code.
static long readMisses = 0, readsTotal = 0;
static Frame readFrame() {
    uint8_t b[12];
    Frame f{};
    for (int tries = 0; tries < 3 && !f.ok; tries++) {
        readsTotal++;
        f.ok = i2cRead(0x42, b, 12);
        if (!f.ok) readMisses++;
    }
    f.mph = (b[0] | (b[1] << 8)) / 100.0f;
    f.dir = (b[4] | (b[5] << 8)) / 10.0f;
    f.dirSd = (b[6] | (b[7] << 8)) / 10.0f;
    f.seq = (uint8_t)((b[10] | (b[11] << 8)) >> 10);
    return f;
}
static void ms(double t) { step((uint64_t)(t * CPS / 1000)); }

static int fails = 0;
static void check(bool ok, const char* what) {
    printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) fails++;
}
static void resetCounters() { runCycles = sleepCycles = ledCycles = 0; }
static double pct(uint64_t a) { uint64_t t = runCycles + sleepCycles; return t ? 100.0 * a / t : 0; }

static uint8_t pmu2[2];
// simavr skips a sleeping core straight to its next timer event, so an
// interrupt raised by the bus model would be seen up to 2 ms late. A timer
// every few cycles keeps each sleep step as short as the real wake-up.
static avr_cycle_count_t tickle(avr_t*, avr_cycle_count_t when, void*) { return when + 4; }
static void noSleepCallback(avr_t*, avr_cycle_count_t) {}   // don't sleep in real time

int main(int argc, char** argv) {
    if (argc < 2) { printf("usage: %s sketch.hex [mph_per_hz]\n", argv[0]); return 1; }
    double mphPerHz = argc > 2 ? atof(argv[2]) : 1.492;

    avr = avr_make_mcu_by_name("attiny85");
    if (!avr) { printf("no attiny85 core\n"); return 1; }
    avr_init(avr);
    avr->frequency = CPS;
    avr->vcc = avr->avcc = avr->aref = 3300;
    avr->log = 0;
    avr->sleep = noSleepCallback;
    avr_cycle_timer_register(avr, 4, tickle, nullptr);

    ihex_chunk_p chunks = nullptr;
    int n = read_ihex_chunks(argv[1], &chunks);
    if (n <= 0) { printf("cannot read %s\n", argv[1]); return 1; }
    uint32_t used = 0;
    for (int i = 0; i < n; i++) {
        memcpy(avr->flash + chunks[i].baseaddr, chunks[i].data, chunks[i].size);
        if (chunks[i].baseaddr + chunks[i].size > used) used = chunks[i].baseaddr + chunks[i].size;
    }
    printf("sketch %u bytes (limit 7168 with the I2C bootloader)\n", used);
    check(used <= 7168, "fits below the bootloader");

    avr->io[AVR_DATA_TO_IO(R_PINB)].r.c = pinbRead;  avr->io[AVR_DATA_TO_IO(R_PINB)].r.param = nullptr;
    avr->io[AVR_DATA_TO_IO(R_USICR)].w.c = usicrWrite; avr->io[AVR_DATA_TO_IO(R_USICR)].w.param = nullptr;
    avr->io[AVR_DATA_TO_IO(R_USISR)].w.c = usisrWrite; avr->io[AVR_DATA_TO_IO(R_USISR)].w.param = nullptr;
    avr->io[AVR_DATA_TO_IO(R_USIDR)].w.c = usidrWrite; avr->io[AVR_DATA_TO_IO(R_USIDR)].w.param = nullptr;
    memset(&vStart, 0, sizeof vStart); memset(&vOvf, 0, sizeof vOvf);
    vStart.vector = 13; vStart.enable = AVR_IO_REGBIT(R_USICR, 7); vStart.raised = AVR_IO_REGBIT(R_USISR, SIF); vStart.raise_sticky = 1;
    vOvf.vector   = 14; vOvf.enable   = AVR_IO_REGBIT(R_USICR, 6); vOvf.raised   = AVR_IO_REGBIT(R_USISR, OIF); vOvf.raise_sticky = 1;
    avr_register_vector(avr, &vStart);
    avr_register_vector(avr, &vOvf);
    pb4Irq = avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ('B'), 4);
    avr_irq_t* adc3 = avr_io_getirq(avr, AVR_IOCTL_ADC_GETIRQ, ADC_IRQ_ADC3);
    avr_raise_irq(adc3, 916);              // SparkFun vane at "0 deg" (ADC ~284)

    const double hz = 5.0;                 // 5 pulses/s
    const float  want = (float)(hz * mphPerHz);

    printf("\n1) Awake, %.0f Hz wind (%.2f mph expected)\n", hz, want);
    pulseHz = hz;
    ms(3000);                              // boot blinks, then I2C starts
    resetCounters();
    ms(4000);
    Frame f = readFrame();
    printf("     frame: %.2f mph, dir %.1f, seq %u\n", f.mph, f.dir, f.seq);
    check(f.ok && f.seq != 0, "answers at 0x42 with data");
    check(f.mph > want * 0.9f && f.mph < want * 1.1f, "speed matches the pulse rate");
    const float awakeDir = f.dir;          // fixed vane voltage: this never changes
    resetCounters();
    uint8_t s0 = readFrame().seq;
    for (int i = 0; i < 50; i++) { ms(100); readFrame(); }           // node polls every 100 ms
    uint8_t s1 = readFrame().seq;
    int pubs = (s1 - s0 + 63) % 63;
    printf("     CPU running %.1f%% of the time, LED lit %.1f%%, %d publishes in ~5.2 s\n",
           pct(runCycles), pct(ledCycles), pubs);
    check(pubs >= 4 && pubs <= 6, "publishes once a second while polled at 10 Hz");
    check(pct(runCycles) < 25.0, "CPU idles between ticks (was 100%)");
    check(pct(ledCycles) < 2.0, "LED lit only for the heartbeat blip (was 50%)");
    check(contention == 0, "no bus contention");

    for (int round = 1; round <= 3; round++) {
        printf("\n%d) Node deep-sleeps 20 s, wind keeps blowing\n", round + 1);
        uint8_t before = readFrame().seq;
        uint8_t cmd[2] = {0x02, 0x00};
        check(i2cWrite(0x42, cmd, 2), "sleep command acked");
        ms(10);
        check(inPowerDown, "chip is in power-down");
        check(!(D(R_PORTB) & 2), "LED off");
        check(!(D(R_WDTCR) & 0x08), "watchdog off");
        check(!(D(R_ADCSRA) & 0x80), "ADC off");
        check(!(D(R_PCMSK) & 0x10), "wind pulses can't wake it");
        long wakes0 = pdWakes;
        resetCounters();
        ms(20000);
        printf("     asleep: CPU running %.4f%% of the time, %ld wakes\n", pct(runCycles), pdWakes - wakes0);
        check(pdWakes == wakes0 && inPowerDown, "stayed asleep the whole 20 s");

        printf("     node wakes: power chip read, then the ATtiny\n");
        uint8_t pmu[2];
        i2cRead(0x34, pmu, 2);              // nobody answers in the sim; the start condition is the point
        ms(2);
        check(!inPowerDown && pdWakes == wakes0 + 1, "start condition for another chip wakes it");
        Frame w = readFrame();
        printf("     right after wake: seq %u (before sleep %u)\n", w.seq, before);
        check(w.ok, "answers at 0x42 right after waking");
        check(w.seq == 0, "frame says 'no data yet' instead of repeating the pre-sleep reading");
        ms(1200);
        Frame g = readFrame();
        printf("     1.2 s later: %.2f mph, dir %.1f, seq %u\n", g.mph, g.dir, g.seq);
        check(g.seq != 0 && g.seq != before, "fresh publish 1 s after waking");
        check(g.mph > want * 0.8f && g.mph < want * 1.2f, "speed right from the first second");
        check(g.dir > awakeDir - 1.0f && g.dir < awakeDir + 1.0f, "same direction as before the sleep");
        ms(2000);
        Frame h = readFrame();
        check(h.mph > want * 0.9f && h.mph < want * 1.1f, "speed settles over the 2 s window");
        check(contention == 0, "no bus contention");
    }

    printf("\n5) Old node that never sends the command: chip never powers down\n");
    long e0 = pdEntries;
    for (int i = 0; i < 30; i++) { ms(100); readFrame(); i2cRead(0x34, pmu2, 2); }
    check(pdEntries == e0, "no power-down without the command");

    printf("\n6) Calm while asleep, wind on wake\n");
    { uint8_t cmd[2] = {0x02, 0x00}; i2cWrite(0x42, cmd, 2); }
    pulseHz = 0; ms(5000); pulseHz = hz;
    i2cRead(0x34, pmu2, 2);
    ms(3200);
    Frame c = readFrame();
    check(c.mph > want * 0.9f && c.mph < want * 1.1f, "wind after a calm sleep reads right");

    printf("\nlongest SCL stretch %.3f ms, %ld of %ld reads missed and retried\n",
           maxStretch / 8000.0, readMisses, readsTotal);
    printf("%s (%d failure%s)\n", fails ? "FAILED" : "ALL PASS", fails, fails == 1 ? "" : "s");
    return fails ? 1 : 0;
}
