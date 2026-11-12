/**
 * ATtiny85 Wind Sensor  -  SparkFun / Inspeed
 * ===========================================
 * Same firmware for either kit. The wind meter is chosen AT COMPILE TIME with
 * the WIND_METER_ lines below -- uncomment exactly one before flashing. The build
 * fails on purpose if none (or both) are selected, so a node can never be
 * flashed with the wrong calibration by accident.
 *
 * The choice sets both halves of the calibration together:
 *   SparkFun : 1.492 mph per Hz, 16-position resistor-ladder vane table
 *   Inspeed  : INSPEED_MPH_PER_HZ, continuous ratiometric E-Vane (5-95% Vcc)
 *
 * (Earlier builds tried to auto-detect the meter at boot by probing the vane.
 *  That misclassified real Inspeed hardware as SparkFun, so it was removed.)
 *
 * LED feedback at boot: 2 blinks = SparkFun, 3 blinks = Inspeed. Check this
 * after flashing -- it confirms which calibration is actually running.
 *
 * What gets reported (refreshed every 1 s, over a rolling 10 s window):
 *   speed_avg : pulses in the window / window length. Calm time counts as calm.
 *   speed_sd  : time-weighted SD of per-revolution speeds. Reads 0 in a
 *               perfectly steady wind; a lull counts as slow air, not as a gap.
 *   gust      : highest 3 s mean speed within the window, from revolution
 *               times, so it equals the speed in a steady wind
 *   dir_avg   : unit-vector mean of 100 ms vane samples taken while moving
 *   dir_sd    : Yamartino SD of the same samples. Reads 103.9 (the maximum)
 *               when the cups are turning but the vane signal is out of range
 *               (broken wire, unplugged vane): direction unknown, not "steady".
 *   rawADC    : bits 0-9 = last vane ADC (debug), bits 10-15 = publish
 *               sequence (1..63, never 0 once data exists). The node uses the
 *               sequence to spot a frozen ATtiny that still answers I2C.
 * In a calm (no pulse for 2 s) the vane is ignored and the last direction is held.
 *
 * Hardware: ATtiny85 @ 8 MHz internal, 3.3 V, I2C addr 0x42
 * Pins: PB0=SDA, PB2=SCL, PB3=vane analog, PB4=anemometer pulse, PB1=LED
 *
 * Recommended fuses: BOD at 2.7 V, so a sagging cold battery resets the chip
 * cleanly instead of leaving it in an undefined state.
 */

#include <TinyWireS.h>
#include <math.h>
#include <avr/wdt.h>

// ============================================================================
// CONFIGURATION
// ============================================================================
#define I2C_SLAVE_ADDRESS 0x42

#define WIND_VANE_PIN   3      // PB3 / ADC3
#define ANEMOMETER_PIN  4      // PB4 / PCINT4
#define DEBUG_LED_PIN   1      // PB1

#define TICK_MS            100UL   // vane sample period
#define TICKS_PER_BUCKET   10      // 10 x 100 ms = 1 s bucket
#define WINDOW_BUCKETS     10      // stats window = 10 s, republished every 1 s
#define GUST_BUCKETS       3       // gust = highest 3 s mean in the window
#define CALM_US            2000000UL   // no pulse for 2 s -> cups stopped; skip vane
#define STOPPED_US         60000000UL  // a gap this long is not a revolution period
#define PERIOD_RING        8       // ISR -> loop queue (100 Hz max = 10 per tick)
#define DEBOUNCE_US        10000UL // min time between pulses: 100 Hz cap (149 mph SparkFun)
#define HIGH_MIN_US        1500UL  // line must be open this long before a new closure
                                   // counts: rejects reed bounce when the switch opens

// ---- Calibration constants for each kit ----
#define SPARKFUN_MPH_PER_HZ  1.492f
// Inspeed publishes a different constant per body and rotor (Inspeed "Reed
// Switch Anemometers" instructions, rev. Feb 2020):
//   WS2 body, D2/D3 plastic (polycarbonate) cups : 2.7  mph per Hz
//   WS2 body, metal cups                         : 2.45 mph per Hz
//   Classic body, D2/D3 plastic cups             : 2.85 mph per Hz
// The project BOM specifies the WS2, sold with either plastic or metal cups.
// Earlier builds used 2.5 for all of them (7% low on plastic, 2% high on metal).
// ---- INSPEED CUP SELECTION -- for Inspeed builds, uncomment EXACTLY ONE ----
//#define INSPEED_WS2_PLASTIC_CUPS   // WS2 with plastic (polycarbonate) cups
//#define INSPEED_WS2_METAL_CUPS     // WS2 with metal cups
//#define INSPEED_CLASSIC_PLASTIC    // Classic (1.25" body) with plastic cups

// Inspeed E-Vane ratiometric span (0..360 deg maps ADC AT_0..AT_360).
// CALIBRATE for your build: point vane North -> read ADC -> VANE_ADC_AT_0.
#define VANE_ADC_AT_0    51       // ~5% of Vcc
#define VANE_ADC_AT_360  972      // ~95% of Vcc

// ---- WIND METER SELECTION -- uncomment EXACTLY ONE line ----
//#define WIND_METER_SPARKFUN   // SparkFun weather meter kit
//#define WIND_METER_INSPEED    // Inspeed Vortex + E-Vane

#if defined(WIND_METER_SPARKFUN) && defined(WIND_METER_INSPEED)
  #error "Both wind meters are selected. Uncomment only one WIND_METER_ line near the top of this file."
#elif !defined(WIND_METER_SPARKFUN) && !defined(WIND_METER_INSPEED)
  #error "No wind meter selected. Uncomment one WIND_METER_ line near the top of this file."
#endif

#if defined(WIND_METER_INSPEED)
  #if (defined(INSPEED_WS2_PLASTIC_CUPS) + defined(INSPEED_WS2_METAL_CUPS) + defined(INSPEED_CLASSIC_PLASTIC)) != 1
    #error "Inspeed build: uncomment exactly ONE INSPEED_ cup line (WS2 plastic, WS2 metal or Classic) near the top of this file."
  #endif
  #if defined(INSPEED_WS2_PLASTIC_CUPS)
    #define INSPEED_MPH_PER_HZ 2.7f
  #elif defined(INSPEED_WS2_METAL_CUPS)
    #define INSPEED_MPH_PER_HZ 2.45f
  #else
    #define INSPEED_MPH_PER_HZ 2.85f
  #endif
#endif

// ============================================================================
// SENSOR SELECTION (fixed at compile time by the WIND_METER_ line above)
// ============================================================================
enum SensorType { SENSOR_SPARKFUN = 0, SENSOR_INSPEED = 1 };
#if defined(WIND_METER_INSPEED)
const SensorType sensorType    = SENSOR_INSPEED;
const float      anemoMphPerHz = INSPEED_MPH_PER_HZ;
#else
const SensorType sensorType    = SENSOR_SPARKFUN;
const float      anemoMphPerHz = SPARKFUN_MPH_PER_HZ;
#endif

// ============================================================================
// I2C DATA STRUCTURE  (12 bytes -- must match the node's AttinyWind struct)
// ============================================================================
struct WindData {
    uint16_t speed_avg;   // mph * 100
    uint16_t speed_sd;    // mph * 100
    uint16_t dir_avg;     // deg * 10
    uint16_t dir_sd;      // deg * 10
    uint16_t gust;        // mph * 100
    uint16_t rawADC;      // bits 0-9 vane ADC, bits 10-15 publish sequence
} windData;

// ============================================================================
// SPARKFUN 16-POSITION VANE TABLE
// ============================================================================
// The vane has 8 reeds; between them two reeds close at once, giving 16
// distinct voltages. The old 8-entry table snapped the in-between positions to
// whichever entry was nearest in ADC counts, which is not nearest in angle:
// 337.5 deg landed 112.5 deg away. All 16 are listed here, sorted by ADC.
//
// ADC values assume the 10k pull-up to the same 3.3 V as the ADC reference.
// The 8 main positions keep the values used in the field; the 8 in-between ones
// are computed from the datasheet resistances (1023 * R / (R + 10k)).
//
// Angles keep the convention this project has used in the field, which is the
// datasheet angle + 180 deg. Vectors are stored as cos/sin * 1000.
struct VaneCal { uint16_t adc; int16_t c1000; int16_t s1000; };
static const VaneCal vaneTable[] PROGMEM = {
    {  66,   383, -924},  // 292.5
    {  84,  -383, -924},  // 247.5
    {  93,     0,-1000},  // 270   W
    { 126,   924, -383},  // 337.5
    { 181,   707, -707},  // 315   NW
    { 244,   924,  383},  //  22.5
    { 284,  1000,    0},  //   0   N
    { 406,  -924, -383},  // 202.5
    { 459,  -707, -707},  // 225   SW
    { 599,   383,  924},  //  67.5
    { 628,   707,  707},  //  45   NE
    { 702,  -924,  383},  // 157.5
    { 784, -1000,    0},  // 180   S
    { 827,  -383,  924},  // 112.5
    { 887,  -707,  707},  // 135   SE
    { 945,     0, 1000},  //  90   E
};
static const uint8_t vaneTableSize = sizeof(vaneTable) / sizeof(vaneTable[0]);

// A vane reading outside the range the sensor can produce is a wiring fault,
// not a direction. SparkFun: the lowest and highest table entries are 66 and
// 945, and an open circuit sits at the pull-up rail (~1023), which used to
// decode as the nearest entry, 90 deg, with an SD of 0. Inspeed: the E-Vane
// only drives 5-95% of Vcc.
#define VANE_SF_ADC_MIN   33      // halfway between 0 and the lowest entry
#define VANE_SF_ADC_MAX   984     // halfway between the highest entry and 1023
#define VANE_IN_ADC_SLACK 30      // counts allowed beyond AT_0 / AT_360
#define DIR_SD_UNKNOWN    1039    // 103.9 deg: the largest value Yamartino gives
bool vaneAdcValid(int adc) {
    if (sensorType == SENSOR_SPARKFUN)
        return adc >= VANE_SF_ADC_MIN && adc <= VANE_SF_ADC_MAX;
    return adc >= (VANE_ADC_AT_0 - VANE_IN_ADC_SLACK) && adc <= (VANE_ADC_AT_360 + VANE_IN_ADC_SLACK);
}

// Unit vector (cos, sin) * 1000 for the current vane reading.
void decodeVaneVector(int adc, int16_t& c1000, int16_t& s1000) {
    if (sensorType == SENSOR_SPARKFUN) {
        int minDiff = 32767; uint8_t best = 0;
        for (uint8_t i = 0; i < vaneTableSize; i++) {
            int diff = adc - (int)pgm_read_word(&vaneTable[i].adc); if (diff < 0) diff = -diff;
            if (diff < minDiff) { minDiff = diff; best = i; }
        }
        c1000 = (int16_t)pgm_read_word(&vaneTable[best].c1000);
        s1000 = (int16_t)pgm_read_word(&vaneTable[best].s1000);
    } else {
        // Inspeed: continuous linear angle -> trig
        float span = (float)(VANE_ADC_AT_360 - VANE_ADC_AT_0);
        float deg  = (span != 0) ? (float)(adc - VANE_ADC_AT_0) * 360.0f / span : 0.0f;
        while (deg < 0.0f)    deg += 360.0f;
        while (deg >= 360.0f) deg -= 360.0f;
        float th = deg * (float)DEG_TO_RAD;
        c1000 = (int16_t)lroundf(cos(th) * 1000.0f);
        s1000 = (int16_t)lroundf(sin(th) * 1000.0f);
    }
}

void blink(uint8_t n) {
    for (uint8_t i = 0; i < n; i++) {
        digitalWrite(DEBUG_LED_PIN, HIGH); delay(180);
        digitalWrite(DEBUG_LED_PIN, LOW);  delay(180);
    }
}

// ============================================================================
// ANEMOMETER -- each accepted pulse is counted and timestamped in the ISR
// ============================================================================
// The ISR counts pulses (never lost, even if the period queue is full) and
// queues the period since the previous pulse. All math happens in loop().
//
// A closure (falling edge) only counts if the line had been open (HIGH) for at
// least HIGH_MIN_US, and at least DEBOUNCE_US has passed since the last pulse.
// The 10 ms debounce alone let release bounce through at low speed, where a
// reed stays closed for hundreds of ms.
#define NO_PERIOD 0xFFFFFFFFUL
volatile uint32_t periodRing[PERIOD_RING];   // completed periods, us (or NO_PERIOD)
volatile uint8_t  ringHead = 0, ringCount = 0;
volatile uint16_t pulseCount = 0;             // every accepted pulse
volatile uint32_t lastEdgeUs = 0;
volatile uint32_t lastRiseUs = 0;
volatile bool     haveEdge   = false;         // false until the first pulse

ISR(PCINT0_vect) {
    uint32_t now = micros();
    if (digitalRead(ANEMOMETER_PIN) != LOW) { lastRiseUs = now; return; }   // opened
    if ((uint32_t)(now - lastRiseUs) < HIGH_MIN_US) return;                  // bounce
    if (haveEdge && (uint32_t)(now - lastEdgeUs) < DEBOUNCE_US) return;      // too fast
    uint32_t dt = haveEdge ? (uint32_t)(now - lastEdgeUs) : NO_PERIOD;
    if (dt >= STOPPED_US) dt = NO_PERIOD;        // after a long calm: not a period
    haveEdge = true;
    lastEdgeUs = now;
    pulseCount++;
    if (ringCount < PERIOD_RING) {
        periodRing[(uint8_t)(ringHead + ringCount) % PERIOD_RING] = dt;
        ringCount++;
    }
}

// ============================================================================
// ROLLING WINDOW: WINDOW_BUCKETS one-second buckets
// ============================================================================
// Mean speed is a plain count over the WHOLE window W (calm time included):
//   mean speed = K * pulses / W
//
// Speed SD and gust use the revolution times. Speed is v = K / T_i during
// revolution i, which lasts T_i. Weighting each revolution by its own duration,
// over the time C = sum(T_i) that the n revolutions took:
//   mean       = K * n / C
//   mean of v2 = K^2 * sum(1/T_i) / C
//   SD         = sqrt(mean of v2 - mean^2)
// In a steady wind every T_i is equal and this is exactly 0. This file first
// divided by W instead of C, which is not: the revolutions never fill the window
// exactly, and the leftover part of a turn was counted as dead calm. That read
// as up to 1 mph of SD in perfectly steady air.
//
// Calm still has to count as calm. A gap longer than CALM_US between pulses is
// not a revolution (the cups were stopped), so it is left out of n and C. Its
// time then shows up as C falling well short of the span, and spanDivisor()
// switches to dividing by the whole span, which counts that time as zero speed.
//
// Gust is the highest 3 s mean, computed the same way over 3 buckets. In a
// steady wind it equals the speed (a pulse count over 3 s is off by up to one
// pulse, +0.5 mph SparkFun, +0.9 mph Inspeed). A short burst in a calm still
// averages over the full 3 s, so a few stray pulses cannot read as a gust.
//
// Direction: unit vectors of 100 ms vane samples, skipping samples taken while
// the cups are stopped (no pulse for CALM_US) -- a vane in a lull just drifts.
struct Bucket {
    float    sumInvT;   // sum of 1/T (1/us) for revolutions ending in this bucket
    uint32_t sumT;      // sum of T (us) for the same revolutions
    int16_t  sinS;      // sum of sin * 1000 (<= 10 samples -> fits)
    int16_t  cosS;      // sum of cos * 1000
    uint8_t  pulses;    // pulses in this bucket (<= 100 at 100 Hz)
    uint8_t  nPer;      // how many revolutions the two sums hold
    uint8_t  nDir;      // vane samples counted for direction
};
#define EMPTY_BUCKET Bucket{0.0f, 0, 0, 0, 0, 0, 0}
static Bucket   bk[WINDOW_BUCKETS];
static uint8_t  curB    = 0;      // bucket being filled
static uint8_t  filledB = 1;      // buckets holding data (window warm-up)
static uint16_t lastPulseSnap = 0;
static uint16_t lastVaneADC   = 0;
static uint8_t  publishSeq    = 0; // 1..63, 0 = nothing published yet
static const float K_MPH_US = anemoMphPerHz * 1000000.0f;  // mph * us per pulse
static const float K_MPH_HZ = anemoMphPerHz;

static inline uint8_t bucketAgo(uint8_t ago) {     // 0 = current bucket
    return (uint8_t)((curB + WINDOW_BUCKETS - ago) % WINDOW_BUCKETS);
}

// Move pulse counts and queued periods from the ISR into the current bucket.
void drainPulses() {
    noInterrupts();
    uint16_t pc = pulseCount;
    interrupts();
    uint16_t d = (uint16_t)(pc - lastPulseSnap);
    lastPulseSnap = pc;
    Bucket& b = bk[curB];
    b.pulses = (uint8_t)((b.pulses + d > 255) ? 255 : b.pulses + d);

    for (;;) {
        noInterrupts();
        if (ringCount == 0) { interrupts(); return; }
        uint32_t T = periodRing[ringHead];
        ringHead = (uint8_t)(ringHead + 1) % PERIOD_RING;
        ringCount--;
        interrupts();
        // Only gaps short enough to be a revolution. Anything longer is the
        // cups starting again after a stop: calm time, not a slow turn.
        if (T != NO_PERIOD && T > 0 && T <= CALM_US && b.nPer < 255) {
            b.sumInvT += 1.0f / (float)T;
            b.sumT    += T;
            b.nPer++;
        }
    }
}

// One 100 ms vane sample into the current bucket.
void sampleVane() {
    int adc = analogRead(WIND_VANE_PIN);
    lastVaneADC = (uint16_t)adc;

    noInterrupts();
    bool     he = haveEdge;
    uint32_t le = lastEdgeUs;
    interrupts();
    bool moving = he && (uint32_t)(micros() - le) < CALM_US;
    if (!moving) return;                       // calm: vane reading is meaningless
    if (!vaneAdcValid(adc)) return;            // wiring fault: not a direction

    int16_t vCos, vSin;
    decodeVaneVector(adc, vCos, vSin);
    Bucket& b = bk[curB];
    b.cosS += vCos;
    b.sinS += vSin;
    b.nDir++;
}

static inline uint16_t toU16(float v, float scale) {
    float x = v * scale + 0.5f;
    if (x < 0.0f) x = 0.0f;
    if (x > 65535.0f) x = 65535.0f;
    return (uint16_t)x;
}

// Time to divide by when averaging n revolutions that took `cov` us inside a
// span of `whole` us. If they fill the span to within two average turns, the
// difference is only where the first and last turn happen to fall, so use the
// time they really took (exact in a steady wind). Otherwise part of the span
// was calm, or a turn reached back well before it: use the whole span, which
// counts the calm as zero speed.
static float spanDivisor(float cov, uint16_t n, float whole) {
    float slack = 2.0f * cov / (float)n;
    float gap   = whole - cov;
    if (gap > slack || gap < -slack) return whole;
    return cov;
}

// Recompute every statistic over the window and publish it for I2C.
void publishWindow() {
    // ---- totals over the window ----
    uint16_t N = 0, NP = 0; float SI = 0.0f; uint32_t CT = 0;
    int32_t  sS = 0, cS = 0; uint16_t nD = 0;
    for (uint8_t a = 0; a < filledB; a++) {
        const Bucket& b = bk[bucketAgo(a)];
        N += b.pulses; NP += b.nPer; SI += b.sumInvT; CT += b.sumT;
        sS += b.sinS; cS += b.cosS; nD += b.nDir;
    }
    float Wsec = (float)filledB * (TICK_MS * TICKS_PER_BUCKET) / 1000.0f;
    float Wus  = Wsec * 1000000.0f;

    float spdAvg = K_MPH_HZ * (float)N / Wsec;

    // ---- speed SD: revolutions weighted by their own duration ----
    float spdSd = 0.0f;
    if (NP > 0) {
        float den  = spanDivisor((float)CT, NP, Wus);
        float mean = K_MPH_US * (float)NP / den;
        float var  = K_MPH_US * K_MPH_US * SI / den - mean * mean;
        if (var > 0.0f) spdSd = sqrt(var);
    }

    // ---- gust: highest GUST_BUCKETS-second mean inside the window ----
    float gust = 0.0f;
    uint8_t span = (filledB < GUST_BUCKETS) ? filledB : GUST_BUCKETS;
    float spanSec = (float)span * (TICK_MS * TICKS_PER_BUCKET) / 1000.0f;
    float spanUs  = spanSec * 1000000.0f;
    for (uint8_t start = 0; start + span <= filledB; start++) {
        uint16_t gN = 0, gP = 0; uint32_t gT = 0;
        for (uint8_t k = 0; k < span; k++) {
            const Bucket& b = bk[bucketAgo(start + k)];
            gN += b.pulses; gP += b.nPer; gT += b.sumT;
        }
        float g;
        if (gP == 0 || gN > gP + 1) {
            // No revolution times to use, or some are missing: wind too light
            // to turn the cups every CALM_US, or the ISR queue overflowed (only
            // above ~80 Hz). Fall back to the plain count.
            g = K_MPH_HZ * (float)gN / spanSec;
        } else {
            g = K_MPH_US * (float)gP / spanDivisor((float)gT, gP, spanUs);
        }
        if (g > gust) gust = g;
    }
    if (gust < spdAvg) gust = spdAvg;

    // ---- direction (moving samples only; keep the last value in a calm) ----
    WindData out = windData;
    if (nD > 0) {
        float meanSin = (float)sS / (1000.0f * nD), meanCos = (float)cS / (1000.0f * nD);
        float dirAvg = atan2(meanSin, meanCos) * (float)RAD_TO_DEG;
        if (dirAvg < 0) dirAvg += 360.0f;
        float R = sqrt(meanSin * meanSin + meanCos * meanCos);
        if (R > 1.0f) R = 1.0f;
        float eps   = sqrt(1.0f - R * R);
        float dirSd = asin(eps) * (1.0f + 0.1547f * eps * eps * eps) * (float)RAD_TO_DEG;   // Yamartino
        out.dir_avg = toU16(dirAvg, 10.0f);
        if (out.dir_avg >= 3600) out.dir_avg -= 3600;
        out.dir_sd  = toU16(dirSd, 10.0f);
    } else if (N > 0) {
        // Cups turning but not one usable vane sample in the window: the vane
        // signal is out of range. Keep the last direction, but say it is
        // unknown rather than leaving a small SD that reads as steady.
        out.dir_sd = DIR_SD_UNKNOWN;
    }
    out.speed_avg = toU16(spdAvg, 100.0f);
    out.speed_sd  = toU16(spdSd,  100.0f);
    out.gust      = toU16(gust,   100.0f);
    publishSeq = (publishSeq >= 63) ? 1 : (uint8_t)(publishSeq + 1);
    out.rawADC    = (uint16_t)((lastVaneADC & 0x03FF) | ((uint16_t)publishSeq << 10));

    // requestEvent() runs inside the I2C interrupt: swap the whole struct in
    // with interrupts off so a read never mixes two windows.
    noInterrupts();
    windData = out;
    interrupts();
}

// Start a fresh bucket, dropping the oldest once the window is full.
void advanceBucket() {
    curB = (uint8_t)(curB + 1) % WINDOW_BUCKETS;
    if (filledB < WINDOW_BUCKETS) filledB++;
    bk[curB] = EMPTY_BUCKET;
}

// ============================================================================
// I2C
// ============================================================================
// This file is written for the "TinyWireS" library in the Arduino Library
// Manager (TinyWireS 1.0.0 by Nadav Matalon, github.com/nadavmatalon/TinyWireS).
// It is the fork whose calls are write() / read() / TinyWireS_stop_check(),
// which are the calls this sketch has always made.
//
// That library calls the request handler ONCE per read, at the address byte,
// and queues into a 32-byte TX buffer. requestEvent() relies on both: it queues
// the whole 12-byte frame in one call.
//
// Do NOT build this against rambo/TinyWire (calls send() / receive()). It calls
// the request handler once per BYTE and has a 16-byte buffer, so queueing a
// full frame on every call overruns it and hangs inside the I2C interrupt.
// rambo/TinyWire has no write(), so that mistake fails at compile time here,
// on purpose, instead of in the field.
//
// Known limit: if a read is cut short (the node resets mid-read, or clocks the
// bus clear), the unsent bytes stay queued and every later frame comes out
// shifted by that many bytes. Nothing in the frame lets the node detect this.
// If the backlog ever reaches the point where a new frame no longer fits,
// write() spins inside the interrupt and the watchdog below resets the chip,
// which also clears it. A proper fix needs the library to flush its TX buffer
// at the address byte, or a checksum in the frame.
void requestEvent() {
    uint8_t* ptr = (uint8_t*)&windData;
    for (uint8_t i = 0; i < sizeof(windData); i++) TinyWireS.write(ptr[i]);
}

// ============================================================================
// SETUP
// ============================================================================
static unsigned long lastTick = 0;
static uint8_t       ticks    = 0;

void setup() {
    MCUSR = 0;                                  // clear reset flags so the WDT
    wdt_disable();                              // can't loop us on startup

    pinMode(DEBUG_LED_PIN, OUTPUT);
    blink(1);                                   // "alive"

    // ---- report which wind meter this build is calibrated for ----
    delay(300);
    blink(sensorType == SENSOR_INSPEED ? 3 : 2); // 2 = SparkFun, 3 = Inspeed

    windData = {0, 0, 0, 0, 0, 0};              // rawADC seq 0 = no data yet
    for (uint8_t i = 0; i < WINDOW_BUCKETS; i++) bk[i] = EMPTY_BUCKET;

    pinMode(ANEMOMETER_PIN, INPUT);             // external ~10k pull-up required
    PCMSK |= (1 << PCINT4);
    GIMSK |= (1 << PCIE);
    sei();

    TinyWireS.begin(I2C_SLAVE_ADDRESS);
    TinyWireS.onRequest(requestEvent);

    // Watchdog: if loop() stops running (including a hang inside the I2C
    // interrupt), reset the chip after ~2 s instead of holding the bus forever.
    wdt_enable(WDTO_2S);

    lastTick = millis();                        // start sampling now, not at boot
}

// ============================================================================
// LOOP
// ============================================================================
void loop() {
    wdt_reset();
    TinyWireS_stop_check();

    unsigned long now = millis();

    if (now - lastTick >= TICK_MS) {
        // If the loop fell more than a tick behind, resync instead of taking a
        // burst of back-to-back samples labelled as 100 ms apart.
        if (now - lastTick >= 2 * TICK_MS) lastTick = now;
        else                               lastTick += TICK_MS;
        drainPulses();
        sampleVane();
        if (++ticks >= TICKS_PER_BUCKET) {       // every 1 s
            ticks = 0;
            publishWindow();
            advanceBucket();
        }
    }

    static unsigned long lastBeat = 0;
    if (now - lastBeat >= 2000) {
        digitalWrite(DEBUG_LED_PIN, !digitalRead(DEBUG_LED_PIN));
        lastBeat = now;
    }
}
