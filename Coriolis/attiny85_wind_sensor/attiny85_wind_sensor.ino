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
 * What gets reported (refreshed every 1 s). speed_avg and dir_avg cover the
 * last 2 s so the phone follows a change within a few seconds; the spread
 * figures (speed_sd, dir_sd) and gust keep the rolling 10 s window:
 *   speed_avg : mean speed over the newest 2 s, from revolution times (worked
 *               out like one gust span). Calm time counts as calm.
 *   speed_sd  : time-weighted SD of per-revolution speeds. Reads 0 in a
 *               perfectly steady wind; a lull counts as slow air, not as a gap.
 *   gust      : highest 3 s mean speed within the window, from revolution
 *               times, so it equals the speed in a steady wind
 *   dir_avg   : unit-vector mean of the 100 ms vane samples taken while moving
 *               in the newest 2 s (held if those 2 s have none)
 *   dir_sd    : Yamartino SD of the vane samples over the 10 s window. Reads 103.9 (the maximum)
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
 *
 * Power (see "POWER" below). The chip is fed from the T-Beam's 3.3 V rail, which
 * stays on while the ESP32 deep-sleeps, so whatever this sketch draws, the node
 * draws around the clock. It used to run flat out (~3 mA) and hold the LED on
 * half the time (~10 mA through the 100 ohm resistor, ~5 mA average). Now:
 *   - between 100 ms ticks the CPU idles instead of spinning (~3 mA -> ~1 mA)
 *   - the ADC is only switched on for the vane sample
 *   - the heartbeat is a 20 ms blip every 2 s instead of 2 s on / 2 s off
 *   - when the node writes {0x02, 0x00} just before it deep-sleeps, the chip
 *     powers down (~1 uA: LED, ADC, watchdog, brown-out detector all off) and
 *     wakes on the next I2C start condition, i.e. when the node wakes and
 *     talks to its power chip. Wind data starts fresh after a wake.
 *   An older node that never sends the command just leaves it awake, as before.
 */

#include <TinyWireS.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <math.h>

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
#define LIVE_BUCKETS       2       // speed_avg/dir_avg = the last 2 s (SD and gust keep 10 s)
#define CALM_US            2000000UL   // no pulse for 2 s -> cups stopped; skip vane
#define STOPPED_US         60000000UL  // a gap this long is not a revolution period
#define PERIOD_RING        8       // ISR -> loop queue (100 Hz max = 10 per tick)
#define DEBOUNCE_US        10000UL // min time between pulses: 100 Hz cap (149 mph SparkFun)
#define HIGH_MIN_US        1500UL  // line must be open this long before a new closure
                                   // counts: rejects reed bounce when the switch opens
#define HEARTBEAT_MS       2000UL  // LED heartbeat period...
#define HEARTBEAT_ON_MS    20UL    // ...and how long it is lit (was 2 s on / 2 s off)

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

// x * 1000 rounded to the nearest integer, for |x| <= 1. Same result as
// lroundf(x * 1000) without pulling lround() into the 7168-byte sketch.
static int16_t round1000(float x) {
    x *= 1000.0f;
    return (int16_t)(x < 0.0f ? x - 0.5f : x + 0.5f);
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
        // Inspeed: continuous linear angle -> trig. cos/sin don't care about
        // whole turns, so the angle needs no wrapping into 0..360 first.
        const float radPerCount = 2.0f * (float)PI / (float)(VANE_ADC_AT_360 - VANE_ADC_AT_0);
        float th = (float)(adc - VANE_ADC_AT_0) * radPerCount;
        c1000 = round1000(cos(th));
        s1000 = round1000(sin(th));
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
// A pulse is the first edge after the line has rested open (HIGH, no edges)
// for at least HIGH_MIN_US, and only if at least DEBOUNCE_US has passed since
// the last pulse. The 10 ms debounce alone let release bounce through at low
// speed, where a reed stays closed for hundreds of ms.
//
// The level is read a few us after the edge, so on a bouncing contact the
// read can already be HIGH again. That edge still counts: it ends a quiet
// open spell, so it is a closure whatever the read says. The read is only
// used to know whether the line is resting open or closed afterwards.
#define NO_PERIOD 0xFFFFFFFFUL
volatile uint32_t periodRing[PERIOD_RING];   // completed periods, us (or NO_PERIOD)
volatile uint8_t  ringHead = 0, ringCount = 0;
volatile uint16_t pulseCount = 0;             // every accepted pulse
volatile uint32_t lastEdgeUs = 0;
volatile uint32_t lastAnyEdgeUs = 0;          // last pin change of any kind
volatile bool     lineHigh   = true;          // level read after that change
volatile bool     haveEdge   = false;         // false until the first pulse

ISR(PCINT0_vect) {
    uint32_t now = micros();
    bool quietHigh = lineHigh && (uint32_t)(now - lastAnyEdgeUs) >= HIGH_MIN_US;
    lastAnyEdgeUs = now;
    lineHigh = digitalRead(ANEMOMETER_PIN) != LOW;
    if (!quietHigh) return;                      // bounce, or the contact opening
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
    // The ADC is only powered for this one conversion (POWER below). The first
    // conversion after switching it on takes 25 ADC clocks instead of 13, about
    // 200 us at 125 kHz, which analogRead() simply waits out.
    ADCSRA |= (1 << ADEN);
    int adc = analogRead(WIND_VANE_PIN);
    ADCSRA &= ~(1 << ADEN);
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

// Mean speed over `span` buckets starting `start` buckets back (0 = newest),
// from revolution times so a steady wind reads exactly.
static float __attribute__((noinline)) spanMean(uint8_t start, uint8_t span) {
    uint16_t gN = 0, gP = 0; uint32_t gT = 0;
    for (uint8_t k = 0; k < span; k++) {
        const Bucket& b = bk[bucketAgo(start + k)];
        gN += b.pulses; gP += b.nPer; gT += b.sumT;
    }
    float spanSec = (float)span * (TICK_MS * TICKS_PER_BUCKET) / 1000.0f;
    if (gP == 0 || gN > gP + 1) {
        // No revolution times to use, or some are missing: wind too light
        // to turn the cups every CALM_US, or the ISR queue overflowed (only
        // above ~80 Hz). Fall back to the plain count.
        return K_MPH_HZ * (float)gN / spanSec;
    }
    return K_MPH_US * (float)gP / spanDivisor((float)gT, gP, spanSec * 1000000.0f);
}

// Recompute every statistic over the window and publish it for I2C.
void publishWindow() {
    // ---- totals over the window ----
    uint16_t N = 0, NP = 0; float SI = 0.0f; uint32_t CT = 0;
    int32_t  sS = 0, cS = 0; uint16_t nD = 0;
    int32_t  lS = 0, lC = 0; uint16_t lD = 0;      // the last LIVE_BUCKETS only
    for (uint8_t a = 0; a < filledB; a++) {
        const Bucket& b = bk[bucketAgo(a)];
        N += b.pulses; NP += b.nPer; SI += b.sumInvT; CT += b.sumT;
        sS += b.sinS; cS += b.cosS; nD += b.nDir;
        if (a < LIVE_BUCKETS) { lS += b.sinS; lC += b.cosS; lD += b.nDir; }
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

    // ---- live speed: the newest LIVE_BUCKETS, worked out like one gust span ----
    float live = spanMean(0, (filledB < LIVE_BUCKETS) ? filledB : LIVE_BUCKETS);

    // ---- gust: highest GUST_BUCKETS-second mean inside the window ----
    // Starts at the live speed: a rising wind shows in 2 s before it fills 3 s.
    float gust = live;
    uint8_t span = (filledB < GUST_BUCKETS) ? filledB : GUST_BUCKETS;
    for (uint8_t start = 0; start + span <= filledB; start++) {
        float g = spanMean(start, span);
        if (g > gust) gust = g;
    }
    if (gust < spdAvg) gust = spdAvg;

    // ---- direction (moving samples only; keep the last value in a calm) ----
    WindData out = windData;
    if (nD > 0) {
        float meanSin = (float)sS / (1000.0f * nD), meanCos = (float)cS / (1000.0f * nD);
        // Live direction: the newest LIVE_BUCKETS only. With no moving sample in
        // them (cups just stopped) keep the last one; the SD keeps the 10 s window.
        if (lD > 0) {
            float dirAvg = atan2((float)lS, (float)lC) * (float)RAD_TO_DEG;
            if (dirAvg < 0) dirAvg += 360.0f;
            out.dir_avg = toU16(dirAvg, 10.0f);
            if (out.dir_avg >= 3600) out.dir_avg -= 3600;
        }
        float R = sqrt(meanSin * meanSin + meanCos * meanCos);
        if (R > 1.0f) R = 1.0f;
        float eps   = sqrt(1.0f - R * R);
        float dirSd = asin(eps) * (1.0f + 0.1547f * eps * eps * eps) * (float)RAD_TO_DEG;   // Yamartino
        out.dir_sd  = toU16(dirSd, 10.0f);
    } else if (N > 0) {
        // Cups turning but not one usable vane sample in the window: the vane
        // signal is out of range. Keep the last direction, but say it is
        // unknown rather than leaving a small SD that reads as steady.
        out.dir_sd = DIR_SD_UNKNOWN;
    }
    out.speed_avg = toU16(live, 100.0f);
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

// I2C firmware update (see attiny85_bootloader/README.md). The node writes
// {0x01, 0x00} here to ask for the bootloader; we answer with a watchdog
// reset, which starts the bootloader at I2C address 0x43. On a chip without
// the bootloader this just restarts the sketch. Keep the sketch under 7168
// bytes (the bootloader uses the top 1 KB) and do not use EE_RDY_vect.
//
// {0x02, 0x00} asks the chip to power down until the node wakes (POWER below).
// Older sketches ignore it, so a new node works with an old ATtiny and the
// other way round.
volatile bool bootloaderRequested = false;
volatile bool sleepRequested      = false;

void receiveEvent(uint8_t n) {
    uint8_t b0 = TinyWireS.read();
    uint8_t b1 = (n > 1) ? TinyWireS.read() : 0xFF;
    while (TinyWireS.available()) TinyWireS.read();
    if (n != 2 || b1 != 0x00) return;
    if (b0 == 0x01) bootloaderRequested = true;
    if (b0 == 0x02) sleepRequested = true;
}

// ============================================================================
// POWER
// ============================================================================
// Wiring that sets the floor (daughterboard schematic, 2026-05-30): the chip,
// the vane and both 10k pull-ups hang off the T-Beam 3.3 V pin, which the
// power chip keeps on while the ESP32 sleeps. The LED is on PB1 through 100
// ohm, so lit it takes ~10 mA, three times the chip itself. Nothing here can
// switch off the vane (SparkFun ladder through R2 ~0.03-0.3 mA, Inspeed
// E-Vane its own supply current) or a reed switch that stopped closed (0.33 mA
// through R3); that needs a switched supply on the next board.

static unsigned long lastTick = 0;
static uint8_t       ticks    = 0;
static uint16_t      lastBeat = 0;     // low 16 bits of millis(): plenty for 2 s

// Parts this sketch never uses, switched off for good at boot.
static void powerTrimAtBoot() {
    ACSR  |= (1 << ACD);            // analog comparator off (on by default)
    DIDR0 |= (1 << ADC3D);          // no digital input buffer on the vane pin:
                                    // a mid-rail analog level makes it draw current
    ADCSRA &= ~(1 << ADEN);         // ADC on only during a sample (sampleVane)
    PRR   |= (1 << PRTIM1);         // timer 1 unused (millis runs on timer 0)
}

// Wind state as if the chip had just booted, keeping the last direction (held
// as in a calm) and the publish counter. millis() and micros() stood still
// while powered down, so the stored pulse times say nothing about the gap; the
// first pulse after a wake only starts timing (haveEdge = false).
static void __attribute__((noinline)) restartMeasuring() {
    memset(bk, 0, sizeof(bk));       // every bucket = EMPTY_BUCKET
    curB = 0; filledB = 1; ticks = 0;
    noInterrupts();
    ringCount     = 0;
    haveEdge      = false;           // the next pulse starts timing, no period yet
    lineHigh      = (PINB & (1 << ANEMOMETER_PIN)) != 0;   // may have moved while asleep
    lastPulseSnap = pulseCount;
    // Sequence 0 says "no data yet" to the node, so it never takes the frame
    // from before the sleep as a new reading. The first publish, 1 s from now,
    // carries the next sequence number as usual.
    windData.rawADC = lastVaneADC & 0x03FF;
    interrupts();
    lastTick = millis();
    lastBeat = (uint16_t)lastTick;
}

// Power down until the next I2C start condition. Called from loop(), so the
// write that asked for it has finished. Every start condition on the bus wakes
// the chip, whoever it is addressed to: the node's first I2C transaction after
// its own wake (the power chip, in setup) brings us back. That is also why the
// node sends the command as its LAST I2C transaction before deep sleep.
static void powerDownUntilI2C() {
    digitalWrite(DEBUG_LED_PIN, LOW);
    PCMSK &= ~(1 << PCINT4);         // wind pulses must not wake us
    wdt_disable();                   // the watchdog oscillator alone is ~4 uA; a
                                     // powered-down chip can't hang anyway
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    cli();
    sleepRequested = false;
    sleep_enable();
#if defined(BODS) && defined(BODSE)
    sleep_bod_disable();             // brown-out detector off while asleep (~20 uA);
#endif                               // the hardware turns it back on at wake
    sei();                           // SEI + SLEEP: no interrupt can slip between
    sleep_cpu();
    sleep_disable();
    // Woken by a start condition: the USI interrupt is already handling it.
    wdt_enable(WDTO_2S);
    restartMeasuring();
    GIFR   = (1 << PCIF);            // drop a pin change latched while asleep
    PCMSK |= (1 << PCINT4);
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
    MCUSR = 0;                                  // clear reset flags so the WDT
    wdt_disable();                              // can't loop us on startup

    pinMode(DEBUG_LED_PIN, OUTPUT);
    blink(1);                                   // "alive"

    // ---- report which wind meter this build is calibrated for ----
    delay(300);
    blink(sensorType == SENSOR_INSPEED ? 3 : 2); // 2 = SparkFun, 3 = Inspeed

    powerTrimAtBoot();
    windData = {0, 0, 0, 0, 0, 0};              // rawADC seq 0 = no data yet

    pinMode(ANEMOMETER_PIN, INPUT);             // external ~10k pull-up required
    PCMSK |= (1 << PCINT4);
    GIMSK |= (1 << PCIE);
    sei();

    TinyWireS.begin(I2C_SLAVE_ADDRESS);
    TinyWireS.onRequest(requestEvent);
    TinyWireS.onReceive(receiveEvent);

    // Watchdog: if loop() stops running (including a hang inside the I2C
    // interrupt), reset the chip after ~2 s instead of holding the bus forever.
    wdt_enable(WDTO_2S);

    restartMeasuring();                         // empty window; start sampling now
}

// ============================================================================
// LOOP
// ============================================================================
void loop() {
    wdt_reset();
    TinyWireS_stop_check();

    if (bootloaderRequested) {                  // reset into the bootloader
        cli();
        wdt_disable();                          // restart the WDT from zero so a
        wdt_enable(WDTO_15MS);                  // longer running timeout can't delay it
        for (;;) {}
    }

    if (sleepRequested) powerDownUntilI2C();    // returns when the node wakes

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

    // Heartbeat: a short blip, not 2 s on / 2 s off (that alone was ~5 mA).
    uint16_t beat = (uint16_t)now - lastBeat;
    if (beat >= HEARTBEAT_MS) { lastBeat += HEARTBEAT_MS; beat -= HEARTBEAT_MS; }
    digitalWrite(DEBUG_LED_PIN, beat < HEARTBEAT_ON_MS ? HIGH : LOW);

    // Nothing to do until the next interrupt: the millis timer (every 2 ms),
    // a wind pulse or I2C. Idle stops the CPU clock but keeps all of those
    // running, so timing and I2C behave exactly as when spinning.
    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_mode();
}
