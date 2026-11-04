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
 *   speed_sd  : time-weighted SD of per-pulse speeds, calm time counted as 0
 *   gust      : highest 3 s mean speed within the window
 *   dir_avg   : unit-vector mean of 100 ms vane samples taken while moving
 *   dir_sd    : Yamartino SD of the same samples
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
// Inspeed publishes a different constant per body and rotor (Reed Switch Sensor
// Instructions): WS2 with plastic D2/D3 cups 2.7, Classic plastic 2.85, WS2
// metal 2.45. 2.5 is what earlier builds used. SET THIS FOR YOUR ROTOR.
#define INSPEED_MPH_PER_HZ   2.5f

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
// Speed is v(t) = K / T_i during revolution i, and 0 when the cups are still.
// Averaging that over the WHOLE window W (calm time included):
//   mean speed  = K * pulses / W
//   mean of v^2 = K^2 * sum(1/T_i) / W
//   speed SD    = sqrt(mean of v^2 - mean^2)
// The earlier version divided by the summed revolution time instead of W, so a
// short burst followed by calm read several times too high, and it counted the
// revolution in progress as complete, so one stray pulse could read 250 mph.
//
// Direction: unit vectors of 100 ms vane samples, skipping samples taken while
// the cups are stopped (no pulse for CALM_US) -- a vane in a lull just drifts.
struct Bucket {
    float    sumInvT;   // sum of 1/T (1/us) for periods ending in this bucket
    int16_t  sinS;      // sum of sin * 1000 (<= 10 samples -> fits)
    int16_t  cosS;      // sum of cos * 1000
    uint8_t  pulses;    // pulses in this bucket (<= 100 at 100 Hz)
    uint8_t  nDir;      // vane samples counted for direction
};
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
        if (T != NO_PERIOD && T > 0) b.sumInvT += 1.0f / (float)T;
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

// Recompute every statistic over the window and publish it for I2C.
void publishWindow() {
    // ---- totals over the window ----
    uint16_t N = 0; float SI = 0.0f;
    int32_t  sS = 0, cS = 0; uint16_t nD = 0;
    for (uint8_t a = 0; a < filledB; a++) {
        const Bucket& b = bk[bucketAgo(a)];
        N += b.pulses; SI += b.sumInvT;
        sS += b.sinS; cS += b.cosS; nD += b.nDir;
    }
    float Wsec = (float)filledB * (TICK_MS * TICKS_PER_BUCKET) / 1000.0f;
    float Wus  = Wsec * 1000000.0f;

    float spdAvg = K_MPH_HZ * (float)N / Wsec;
    float m2     = K_MPH_US * K_MPH_US * SI / Wus;
    float var    = m2 - spdAvg * spdAvg;
    float spdSd  = (var > 0.0f) ? sqrt(var) : 0.0f;

    // ---- gust: highest GUST_BUCKETS-second mean inside the window ----
    float gust = 0.0f;
    uint8_t span = (filledB < GUST_BUCKETS) ? filledB : GUST_BUCKETS;
    float spanSec = (float)span * (TICK_MS * TICKS_PER_BUCKET) / 1000.0f;
    for (uint8_t start = 0; start + span <= filledB; start++) {
        uint16_t gN = 0;
        for (uint8_t k = 0; k < span; k++) gN += bk[bucketAgo(start + k)].pulses;
        float g = K_MPH_HZ * (float)gN / spanSec;
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
    bk[curB] = Bucket{0.0f, 0, 0, 0, 0};
}

// ============================================================================
// I2C
// ============================================================================
// TinyWireS (rambo/TinyWire) calls this once per read, at the address byte, and
// queues into a 16-byte TX buffer. If a read is ever cut short (bus clear,
// timeout), the unsent bytes stay queued: the next frame comes out shifted, and
// once fewer than 12 bytes are free, TinyWireS.write() spins forever inside the
// I2C interrupt. The watchdog below is what gets the chip (and the shared bus)
// back from that. The node's sequence check rejects shifted frames.
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
    for (uint8_t i = 0; i < WINDOW_BUCKETS; i++) bk[i] = Bucket{0.0f, 0, 0, 0, 0};

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
