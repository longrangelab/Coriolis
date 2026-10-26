/**
 * ATtiny85 Wind Sensor  -  SparkFun / Inspeed
 * ===========================================
 * Same firmware for either kit. The wind meter is chosen AT COMPILE TIME with
 * the WIND_METER_ lines below -- uncomment exactly one before flashing. The build
 * fails on purpose if none (or both) are selected, so a node can never be
 * flashed with the wrong calibration by accident.
 *
 * The choice sets both halves of the calibration together:
 *   SparkFun : 1.492 mph per Hz, 8-position resistor-ladder vane table
 *   Inspeed  : 2.5 mph per Hz,   continuous ratiometric E-Vane (5-95% Vcc)
 *
 * (Earlier builds tried to auto-detect the meter at boot by probing the vane.
 *  That misclassified real Inspeed hardware as SparkFun, so it was removed.)
 *
 * LED feedback at boot: 2 blinks = SparkFun, 3 blinks = Inspeed. Check this
 * after flashing -- it confirms which calibration is actually running.
 *
 * The anemometer (single reed/Hall pulse, 1 pulse/rev) is the same for both;
 * only the mph-per-Hz constant differs.
 *
 * What gets reported (refreshed every 1 s, over a rolling 10 s window):
 *   speed_avg : revolutions / elapsed time, from timed pulses (no 100 ms steps)
 *   speed_sd  : time-weighted SD of per-revolution speeds
 *   gust      : highest 3 s mean speed within the window
 *   dir_avg   : unit-vector mean of 100 ms vane samples taken while moving
 *   dir_sd    : Yamartino SD of the same samples
 * In a calm (no pulse for 2 s) the vane is ignored and the last direction is
 * held; speed decays toward 0 as the gap since the last pulse grows.
 *
 * Hardware: ATtiny85 @ 8 MHz internal, 3.3 V, I2C addr 0x42
 * Pins: PB0=SDA, PB2=SCL, PB3=vane analog, PB4=anemometer pulse, PB1=LED
 */

#include <TinyWireS.h>
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
#define CALM_US            2000000UL   // no pulse for 2 s -> cups stopped; skip vane
#define STOPPED_US         60000000UL  // no pulse for 60 s -> forget the last edge
#define PERIOD_RING        8       // ISR -> loop queue (100 Hz max = 10 per tick)
#define DEBOUNCE_US        10000UL // reed debounce: 100 Hz cap (149 mph SparkFun, 250 mph Inspeed)

// ---- Calibration constants for each kit ----
#define SPARKFUN_MPH_PER_HZ  1.492f
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
    uint16_t rawADC;      // vane ADC (debug)
} windData;

// ============================================================================
// SPARKFUN 8-DIRECTION VANE TABLE (precomputed unit vectors)
// ============================================================================
struct VaneCal { uint16_t adc; float cosv; float sinv; };
// In flash (PROGMEM), not RAM: the ATtiny85 only has 512 bytes of RAM.
static const VaneCal vaneTable[] PROGMEM = {
    { 93,  0.0000f, -1.0000f},  // W   270
    {181,  0.7071f, -0.7071f},  // NW  315
    {284,  1.0000f,  0.0000f},  // N     0
    {459, -0.7071f, -0.7071f},  // SW  225
    {628,  0.7071f,  0.7071f},  // NE   45
    {784, -1.0000f,  0.0000f},  // S   180
    {887, -0.7071f,  0.7071f},  // SE  135
    {945,  0.0000f,  1.0000f}   // E    90
};
static const uint8_t vaneTableSize = sizeof(vaneTable) / sizeof(vaneTable[0]);

// Returns unit vector (cos,sin) for the current vane reading, per detected kit.
void decodeVaneVector(int adc, float& cosv, float& sinv) {
    if (sensorType == SENSOR_SPARKFUN) {
        int minDiff = 32767; uint8_t best = 0;
        for (uint8_t i = 0; i < vaneTableSize; i++) {
            int diff = adc - (int)pgm_read_word(&vaneTable[i].adc); if (diff < 0) diff = -diff;
            if (diff < minDiff) { minDiff = diff; best = i; }
        }
        VaneCal c;
        memcpy_P(&c, &vaneTable[best], sizeof(c));
        cosv = c.cosv;
        sinv = c.sinv;
    } else {
        // Inspeed: continuous linear angle -> trig
        float span = (float)(VANE_ADC_AT_360 - VANE_ADC_AT_0);
        float deg  = (span != 0) ? (float)(adc - VANE_ADC_AT_0) * 360.0f / span : 0.0f;
        while (deg < 0.0f)    deg += 360.0f;
        while (deg >= 360.0f) deg -= 360.0f;
        float th = deg * (float)DEG_TO_RAD;
        cosv = cos(th);
        sinv = sin(th);
    }
}

void blink(uint8_t n) {
    for (uint8_t i = 0; i < n; i++) {
        digitalWrite(DEBUG_LED_PIN, HIGH); delay(180);
        digitalWrite(DEBUG_LED_PIN, LOW);  delay(180);
    }
}

// ============================================================================
// ANEMOMETER -- each debounced pulse is TIMESTAMPED in the ISR
// ============================================================================
// The ISR only measures the period between pulses (one pulse = one revolution)
// and queues it. All math happens in loop(). Timing whole revolutions instead
// of counting pulses per 100 ms tick removes the old 14.9 mph (SparkFun) /
// 25 mph (Inspeed) quantization steps from speed SD and gust.
volatile uint32_t periodRing[PERIOD_RING];   // completed periods, us
volatile uint8_t  ringHead = 0, ringCount = 0;
volatile uint32_t lastEdgeUs = 0;
volatile bool     haveEdge   = false;         // false until the first pulse

ISR(PCINT0_vect) {
    if (digitalRead(ANEMOMETER_PIN) != LOW) return;
    uint32_t now = micros();
    if (!haveEdge) { haveEdge = true; lastEdgeUs = now; return; }   // no period yet
    uint32_t dt = now - lastEdgeUs;
    if (dt < DEBOUNCE_US) return;                                     // reed bounce
    lastEdgeUs = now;
    if (ringCount < PERIOD_RING) {
        periodRing[(uint8_t)(ringHead + ringCount) % PERIOD_RING] = dt;
        ringCount++;
    }
}

// ============================================================================
// ROLLING WINDOW: WINDOW_BUCKETS one-second buckets
// ============================================================================
// Each revolution is filed in the bucket where it ENDS. Over the window, with
// per-revolution speed v_i = K / T_i and each revolution weighted by its
// duration T_i (time-weighted, so fast revolutions don't get extra votes):
//   mean speed = K * N / sum(T)              (revolutions / elapsed time)
//   mean of v^2 = K^2 * sum(1/T) / sum(T)
//   speed SD   = sqrt(mean of v^2 - mean^2)
// Direction: unit vectors of 100 ms vane samples, skipping samples taken while
// the cups are stopped (no pulse for CALM_US) -- a vane in a lull just drifts.
struct Bucket {
    uint32_t sumT;      // us, sum of periods ending in this bucket
    float    sumInvT;   // sum of 1/T (1/us)
    int16_t  sinS;      // sum of sin * 1000 (<= 10 samples -> fits)
    int16_t  cosS;      // sum of cos * 1000
    uint8_t  revs;      // revolutions ending in this bucket (<= 100 at 100 Hz)
    uint8_t  nDir;      // vane samples counted for direction
};
static Bucket   bk[WINDOW_BUCKETS];
static uint8_t  curB    = 0;      // bucket being filled
static uint8_t  filledB = 1;      // buckets holding data (window warm-up)
static uint32_t lastPeriodUs = 0; // most recent completed period
static uint16_t lastVaneADC  = 0;
static const float K_MPH_US = anemoMphPerHz * 1000000.0f;  // mph * us per rev

static inline uint8_t bucketAgo(uint8_t ago) {     // 0 = current bucket
    return (uint8_t)((curB + WINDOW_BUCKETS - ago) % WINDOW_BUCKETS);
}

// Move queued periods from the ISR into the current bucket.
void drainPeriods() {
    for (;;) {
        noInterrupts();
        if (ringCount == 0) { interrupts(); return; }
        uint32_t T = periodRing[ringHead];
        ringHead = (uint8_t)(ringHead + 1) % PERIOD_RING;
        ringCount--;
        interrupts();
        Bucket& b = bk[curB];
        if (b.revs < 255) b.revs++;
        b.sumT    += T;
        b.sumInvT += 1.0f / (float)T;
        lastPeriodUs = T;
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

    float vCos, vSin;
    decodeVaneVector(adc, vCos, vSin);
    Bucket& b = bk[curB];
    b.cosS += (int16_t)lroundf(vCos * 1000.0f);
    b.sinS += (int16_t)lroundf(vSin * 1000.0f);
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
    uint16_t N = 0; uint32_t ST = 0; float SI = 0.0f;
    int32_t  sS = 0, cS = 0; uint16_t nD = 0;
    for (uint8_t a = 0; a < filledB; a++) {
        const Bucket& b = bk[bucketAgo(a)];
        N += b.revs; ST += b.sumT; SI += b.sumInvT;
        sS += b.sinS; cS += b.cosS; nD += b.nDir;
    }

    // ---- the revolution in progress ----
    // If it is already longer than the last full one, the wind has slowed (or
    // stopped): the speed now is at most K / elapsed. Count it as that upper
    // bound so a stopped cup decays toward 0 instead of freezing the last value.
    noInterrupts();
    bool     he = haveEdge;
    uint32_t le = lastEdgeUs;
    interrupts();
    uint32_t E = he ? (uint32_t)(micros() - le) : 0;
    if (he && E > STOPPED_US) {                // long calm: forget the stale edge
        noInterrupts(); haveEdge = false; interrupts();
        he = false; lastPeriodUs = 0;
    }
    float num = (float)N, den = (float)ST, inv = SI;
    if (he && E > lastPeriodUs) { num += 1.0f; den += (float)E; inv += 1.0f / (float)E; }

    float spdAvg = 0.0f, spdSd = 0.0f;
    if (num > 0.0f && den > 0.0f) {
        spdAvg = K_MPH_US * num / den;
        float m2  = K_MPH_US * K_MPH_US * inv / den;
        float var = m2 - spdAvg * spdAvg;
        spdSd = (var > 0.0f) ? sqrt(var) : 0.0f;
    }

    // ---- gust: highest GUST_BUCKETS-second mean inside the window ----
    float gust = 0.0f;
    uint8_t span = (filledB < GUST_BUCKETS) ? filledB : GUST_BUCKETS;
    for (uint8_t start = 0; start + span <= filledB; start++) {
        uint16_t gN = 0; uint32_t gT = 0;
        for (uint8_t k = 0; k < span; k++) {
            const Bucket& b = bk[bucketAgo(start + k)];
            gN += b.revs; gT += b.sumT;
        }
        if (gN > 0 && gT > 0) {
            float g = K_MPH_US * (float)gN / (float)gT;
            if (g > gust) gust = g;
        }
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
    out.rawADC    = lastVaneADC;

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
    bk[curB] = Bucket{0, 0.0f, 0, 0, 0, 0};
}

// ============================================================================
// I2C
// ============================================================================
void requestEvent() {
    uint8_t* ptr = (uint8_t*)&windData;
    for (uint8_t i = 0; i < sizeof(windData); i++) TinyWireS.write(ptr[i]);
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
    pinMode(DEBUG_LED_PIN, OUTPUT);
    blink(1);                                   // "alive"

    // ---- report which wind meter this build is calibrated for ----
    delay(300);
    blink(sensorType == SENSOR_INSPEED ? 3 : 2); // 2 = SparkFun, 3 = Inspeed

    windData = {0, 0, 0, 0, 0, 0};
    for (uint8_t i = 0; i < WINDOW_BUCKETS; i++) bk[i] = Bucket{0, 0.0f, 0, 0, 0, 0};

    pinMode(ANEMOMETER_PIN, INPUT);             // external ~10k pull-up required
    PCMSK |= (1 << PCINT4);
    GIMSK |= (1 << PCIE);
    sei();

    TinyWireS.begin(I2C_SLAVE_ADDRESS);
    TinyWireS.onRequest(requestEvent);
}

// ============================================================================
// LOOP
// ============================================================================
void loop() {
    TinyWireS_stop_check();

    unsigned long now = millis();

    static unsigned long lastTick = 0;
    static uint8_t       ticks    = 0;
    if (now - lastTick >= TICK_MS) {
        lastTick += TICK_MS;
        drainPeriods();
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
