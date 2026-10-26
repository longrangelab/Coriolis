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

#define TICK_MS            100UL
#define TICKS_PER_WINDOW   20      // 20 x 100 ms = 2.0 s window
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
static const VaneCal vaneTable[] = {
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
            int diff = adc - (int)vaneTable[i].adc; if (diff < 0) diff = -diff;
            if (diff < minDiff) { minDiff = diff; best = i; }
        }
        cosv = vaneTable[best].cosv;
        sinv = vaneTable[best].sinv;
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
// ANEMOMETER (pulse counted in ISR) -- identical for both kits
// ============================================================================
volatile uint16_t      pulseCount = 0;
volatile unsigned long lastEdgeUs = 0;

ISR(PCINT0_vect) {
    if (digitalRead(ANEMOMETER_PIN) == LOW) {
        unsigned long now = micros();
        if (now - lastEdgeUs >= DEBOUNCE_US) { pulseCount++; lastEdgeUs = now; }
    }
}

// ============================================================================
// WINDOW ACCUMULATORS
// ============================================================================
static float    sumSin = 0, sumCos = 0;
static float    spdSum = 0, spdSumSq = 0, gustMax = 0;
static uint8_t  tickCount = 0;
static uint16_t lastPulseSnapshot = 0, lastVaneADC = 0;

void resetWindow() {
    sumSin = sumCos = 0; spdSum = spdSumSq = 0; gustMax = 0; tickCount = 0;
}

void sampleTick() {
    int adc = analogRead(WIND_VANE_PIN);
    lastVaneADC = (uint16_t)adc;
    float vCos, vSin;
    decodeVaneVector(adc, vCos, vSin);
    sumCos += vCos; sumSin += vSin;

    noInterrupts();
    uint16_t pulseNow = pulseCount;
    interrupts();
    uint16_t delta = pulseNow - lastPulseSnapshot;
    lastPulseSnapshot = pulseNow;

    float binHz  = (float)delta * (1000.0f / (float)TICK_MS);
    float binMph = binHz * anemoMphPerHz;
    spdSum   += binMph;
    spdSumSq += binMph * binMph;
    if (binMph > gustMax) gustMax = binMph;

    tickCount++;
}

void finalizeWindow() {
    uint8_t n = tickCount;
    if (n == 0) return;

    float meanSin = sumSin / n, meanCos = sumCos / n;
    float dirAvg = atan2(meanSin, meanCos) * (float)RAD_TO_DEG;
    if (dirAvg < 0) dirAvg += 360.0f;

    float R = sqrt(meanSin * meanSin + meanCos * meanCos);
    if (R > 1.0f) R = 1.0f;
    float eps   = sqrt(1.0f - R * R);
    float dirSd = asin(eps) * (1.0f + 0.1547f * eps * eps * eps) * (float)RAD_TO_DEG;

    float spdAvg = spdSum / n;
    float var    = (spdSumSq / n) - (spdAvg * spdAvg);
    if (var < 0) var = 0;
    float spdSd  = sqrt(var);

    windData.speed_avg = (uint16_t)(spdAvg  * 100.0f + 0.5f);
    windData.speed_sd  = (uint16_t)(spdSd   * 100.0f + 0.5f);
    windData.dir_avg   = (uint16_t)(dirAvg  * 10.0f  + 0.5f);
    windData.dir_sd    = (uint16_t)(dirSd   * 10.0f  + 0.5f);
    windData.gust      = (uint16_t)(gustMax * 100.0f + 0.5f);
    windData.rawADC    = lastVaneADC;

    resetWindow();
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

    pinMode(ANEMOMETER_PIN, INPUT);             // external ~10k pull-up required
    PCMSK |= (1 << PCINT4);
    GIMSK |= (1 << PCIE);
    sei();

    TinyWireS.begin(I2C_SLAVE_ADDRESS);
    TinyWireS.onRequest(requestEvent);

    windData = {0, 0, 0, 0, 0, 0};
    resetWindow();
    lastPulseSnapshot = 0;
}

// ============================================================================
// LOOP
// ============================================================================
void loop() {
    TinyWireS_stop_check();

    unsigned long now = millis();

    static unsigned long lastTick = 0;
    if (now - lastTick >= TICK_MS) {
        lastTick += TICK_MS;
        sampleTick();
        if (tickCount >= TICKS_PER_WINDOW) finalizeWindow();
    }

    static unsigned long lastBeat = 0;
    if (now - lastBeat >= 2000) {
        digitalWrite(DEBUG_LED_PIN, !digitalRead(DEBUG_LED_PIN));
        lastBeat = now;
    }
}
