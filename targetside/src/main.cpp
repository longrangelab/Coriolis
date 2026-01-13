#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_Sensor.h>
#include "SensorQMI8658.hpp"
#include "Telemetry.h"
#include "Motion.h"
#include "Screen.h"
#include "Environment.h"
#include "Location.h"
#include "ExternalNotification.h"
#include "Power.h"

#define TTGO_T_BEAM_V1_1
#define IS_TARGET

#define TARGET_ID 4
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
#define TARGET_ADDRESS TOSTRING(TARGET_ID)
#define USER_ADDRESS "User"

#ifdef TTGO_T_BEAM_V1_1
#define GPS_RX_PIN 34
#define GPS_TX_PIN 12
#else
#define GPS_RX_PIN 9
#define GPS_TX_PIN 8
#define GPS_WAKEUP_PIN 7
#define GPS_1PPS_PIN 6
#endif

#define BUZZER_PORT 32

#ifdef TTGO_T_BEAM_V1_1
#define DIRA 2
#define DIRB 13
#define SPDA 14
#define SPDB 25
#define BUTTON_IMU_SENSITIVITY 38
#else
#define CHANNEL_A 48
#define CHANNEL_B 21
#endif

#ifdef TTGO_T_BEAM_V1_1
#define PMU_SCL 22
#define PMU_SDA 21
#else
#define PMU_SCL 41
#define PMU_SDA 42
#endif

#ifndef TTGO_T_BEAM_V1_1
#define LORA_SCK 12
#define LORA_MISO 13
#define LORA_MOSI 11
#define LORA_CS 10
#endif

#define SPI_MOSI (35)
#define SPI_SCK  (36)
#define SPI_MISO (37)
#define IMU_CS   (34)
#define IMU_INT  (33)

Screen *screen = nullptr;
Motion *motion = nullptr;
bool hitFlag = false;
WindEnvironment *environment = nullptr;
GPSModule *gps = nullptr;
double current_longtitude = 0.0;
double current_latitude = 0.0;

LoRaModule *loraModule = nullptr;
ExternalNotification *externalNotification = nullptr;
PowerManagement *pmu = nullptr;

#define SPI_FREQ 4000000
SPIClass LoRaSPI(HSPI);
SPISettings spiSettings(SPI_FREQ, MSBFIRST, SPI_MODE0);

// -------------------- runtime flags & state --------------------
volatile bool changeSensitivityFlag = false;
volatile bool imuISRFlag = false;             // minimal ISR for IMU pin (if used)

// Sensitivity button ISR
void IRAM_ATTR handleSensitivityButton() {
    static unsigned long lastPress = 0;
    unsigned long now = millis();
    if (now - lastPress > 300) {
        changeSensitivityFlag = true;
        lastPress = now;
    }
}

// Minimal IMU ISR: only set a flag. Motion classes read their own interrupt flags.
void IRAM_ATTR imuPinISR() {
    imuISRFlag = true;
}

// -------------------- alert queue + retry (non-blocking) --------------------
volatile bool alertPending = false;
char alertPayload[64] = {0};

// Retry state
volatile bool alertRetryPending = false;
unsigned long alertRetryTime = 0;
int alertRetryCount = 0;
const int ALERT_MAX_RETRIES = 1;      // send original + 1 retry
const unsigned long ALERT_RETRY_DELAY = 100; // ms between sends

// Sequence counter to help receiver dedupe
volatile uint32_t alertSeq = 0;

// queue an alert (builds payload with timestamp and seq)
void queueAlert(const char *basePayload) {
    noInterrupts();
    alertSeq++;
    unsigned long ts = millis();
    snprintf(alertPayload, sizeof(alertPayload), "%s:%lu:%u", basePayload, ts, (unsigned)alertSeq);
    alertPending = true;
    alertRetryPending = false;
    alertRetryCount = 0;
    interrupts();
}

// Synchronous single send attempt; suspend I2C briefly around TX to reduce bus contention
volatile unsigned long suspendI2CUntil = 0;
bool sendAlertOnceNow() {
    if (!loraModule) return false;
    // suspend I2C reads for a short window to avoid bus contention during TX
    suspendI2CUntil = millis() + 60; // 60 ms suspend window
    bool ok = loraModule->sendAlert(alertPayload, USER_ADDRESS);
    return ok;
}

// -------------------- timing & cadence --------------------
const unsigned long IMU_SAMPLE_INTERVAL_MS = 10; // 100 Hz
unsigned long lastImuSample = 0;

unsigned long lastMotionTime = 0;
const unsigned long motionDebounceDelay = 2000;

unsigned long lastScreenUpdate = 0;
const unsigned long SCREEN_UPDATE_INTERVAL = 500;

unsigned long lastEnvAttempt = 0;
const unsigned long ENVIRONMENT_CHECK_INTERVAL = 250;
unsigned long lastEnvSent = 0;
const unsigned long ENV_MIN_INTERVAL = 5000; // ms between environment sends
unsigned long lastEnvSpamLog = 0;

// Hit display timing
unsigned long hitDisplayTime = 0;
const unsigned long HIT_DISPLAY_DURATION = 2000; // ms

// IMU reconnect attempts
unsigned long lastImuReconnectAttempt = 0;
const unsigned long IMU_RECONNECT_INTERVAL_MS = 5000; // try reinit every 5s

// -------------------- I2C recovery helpers --------------------
const int SDA_PIN = PMU_SDA;
const int SCL_PIN = PMU_SCL;

void i2cBusRecover() {
    Serial.println("[I2C] Attempting bus recovery...");
    // End Wire to reset driver state
    Wire.end();
    delay(10);

    // Toggle SCL up to 9 times to free a stuck slave
    pinMode(SCL_PIN, OUTPUT);
    for (int i = 0; i < 9; ++i) {
        digitalWrite(SCL_PIN, HIGH);
        delayMicroseconds(5);
        digitalWrite(SCL_PIN, LOW);
        delayMicroseconds(5);
    }
    pinMode(SCL_PIN, INPUT_PULLUP);
    pinMode(SDA_PIN, INPUT_PULLUP);
    delay(5);

    // Restart Wire at 100kHz for robustness
    Wire.begin(SDA_PIN, SCL_PIN, 100000);
    delay(10);
    Serial.println("[I2C] Bus recovery complete.");
}

// -------------------- setup --------------------
void setup()
{
    delay(2000);
    Serial.begin(115200);

    Serial.println(" ================= TARGET CONFIGURATION =================");
    Serial.printf("TARGET ID: %d\n", TARGET_ID);
    Serial.printf("TARGET ADDRESS: %s\n", TARGET_ADDRESS);
    Serial.printf("USER ADDRESS: %s\n", USER_ADDRESS);
    Serial.println("========================================================");

#ifdef IS_TARGET
    // Start Wire at 100 kHz to improve reliability on noisy/long wiring
    Wire.begin(PMU_SDA, PMU_SCL, 100000);
#endif

    externalNotification = new Buzzer(BUZZER_PORT);

    screen = ScreenFactory::createScreen();
    screen->begin();
    Serial.println("Screen instance created");

    pmu = new AXPManagement(PMU_SDA, PMU_SCL);
    if (pmu->init())
        Serial.println("Power Management Unit initialized successfully.");
    else
        Serial.println("Failed to initialize Power Management Unit.");

    gps = new GPSModule(Serial1, GPS_RX_PIN, GPS_TX_PIN);
    gps->begin();
    gps->getCoordinates(current_latitude, current_longtitude);

    // Auto-detect IMU
    Serial.println("Auto-detecting IMU...");
    motion = MotionFactory::autoDetect();
    if (!motion) {
        Serial.println("FATAL: No IMU detected. Halting.");
        while (true) delay(1000);
    }
    Serial.println("IMU auto-detection complete.");
    Serial.print("Initial IMU Sensitivity: ");
    Serial.println(motion->getSensitivity());

    // Calibrate IMU (short static calibration)
    motion->calibrate();

    // Attach minimal ISR for IMU interrupt pin (if used by QMI)
    pinMode(IMU_INT, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(IMU_INT), imuPinISR, CHANGE);

    // Sensitivity button
    pinMode(BUTTON_IMU_SENSITIVITY, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BUTTON_IMU_SENSITIVITY),
                    handleSensitivityButton, FALLING);

    environment = new WindEnvironment(DIRA, DIRB, SPDA, SPDB);
    environment->begin();

#ifndef TTGO_T_BEAM_V1_1
    LoRaSPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
#endif

#ifdef IS_TARGET
#ifndef TTGO_T_BEAM_V1_1
    loraModule = new LoRaRadioBoards(TARGET_ADDRESS, LoRaSPI, spiSettings);
#else
    loraModule = new LoRaRadioBoards(TARGET_ADDRESS);
#endif
#else
#ifndef TTGO_T_BEAM_V1_1
    loraModule = new LoRaRadioBoards(USER_ADDRESS, LoRaSPI, spiSettings);
#else
    loraModule = new LoRaRadioBoards(USER_ADDRESS);
#endif
#endif

    loraModule->begin();

    float batteryPercentage = pmu->getBatteryPercentage();
    // Keep original drawInterface usage (no region clearing changes)
    screen->drawInterface(batteryPercentage, 0, false, 0,
                          TARGET_ADDRESS, 0, 0, 1);
    lastScreenUpdate = millis();
    lastImuSample = millis();
}

// -------------------- main loop --------------------
void loop()
{
    unsigned long now = millis();

    // IMU sampling at steady cadence with connection check and reconnect attempts
    if (now - lastImuSample >= IMU_SAMPLE_INTERVAL_MS) {
        lastImuSample = now;

        // Skip IMU reads if we recently suspended I2C for TX
        if (millis() < suspendI2CUntil) {
            // skip this IMU sample to avoid bus contention
        } else {
            bool motionDetected = false;

            if (!motion) {
                // no IMU object
            } else {
                bool connected = motion->isConnected();
                if (!connected) {
                    // Attempt reconnect periodically
                    if (now - lastImuReconnectAttempt >= IMU_RECONNECT_INTERVAL_MS) {
                        lastImuReconnectAttempt = now;
                        Serial.println("[IMU] Not connected - attempting bus recovery + reinit...");
                        // Try bus recovery first
                        i2cBusRecover();
                        // Then attempt reinit
                        if (motion->begin()) {
                            Serial.println("[IMU] Reinit succeeded. Running calibrate...");
                            motion->calibrate();
                        } else {
                            Serial.println("[IMU] Reinit failed or still not present.");
                        }
                    }
                    // Skip detection while disconnected
                } else {
                    // Normal detection path
                    motionDetected = motion->detectMotion();

                    if (motionDetected) {
                        if (now - lastMotionTime > motionDebounceDelay) {
                            lastMotionTime = now;
                            hitFlag = true;
                            hitDisplayTime = millis();   // set display timer

                            // queue alert (non-blocking) with seq+timestamp
                            queueAlert("Hit!!!");

                            // immediate local notify
                            if (externalNotification) externalNotification->notify();

                            // immediate screen update (existing drawInterface)
                            float batteryPercentage = pmu->getBatteryPercentage();
                            screen->drawInterface(batteryPercentage, 0, true, 0,
                                                  TARGET_ADDRESS,
                                                  environment ? environment->getWindDirection() : 0,
                                                  environment ? environment->getWindSpeed() : 0,
                                                  motion->getSensitivity());

                            Serial.println("[Target] Motion detected and queued alert.");
                        } else {
                            // suppressed by debounce (throttle log)
                            static unsigned long lastSuppressedLog = 0;
                            if (now - lastSuppressedLog > 2000) {
                                Serial.println("[Target] Motion detected but suppressed by debounce.");
                                lastSuppressedLog = now;
                            }
                        }
                    }
                }
            }
        }
    }

    // Handle sensitivity button (non-ISR)
    if (changeSensitivityFlag) {
        changeSensitivityFlag = false;
        if (motion) {
            motion->increaseSensitivity();
            // Force a screen update to show new sensitivity
            float batteryPercentage = pmu->getBatteryPercentage();
            screen->drawInterface(batteryPercentage, 0, false, 0,
                                  TARGET_ADDRESS,
                                  environment ? environment->getWindDirection() : 0,
                                  environment ? environment->getWindSpeed() : 0,
                                  motion->getSensitivity());
        }
    }

    // Send queued alert (first send + schedule retry)
    if (alertPending) {
        noInterrupts();
        bool pending = alertPending;
        alertPending = false;
        // schedule retry
        alertRetryPending = true;
        alertRetryTime = millis() + ALERT_RETRY_DELAY;
        alertRetryCount = 0;
        interrupts();

        if (pending) {
            bool ok = sendAlertOnceNow();
            Serial.print("[Target] sendAlert (first) -> ");
            Serial.println(ok ? "OK" : "FAIL");
        }
    }

    // Handle scheduled retry(s) without blocking
    if (alertRetryPending && millis() >= alertRetryTime) {
        bool ok = sendAlertOnceNow();
        Serial.print("[Target] sendAlert (retry) -> ");
        Serial.println(ok ? "OK" : "FAIL");

        alertRetryCount++;
        if (alertRetryCount >= ALERT_MAX_RETRIES) {
            alertRetryPending = false;
        } else {
            alertRetryTime = millis() + ALERT_RETRY_DELAY;
        }
    }

    // Environment telemetry (rate-limited and time-slot gated)
    if (now - lastEnvAttempt >= ENVIRONMENT_CHECK_INTERVAL) {
        lastEnvAttempt = now;

        // Decide whether to send environment telemetry
        unsigned long cycleTime = now % 10000;
        unsigned long slotStart = (TARGET_ID - 1) * 1000;
        unsigned long slotEnd = (slotStart + 1000) % 10000;
        bool inTimeSlot = false;
        if (slotStart < slotEnd)
            inTimeSlot = (cycleTime >= slotStart && cycleTime < slotEnd);
        else
            inTimeSlot = (cycleTime >= slotStart || cycleTime < slotEnd);

        bool enoughTimePassed = (now - lastEnvSent) >= ENV_MIN_INTERVAL;

        if (inTimeSlot && enoughTimePassed) {
            int windDir = environment ? environment->getWindDirection() : 0;
            int windSpeed = environment ? environment->getWindSpeed() : 0;
            double lat = 0, lon = 0;
            gps->getCoordinates(lat, lon);

            // Suspend I2C briefly around environment send to reduce contention
            suspendI2CUntil = millis() + 40;

            bool ok = loraModule->sendEnvironment(windSpeed, 0, windDir, lat, lon, motion->getSensitivity(), USER_ADDRESS);
            if (ok) {
                lastEnvSent = now;
            } else {
                Serial.println("[Target] Environment send failed.");
            }
        } else {
            // throttle spam logs
            if (now - lastEnvSpamLog > 5000) {
                Serial.println("[Target] Environment data sent too recently, skipping");
                lastEnvSpamLog = now;
            }
        }
    }

    // Periodic screen refresh (keep original drawInterface)
    if (now - lastScreenUpdate >= SCREEN_UPDATE_INTERVAL) {
        lastScreenUpdate = now;

        // Clear hit flag after display duration
        if (hitFlag && (millis() - hitDisplayTime > HIT_DISPLAY_DURATION)) {
            hitFlag = false;
        }

        float batteryPercentage = pmu->getBatteryPercentage();
        screen->drawInterface(batteryPercentage, 0, hitFlag, 0,
                              TARGET_ADDRESS,
                              environment ? environment->getWindDirection() : 0,
                              environment ? environment->getWindSpeed() : 0,
                              motion ? motion->getSensitivity() : 0);
    }

    // Keep loop responsive
    yield();
}
