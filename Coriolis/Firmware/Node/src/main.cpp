/**
 * Wind Node  -  LiLyGO T-Beam (ESP32, v1.1 or v1.2)
 * =================================================
 * TDMA node: listens for the base-station BEACON, transmits its NodePacket in
 * its slot, and falls back to autonomous jittered transmit if no beacon.
 *
 * Battery savers:
 *   - OLED sleeps after 20 s; wake with the center button (GPIO 38)
 *   - GPS powers DOWN after the first fix (stationary meter needs position once)
 *   - CPU dropped to 80 MHz, WiFi/BT off
 *   - Sensors read once per transmit cycle (not on a fast timer)
 *
 * Creature comfort:
 *   - Change the node number in the field: long-press to enter SET NODE ID,
 *     short-press to increment, long-press to save (persists in flash/NVS).
 *     No reflashing to renumber a meter.
 */

#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <WiFi.h>

#include "screen.h"
#include "Power.h"
#include "Environment.h"          // BMP085Sensor (baro) + GPSModule (GPS)
#include "telemetry.h"

//#define USE_TSL2591            // uncomment when the TSL2591 is installed
#ifdef USE_TSL2591
  #include "Light.h"
#endif

// ============================================================================
// CONFIGURATION
// ============================================================================
#define DEFAULT_NODE_ID     1        // used only until you set/save one via button
#define MAX_NODE_ID         15

#define I2C_SDA             21
#define I2C_SCL             22
#define GPS_RX_PIN          34
#define GPS_TX_PIN          12
#define ATTINY_ADDR         0x42
#define ATTINY_BYTES        12
#define BUTTON_PIN          38       // center/user button (active low)

#define DISPLAY_TIMEOUT_MS  20000UL  // OLED sleeps after this idle time
#define LONG_PRESS_MS       1200UL
#define EDIT_TIMEOUT_MS     8000UL

#define GPS_SLEEP_AFTER_FIX 1        // 1 = power GPS down once we have a position
#define AUTO_TX_MS          3000UL   // fallback transmit period (no beacon)
#define AUTO_JITTER_MS      500

// ============================================================================
// ATtiny wire struct (must match attiny85_wind_sensor.ino)
// ============================================================================
struct AttinyWind { uint16_t speed_avg, speed_sd, dir_avg, dir_sd, gust, rawADC; };

// ============================================================================
// GLOBALS
// ============================================================================
Screen*        screen = nullptr;
AXPManagement  power(I2C_SDA, I2C_SCL);
BMP085Sensor   baro;
GPSModule      gps(Serial1, GPS_RX_PIN, GPS_TX_PIN);
NodeRadio      radio;
Preferences    prefs;
#ifdef USE_TSL2591
  LightSensor  light;
#endif

uint8_t    nodeId = DEFAULT_NODE_ID;
bool       baroOk = false, attinyOk = false;

// GPS (cached once, then powered down)
bool       gpsFixCached = false;
double     cachedLat = 0, cachedLon = 0;

// wind hold-last-good
AttinyWind lastWind = {0,0,0,0,0,0};
bool       haveWind = false;

// most recent reading (for the display)
NodeReading current;

// TDMA state
bool       synced = false;
uint32_t   beaconLocalMs = 0, lastBeaconAnyMs = 0, lastAutoTx = 0;
uint16_t   slotMs = SLOT_MS, cycleMs = (MIN_SLOTS + 1) * SLOT_MS;
uint8_t    maxSlot = MIN_SLOTS;
bool       txThisCycle = false;

// UI / display
bool       displayOn = true, editMode = false;
uint32_t   lastActivity = 0, editLastMs = 0, lastDraw = 0;

// button
int        btnPrev = HIGH;
uint32_t   pressStart = 0;
bool       longFired = false;

// ============================================================================
// HELPERS
// ============================================================================
const char* dirName(int deg) {
    static const char* n[] = {"N","NE","E","SE","S","SW","W","NW"};
    int i = ((deg + 22) / 45) % 8; if (i < 0) i += 8; return n[i];
}

bool readAttiny(AttinyWind& w) {
    if (Wire.requestFrom(ATTINY_ADDR, (uint8_t)ATTINY_BYTES) != ATTINY_BYTES) return false;
    uint8_t b[ATTINY_BYTES];
    for (uint8_t i = 0; i < ATTINY_BYTES; i++) b[i] = Wire.read();
    w.speed_avg = b[0]|(b[1]<<8); w.speed_sd = b[2]|(b[3]<<8);
    w.dir_avg   = b[4]|(b[5]<<8); w.dir_sd   = b[6]|(b[7]<<8);
    w.gust      = b[8]|(b[9]<<8); w.rawADC   = b[10]|(b[11]<<8);
    return true;
}

// Read every sensor into r (called once per transmit cycle).
void readSensors(NodeReading& r) {
    AttinyWind w;
    attinyOk = readAttiny(w);
    if (attinyOk) { lastWind = w; haveWind = true; }
    if (haveWind) {
        r.windSpeed   = lastWind.speed_avg / 100.0f;
        r.windSpeedSd = lastWind.speed_sd  / 100.0f;
        r.windDir     = lastWind.dir_avg   / 10.0f;
        r.windDirSd   = lastWind.dir_sd    / 10.0f;
        r.windGust    = lastWind.gust      / 100.0f;
    }
    if (baroOk) {
        r.temperatureC = baro.readTemperatureC();
        r.pressurePa   = baro.readPressurePa();
        r.baroValid    = (r.pressurePa > 30000.0f && r.pressurePa < 120000.0f);
    }
#ifdef USE_TSL2591
    r.lightValid = light.readLux(r.lightLux);
#else
    r.lightValid = false;
#endif
    r.gpsValid  = gpsFixCached;
    r.latitude  = cachedLat;
    r.longitude = cachedLon;
    r.batteryVolts = power.getBatteryVoltage();
    current = r;
}

void printReading(const NodeReading& r) {
    float tF = r.temperatureC * 9.0f / 5.0f + 32.0f;    // imperial
    float pHg = r.pressurePa / 3386.389f;
    Serial.println(F("------------------------------------------------"));
    Serial.printf("Node %d  %s\n", nodeId, synced ? "[TDMA synced]" : "[autonomous]");
    Serial.printf("  Wind speed   : %5.2f mph  (SD %4.2f)  gust %.2f\n",
                  r.windSpeed, r.windSpeedSd, r.windGust);
    Serial.printf("  Wind dir     : %5.1f deg %-2s (SD %4.1f)\n",
                  r.windDir, dirName((int)r.windDir), r.windDirSd);
    if (r.baroValid) Serial.printf("  Temp / Baro  : %.1f F   %.2f inHg\n", tF, pHg);
    else             Serial.println(F("  Temp / Baro  : --  (BMP180 absent)"));
    if (r.lightValid) Serial.printf("  Light        : %.1f lux\n", r.lightLux);
    else              Serial.println(F("  Light        : --  (no TSL2591)"));
    if (r.gpsValid) Serial.printf("  GPS          : %.6f, %.6f%s\n", r.latitude, r.longitude,
                                  GPS_SLEEP_AFTER_FIX ? " (cached, GPS off)" : "");
    else            Serial.println(F("  GPS          : -- (acquiring)"));
    Serial.printf("  Battery      : %.2f V (%d%%)\n",
                  r.batteryVolts, (int)power.getBatteryPercentage());
}

// ---- node id persistence ----
void loadNodeId() {
    prefs.begin("windnode", true);
    nodeId = prefs.getUChar("nodeid", DEFAULT_NODE_ID);
    prefs.end();
    if (nodeId < 1 || nodeId > MAX_NODE_ID) nodeId = DEFAULT_NODE_ID;
}
void saveNodeId() {
    prefs.begin("windnode", false);
    prefs.putUChar("nodeid", nodeId);
    prefs.end();
    Serial.printf("[UI] node id saved: %d\n", nodeId);
}

// ---- display helpers ----
void drawMain() {
    if (!displayOn || !screen) return;
    int battPct = (int)power.getBatteryPercentage(); if (battPct < 0) battPct = 0;
    screen->drawNode(nodeId, battPct, -999, current.windSpeed,
                     (int)(current.windDir + 0.5f), current.gpsValid);
}
void wakeDisplay() {
    if (!screen) return;
    if (!displayOn) { screen->powerOn(); displayOn = true; }
    lastActivity = millis();
    editMode ? screen->drawNodeId(nodeId) : drawMain();
}
void sleepDisplay() {
    if (screen && displayOn) { screen->powerOff(); displayOn = false; }
}

// ---- button ----
void onShortPress() {
    lastActivity = millis();
    if (!displayOn) { wakeDisplay(); return; }        // asleep -> just wake
    if (editMode) { nodeId = (nodeId % MAX_NODE_ID) + 1; editLastMs = millis(); screen->drawNodeId(nodeId); }
}
void onLongPress() {
    lastActivity = millis();
    if (!displayOn) { wakeDisplay(); return; }
    if (!editMode) { editMode = true; editLastMs = millis(); screen->drawNodeId(nodeId); }
    else           { saveNodeId(); editMode = false; drawMain(); }
}
void pollButton() {
    int level = digitalRead(BUTTON_PIN);              // LOW = pressed
    if (level == LOW && btnPrev == HIGH) { pressStart = millis(); longFired = false; }
    if (level == LOW && !longFired && millis() - pressStart >= LONG_PRESS_MS) {
        onLongPress(); longFired = true;
    }
    if (level == HIGH && btnPrev == LOW && !longFired) onShortPress();
    btnPrev = level;
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.println(F("\n=== Wind Node booting ==="));

    WiFi.mode(WIFI_OFF); btStop();                     // radios we don't use -> off
    setCpuFrequencyMhz(80);                            // lower CPU clock (battery)

    Wire.begin(I2C_SDA, I2C_SCL);
    randomSeed(esp_random());
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    loadNodeId();
    Serial.printf("Node ID: %d\n", nodeId);

    power.init();
    screen = ScreenFactory::create();
    if (screen) screen->begin();
    baroOk = baro.init();
    gps.begin();
#ifdef USE_TSL2591
    light.begin();
#endif

    Wire.beginTransmission(ATTINY_ADDR);
    Serial.println(Wire.endTransmission() == 0 ? F("[ATtiny] found") : F("[ATtiny] NOT found"));

    radio.begin();

    lastActivity = millis();
    Serial.println(F("=== running ===\n"));
}

// ============================================================================
// LOOP
// ============================================================================
void loop() {
    // ---- 1) beacon sync ----
    BeaconPacket b;
    if (radio.pollBeacon(b)) {
        beaconLocalMs = millis(); lastBeaconAnyMs = millis();
        slotMs = b.slot_ms; cycleMs = b.cycle_ms; maxSlot = b.max_slot;
        synced = true; txThisCycle = false;
    }

    // ---- 2) transmit scheduling ----
    if (synced) {
        uint32_t myOffset = (uint32_t)nodeId * slotMs;
        bool fits = (nodeId <= maxSlot) && (myOffset + slotMs <= cycleMs);
        if (fits) {
            if (!txThisCycle && (millis() - beaconLocalMs) >= myOffset) {
                NodeReading r; readSensors(r);
                radio.sendData(r, nodeId);
                txThisCycle = true;
                printReading(r);
            }
        } else {
            // our id is beyond the current cycle -> jitter in so the base grows it
            if (millis() - lastAutoTx > (uint32_t)cycleMs + random(0, 400)) {
                NodeReading r; readSensors(r);
                radio.sendData(r, nodeId); lastAutoTx = millis();
            }
        }
        if (millis() - lastBeaconAnyMs > 3UL * cycleMs) synced = false;   // lost base
    } else {
        // ---- autonomous fallback (no beacon heard) ----
        if (millis() - lastAutoTx > AUTO_TX_MS + (uint32_t)random(0, AUTO_JITTER_MS)) {
            NodeReading r; readSensors(r);
            radio.sendData(r, nodeId); lastAutoTx = millis();
            printReading(r);
        }
    }

    // ---- 3) GPS: acquire once, cache, power down ----
    if (!gpsFixCached) {
        double la = 0, lo = 0;
        gps.getCoordinates(la, lo);
        if (la != 0 && lo != 0) {
            cachedLat = la; cachedLon = lo; gpsFixCached = true;
            Serial.printf("[GPS] fix cached: %.6f, %.6f\n", cachedLat, cachedLon);
#if GPS_SLEEP_AFTER_FIX
            power.gpsPower(false);
            Serial.println(F("[GPS] powered down (stationary meter)"));
#endif
        }
    }

    // ---- 4) button + display ----
    pollButton();
    if (editMode && millis() - editLastMs > EDIT_TIMEOUT_MS) { saveNodeId(); editMode = false; drawMain(); }
    if (displayOn && !editMode && millis() - lastActivity > DISPLAY_TIMEOUT_MS) sleepDisplay();
    if (displayOn && !editMode && millis() - lastDraw > 500) { lastDraw = millis(); drawMain(); }
}
