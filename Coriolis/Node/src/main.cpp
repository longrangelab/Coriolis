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
#include <esp_sleep.h>
#include <esp_system.h>          // esp_restart() -- I2C recovery last resort

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
// DEEP SLEEP (piece 2 of 3 -- see DEEPSLEEP_v6.md)
// ============================================================================
// The base commands sleep by setting BFLAG_SLEEP_CMD in its beacons. We ack
// (FLAG_SLEEP_ACK on our next TX) and drop into ESP32 deep sleep. On each
// timer wake we listen for a beacon: still commanding sleep -> transmit one
// reading and sleep again; not commanding sleep (or BFLAG_WAKE_CMD) -> stay up.
//
// The DUTY CYCLE -- how long we sleep between check-ins -- is chosen LOCALLY
// from battery voltage and temperature (see chooseDutySec). Local is the right
// place for this decision: the node knows its own battery, and different nodes
// can be in different shape (one shaded, one in sun).
#define SLEEP_DUTY_MIN_S     20      // default duty: warm + charged
#define SLEEP_DUTY_MAX_S     300     // deep-winter floor: check in every 5 min
#define SLEEP_LISTEN_MIN_MS  2500UL  // beacon-listen window bounds on wake
#define SLEEP_LISTEN_MAX_MS  16000UL

// ---- I2C bus recovery -------------------------------------------------------
// The ATtiny wind sensor shares the I2C bus with the PMU and OLED. The classic
// failure is a peripheral left holding SDA low mid-byte, which hangs the bus
// and NACKs every subsequent transfer. Recovery escalates:
//   1) after I2C_NACK_LIMIT consecutive failed reads -> clock the bus free
//      (bit-bang up to 9 SCL pulses so a stuck slave finishes its byte) and
//      re-init Wire. This fixes the common hang WITHOUT losing state.
//   2) if that still doesn't help after I2C_RECOVER_LIMIT attempts in a row ->
//      full esp_restart() as a last resort. A reboot counter prevents a boot
//      loop: a node that can't recover falls back to headless/stale operation
//      and keeps transmitting what it can rather than restarting forever.
#define I2C_NACK_LIMIT       5    // consecutive read fails -> attempt bus clear
#define I2C_RECOVER_LIMIT    4    // bus clears that didn't help -> allow a reset
#define I2C_MAX_RESETS       3    // hard cap on self-resets before giving up

// ---- cold-charge cutoff (protect the Li-ion cell) --------------------------
// Charging a lithium cell below freezing plates lithium: permanent capacity
// loss and, worst case, an internal short. The PMU can gate charging but the
// stock T-Beam has no thermistor on the cell, so we enforce it in firmware from
// the BMP180 ambient reading (a proxy: bias the cutoff a few degrees above 0 C
// since the cell may be colder than the air and the sensor lags). Two thresholds
// give hysteresis so charging doesn't chatter near the boundary. The decision is
// re-evaluated on every sensor read, i.e. every awake cycle and every wake from
// deep sleep -- and the PMU keeps the setting while the ESP32 sleeps.
#define COLD_CHARGE_CUTOFF_C   3.0f  // at/below this ambient, DISABLE charging
#define COLD_CHARGE_RESUME_C   6.0f  // only re-ENABLE once we warm past this
// What to do when the temperature is UNREADABLE (BMP180 glitch, or absent) AND
// the node is asleep. 0 = fail safe: no trusted temp -> no charging (default).
// Awake nodes are exempt from the gate entirely and charge regardless, so this
// only affects a DEPLOYED (sleeping) node -- one with no working BMP180 won't
// charge while deployed. Set to 1 if you ever field a baro-less solar node.
#define COLD_CHARGE_FAILSAFE_ALLOW  0
#define SLEEP_SILENT_LIMIT   3       // this many wakes w/o beacon -> assume base
                                     // off, escalate duty to save power
#define BUTTON_WAKE_GRACE_MS 60000UL // after a button wake, ignore sleep cmds
                                     // this long so a human can read the screen

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

// ---- state that must SURVIVE deep sleep (RTC slow memory, ~8 KB) ----
// Ordinary globals are lost on deep sleep (it's a reboot). Everything the node
// needs to resume without re-doing work lives here: whether we're in sleep
// mode, the duty we chose, the last known TDMA cycle length (sizes the beacon
// listen window), a counter of beacon-less wakes, and -- the big one -- the GPS
// fix. Without the RTC-cached fix, every wake would re-power the GPS and burn
// 30+ s at ~40 mA hunting satellites, which would erase the entire point of
// sleeping.
RTC_DATA_ATTR bool     rtcSleepMode   = false;
RTC_DATA_ATTR uint16_t rtcDutySec     = SLEEP_DUTY_MIN_S;
RTC_DATA_ATTR uint16_t rtcCycleMs     = (MIN_SLOTS + 1) * SLOT_MS;
RTC_DATA_ATTR uint8_t  rtcSilentWakes = 0;
// I2C self-reset counter, persisted so a deployed (sleeping) node can't boot-loop
// on a permanently stuck bus: after I2C_MAX_RESETS it stops resetting and just
// runs degraded. Cleared once a read succeeds (see readAttiny recovery path).
RTC_DATA_ATTR uint8_t  rtcI2cResets   = 0;
// Failure-run counters, RTC-persisted so escalation accumulates even in sleep
// mode -- there the node reads once per wake then deep-sleeps (a reboot), so
// plain statics would reset every wake and never reach the recovery threshold.
RTC_DATA_ATTR uint8_t  rtcNackRun     = 0;   // consecutive failed reads
RTC_DATA_ATTR uint8_t  rtcClearRun    = 0;   // consecutive bus-clears that didn't help
RTC_DATA_ATTR bool     rtcGpsFix      = false;
RTC_DATA_ATTR double   rtcLat = 0, rtcLon = 0;
RTC_DATA_ATTR uint32_t rtcWakeCount   = 0;
// Cold-charge cutoff state, persisted across deep sleep so hysteresis carries
// over between wakes (and matches what the PMU is actually doing). Starts true:
// on a fresh boot charging is allowed until the first reading says otherwise.
RTC_DATA_ATTR bool     rtcChargeAllowed = true;

// GPS (cached once, then powered down)
bool       gpsFixCached = false;
double     cachedLat = 0, cachedLon = 0;

// sleep bookkeeping (normal RAM -- valid only while awake)
bool       quickWake = false;          // this boot is a timer wake from sleep
uint32_t   sleepIgnoreUntil = 0;       // button-wake grace window
bool       pendingSleepAck = false;    // next slot TX should carry the ack

// wind hold-last-good
AttinyWind lastWind = {0,0,0,0,0,0};
bool       haveWind = false;
uint32_t   lastWindMs = 0;              // when the last successful ATtiny read landed

// How long a held-over wind reading stays trustworthy. The ATtiny publishes a
// fresh 2 s window continuously, so anything older than this means the I2C link
// or the meter itself is down -- past that we mark the wind INVALID rather than
// keep transmitting a stale value that looks live.
#define WIND_STALE_MS  30000UL

// most recent reading (for the display)
NodeReading current;

// TDMA state
bool       synced = false;
uint32_t   beaconLocalMs = 0, lastBeaconAnyMs = 0, lastAutoTx = 0;
uint16_t   slotMs = SLOT_MS, cycleMs = (MIN_SLOTS + 1) * SLOT_MS;
uint8_t    maxSlot = MIN_SLOTS;
uint16_t   nodeMask = 0;      // v4: slot map from the last beacon (bit i => id i scheduled)
uint8_t    mySlot = 0;        // v4: our compact slot this cycle (0 = not scheduled yet)
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

// Raw single-shot read of the ATtiny wind sensor. Returns false on any I2C
// short-read (NACK / stuck bus). No recovery here -- that's readAttiny()'s job.
static bool readAttinyRaw(AttinyWind& w) {
    if (Wire.requestFrom(ATTINY_ADDR, (uint8_t)ATTINY_BYTES) != ATTINY_BYTES) return false;
    uint8_t b[ATTINY_BYTES];
    for (uint8_t i = 0; i < ATTINY_BYTES; i++) b[i] = Wire.read();
    w.speed_avg = b[0]|(b[1]<<8); w.speed_sd = b[2]|(b[3]<<8);
    w.dir_avg   = b[4]|(b[5]<<8); w.dir_sd   = b[6]|(b[7]<<8);
    w.gust      = b[8]|(b[9]<<8); w.rawADC   = b[10]|(b[11]<<8);
    return true;
}

// Clock a stuck I2C bus free. A slave that was interrupted mid-byte can hold
// SDA low forever; pulsing SCL lets it finish its transaction and release the
// line. We end the Wire driver, bit-bang up to 9 SCL pulses (a full byte + ack)
// while SDA is released, issue a manual STOP, then re-init Wire. No reboot, no
// lost state -- this alone clears the common hang.
static void i2cBusClear() {
    Wire.end();
    pinMode(I2C_SCL, OUTPUT_OPEN_DRAIN);
    pinMode(I2C_SDA, INPUT_PULLUP);              // release SDA, watch it
    for (uint8_t i = 0; i < 9 && digitalRead(I2C_SDA) == LOW; i++) {
        digitalWrite(I2C_SCL, LOW);  delayMicroseconds(5);
        digitalWrite(I2C_SCL, HIGH); delayMicroseconds(5);
    }
    // manual STOP: SDA low->high while SCL high
    pinMode(I2C_SDA, OUTPUT_OPEN_DRAIN);
    digitalWrite(I2C_SDA, LOW);  delayMicroseconds(5);
    digitalWrite(I2C_SCL, HIGH); delayMicroseconds(5);
    digitalWrite(I2C_SDA, HIGH); delayMicroseconds(5);
    Wire.begin(I2C_SDA, I2C_SCL);
}

// ATtiny read with tiered bus recovery. Most calls just succeed and reset the
// failure counters. On a run of failures it clears the bus; if clearing the bus
// repeatedly fails to help, it allows a bounded self-reset (RTC-counted so a
// permanently stuck node degrades gracefully instead of boot-looping).
bool readAttiny(AttinyWind& w) {
    if (readAttinyRaw(w)) {          // success -> everything resets
        rtcNackRun = 0; rtcClearRun = 0; rtcI2cResets = 0;
        return true;
    }

    if (++rtcNackRun < I2C_NACK_LIMIT) return false;  // tolerate a few, stay degraded

    // Threshold hit -> try to clock the bus free and read once more.
    Serial.printf("[i2c] %u consecutive NACKs -> bus clear\n", rtcNackRun);
    i2cBusClear();
    rtcNackRun = 0;
    if (readAttinyRaw(w)) { rtcClearRun = 0; rtcI2cResets = 0; return true; }

    // Bus clear didn't restore comms.
    if (++rtcClearRun < I2C_RECOVER_LIMIT) return false;
    rtcClearRun = 0;

    // Last resort: bounded self-reset. Skip once the cap is hit so a hard-stuck
    // node keeps running headless/stale rather than restarting forever.
    if (rtcI2cResets < I2C_MAX_RESETS) {
        rtcI2cResets++;
        Serial.printf("[i2c] bus unrecoverable -> self-reset %u/%u\n",
                      rtcI2cResets, I2C_MAX_RESETS);
        Serial.flush();
        esp_restart();               // no return
    }
    Serial.println(F("[i2c] reset cap reached -> running degraded (wind invalid)"));
    return false;
}

// Cold-charge decision with hysteresis. `prevAllowed` is the current state, so
// the band between CUTOFF and RESUME holds whatever we were doing (no chatter).
// Unreadable temperature falls back to COLD_CHARGE_FAILSAFE_ALLOW.
bool decideCharge(float tempC, bool tempValid, bool prevAllowed) {
    if (!tempValid) return (bool)COLD_CHARGE_FAILSAFE_ALLOW;   // no trusted temp
    if (prevAllowed) return tempC >  COLD_CHARGE_CUTOFF_C;     // charging -> cut when cold
    else             return tempC >= COLD_CHARGE_RESUME_C;     // held off -> resume when warm
}

// Apply the cutoff to the PMU, but only when the state actually changes and the
// PMU accepted the call -- so we don't spam I2C, and we retry if the PMU wasn't
// ready. Called at the end of readSensors, i.e. on every awake cycle and on
// every wake from deep sleep, which is exactly the "re-probe on wake" behaviour.
void applyColdChargePolicy(const NodeReading& r) {
    // Cold-charge protection applies ONLY to deployed nodes -- ones in, or
    // entering, sleep mode (the permanently-stationed solar units that sit
    // unattended through winter). An AWAKE node is attended and runs its radio
    // continuously, so it rarely net-charges in the cold and is allowed to charge
    // WITHOUT the temperature gate -- and doesn't need a working BMP180 to charge.
    // (Note: awake does not strictly mean above freezing -- you can shoot below 0
    // -- but an awake node's high, steady draw makes cold-charging a non-issue.)
    bool sleeping = rtcSleepMode || pendingSleepAck;   // pendingSleepAck => this is
                                                       // the transition-to-sleep cycle
    bool allow = sleeping ? decideCharge(r.temperatureC, r.baroValid, rtcChargeAllowed)
                          : true;                      // awake -> unconditional charge
    if (allow == rtcChargeAllowed) return;
    if (power.setChargeEnable(allow)) {
        rtcChargeAllowed = allow;
        if (allow)
            Serial.println(F("[charge] ENABLED"));
        else if (r.baroValid)
            Serial.printf("[charge] DISABLED -- cold-guard (ambient %.1f C <= %.1f)\n",
                          r.temperatureC, (float)COLD_CHARGE_CUTOFF_C);
        else
            Serial.println(F("[charge] DISABLED -- temp unreadable while asleep (fail-safe)"));
    }
}

// Read every sensor into r (called once per transmit cycle).
void readSensors(NodeReading& r) {
    AttinyWind w;
    attinyOk = readAttiny(w);
    if (attinyOk) { lastWind = w; haveWind = true; lastWindMs = millis(); }
    if (haveWind) {
        r.windSpeed   = lastWind.speed_avg / 100.0f;
        r.windSpeedSd = lastWind.speed_sd  / 100.0f;
        r.windDir     = lastWind.dir_avg   / 10.0f;
        r.windDirSd   = lastWind.dir_sd    / 10.0f;
        r.windGust    = lastWind.gust      / 100.0f;
    }
    // Tell the receiver whether to trust these numbers at all. If we've never
    // read the meter, or the last good read has gone stale, the fields above
    // are 0 or old -- flagging that is what stops a dead anemometer from
    // quietly pulling the network's weighted average toward "calm".
    r.windValid = haveWind && ((uint32_t)(millis() - lastWindMs) < WIND_STALE_MS);
    if (!r.windValid && haveWind)
        Serial.println(F("[ATtiny] wind STALE -- transmitting as invalid"));
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

    // Gate charging on temperature (cold-guard) -- but only while asleep or
    // entering sleep; awake nodes charge unconditionally (see applyColdChargePolicy).
    // The PMU holds the setting through deep sleep, so a deployed cell that drops
    // below freezing stays uncharged until it warms.
    applyColdChargePolicy(r);
}

void printReading(const NodeReading& r) {
    float tF = r.temperatureC * 9.0f / 5.0f + 32.0f;    // imperial
    float pHg = r.pressurePa / 3386.389f;
    Serial.println(F("------------------------------------------------"));
    if (synced)
        Serial.printf("Node %d  [TDMA synced]  slot %u/%u\n", nodeId, mySlot, maxSlot);
    else
        Serial.printf("Node %d  [autonomous]\n", nodeId);
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

// ============================================================================
// DEEP SLEEP ENGINE
// ============================================================================
// Pick the sleep duration from battery voltage and temperature.
//
// Design intent (matches the project spec):
//   - warm + solar keeping the pack full  -> 20 s duty ("on briefly every 20s")
//   - pack sagging                        -> stretch toward minutes
//   - below freezing                      -> 5 min floor. Cold is double
//     trouble: Li-ion capacity drops hard below 0 C AND charging a frozen
//     lithium cell damages it, so most solar chargers stop entirely -- the
//     node must assume it's running on stored energy only.
//   - a genuinely full pack (>4.15 V means the panel is actively holding it
//     at absorption voltage) buys back speed even when cold, because energy
//     is provably coming in.
//
// Voltage is read at wake, AFTER the radio has been on for a moment, so it
// reflects loaded voltage -- thresholds are set for that.
uint16_t chooseDutySec(float battV, float tempC, bool tempValid) {
    // Stay at the 20 s baseline until the cell is nearly empty (~10%),
    // then jump to the max to protect what's left. The battery percentage
    // formula (battPctFromVolts) is linear: (v - 3.3) / 0.9 * 100, so
    // 10% corresponds to ~3.39 V; we use 3.40 V as a clean boundary.
    // Previous code started extending sleep at 4.05 V (~83%), which caused
    // 40-90 s cycles even on a healthy, partially-charged cell. Removed.
    uint16_t duty = SLEEP_DUTY_MIN_S;                  // 20 s baseline
    if (battV > 0 && battV < 3.40f)
        duty = SLEEP_DUTY_MAX_S;                       // <= ~10% -- protect battery
    // Temperature overrides (cold conserves more aggressively than low battery).
    if (tempValid) {
        if (tempC < 0.0f)      duty = max(duty, (uint16_t)SLEEP_DUTY_MAX_S);
        else if (tempC < 5.0f) duty = max(duty, (uint16_t)60);
    }
    if (duty < SLEEP_DUTY_MIN_S) duty = SLEEP_DUTY_MIN_S;
    if (duty > SLEEP_DUTY_MAX_S) duty = SLEEP_DUTY_MAX_S;
    return duty;
}

// Enter deep sleep for `sec` seconds. Wakes on the timer OR the center button
// (GPIO 38, active low) so a human can always bring a node up by hand.
void enterDeepSleep(uint16_t sec) {
    Serial.printf("[sleep] entering deep sleep for %u s (wake #%lu)\n",
                  sec, (unsigned long)rtcWakeCount);
    Serial.flush();
    if (screen) { screen->powerOff(); }
    radio.sleepRadio();                    // SX1276 -> ~1 uA sleep mode
    power.gpsPower(false);                 // GPS rail off (fix is in RTC memory)
    esp_sleep_enable_timer_wakeup((uint64_t)sec * 1000000ULL);
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_38, 0);   // button = manual wake
    esp_deep_sleep_start();                // never returns
}

// Read sensors + choose the next duty from the freshest numbers we have.
uint16_t nextDuty(const NodeReading& r) {
    uint16_t d = chooseDutySec(r.batteryVolts, r.temperatureC, r.baroValid);
    rtcDutySec = d;
    return d;
}

// One complete sleep-mode CHECK-IN, run straight from setup() on a timer wake.
// Listens for a beacon, and:
//   beacon w/ SLEEP  -> TX one reading in our slot (ack'd), sleep again
//   beacon w/o SLEEP -> base wants us awake: return and fall into normal loop
//   no beacon at all -> count it; after SLEEP_SILENT_LIMIT, double the duty
//                       (base is off -- conserve until it comes back)
// Never returns except in the stay-awake case.
void sleepCheckIn() {
    rtcWakeCount++;
    uint32_t listenMs = (uint32_t)rtcCycleMs * 2 + 500;
    if (listenMs < SLEEP_LISTEN_MIN_MS) listenMs = SLEEP_LISTEN_MIN_MS;
    if (listenMs > SLEEP_LISTEN_MAX_MS) listenMs = SLEEP_LISTEN_MAX_MS;
    Serial.printf("[sleep] check-in: listening %lu ms for beacon\n", (unsigned long)listenMs);

    uint32_t t0 = millis();
    BeaconPacket b;
    while (millis() - t0 < listenMs) {
        if (radio.pollBeacon(b)) {
            rtcCycleMs = b.cycle_ms;                       // keep the window sized right
            rtcSilentWakes = 0;
            // v5: stay awake only if NEITHER the fleet flag NOR our own per-node
            // bit commands sleep. Either one keeps us asleep.
            bool sleepMe = (b.flags & BFLAG_SLEEP_CMD) || ((b.sleep_mask >> nodeId) & 1u);
            if (!sleepMe) {
                Serial.println(F("[sleep] beacon says WAKE -- staying up"));
                rtcSleepMode = false;
                return;                                    // fall into normal loop
            }
            // Base still wants sleep: transmit one reading in our TDMA slot.
            uint32_t beaconAt = millis();
            // v4: our slot is our rank in node_mask. If we're not scheduled (the
            // base hasn't heard us since this unsynchronized wake, OR -- v5 -- we
            // were per-node-slept and the base already dropped our bit from
            // node_mask), borrow a slot by rank position to squeeze in this one
            // ack. Its owner is almost certainly asleep too, so collisions are rare.
            uint8_t  slotId = beaconSlotOf(b.node_mask, nodeId);
            if (slotId == 0) {
                uint8_t denom = b.max_slot ? b.max_slot : MIN_SLOTS;   // guard /0
                slotId = (uint8_t)(((nodeId - 1) % denom) + 1);
            }
            uint32_t myOffset = (uint32_t)slotId * b.slot_ms;
            while (millis() - beaconAt < myOffset) delay(1);
            NodeReading r; readSensors(r);
            radio.sendData(r, nodeId, /*ackSleep=*/true);
            enterDeepSleep(nextDuty(r));                   // no return
        }
        delay(2);
    }

    // Silence. Base off, out of range, or we straddled its beacon badly.
    rtcSilentWakes++;
    Serial.printf("[sleep] no beacon (%u consecutive)\n", rtcSilentWakes);
    if (rtcSilentWakes >= SLEEP_SILENT_LIMIT) {
        uint16_t d = rtcDutySec * 2;
        if (d > SLEEP_DUTY_MAX_S) d = SLEEP_DUTY_MAX_S;
        rtcDutySec = d;
    }
    enterDeepSleep(rtcDutySec);                            // no return
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
    if (editMode) { nodeId = (nodeId % MAX_NODE_ID) + 1; editLastMs = millis(); if (screen) screen->drawNodeId(nodeId); }
}
void onLongPress() {
    lastActivity = millis();
    if (!displayOn) { wakeDisplay(); return; }
    if (!editMode) { editMode = true; editLastMs = millis(); if (screen) screen->drawNodeId(nodeId); }
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

    // What kind of boot is this? Deep sleep ends in a reboot, so the wake
    // cause is the FIRST thing to look at -- it decides the whole boot path.
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    bool timerWake  = (cause == ESP_SLEEP_WAKEUP_TIMER);
    bool buttonWake = (cause == ESP_SLEEP_WAKEUP_EXT0);
    quickWake = rtcSleepMode && timerWake;

    delay(quickWake ? 50 : 300);          // quick wakes skip the settle delay
    Serial.printf("\n=== Wind Node booting (%s) ===\n",
                  quickWake ? "timer wake" : buttonWake ? "button wake" : "cold boot");

    WiFi.mode(WIFI_OFF); btStop();                     // radios we don't use -> off
    setCpuFrequencyMhz(80);                            // lower CPU clock (battery)

    Wire.begin(I2C_SDA, I2C_SCL);
    randomSeed(esp_random());
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    loadNodeId();
    Serial.printf("Node ID: %d\n", nodeId);

    power.init();

    // Restore the GPS fix cached before sleep -- the single biggest power win
    // of the RTC state. The rail was re-enabled by power.init(); if we already
    // have a position, turn it straight back off.
    if (rtcGpsFix) {
        gpsFixCached = true; cachedLat = rtcLat; cachedLon = rtcLon;
        power.gpsPower(false);
    }

    // ---- QUICK WAKE: no screen, no boot animation, no GPS re-init ----------
    // The CORIOLIS intro is ~1.6 s of full-brightness OLED. Fine once on a
    // cold boot; replayed every 20 s it would be a strobing beacon downrange
    // AND a meaningful chunk of the wake-window power budget. On a timer wake
    // the screen simply never comes on.
    if (quickWake) {
        baroOk = baro.init();
        radio.begin();
        sleepCheckIn();          // returns ONLY if the base wants us awake...

        // ...in which case finish becoming a normal awake node:
        screen = ScreenFactory::create();
        if (screen) screen->begin();
        if (!gpsFixCached) gps.begin();
        lastActivity = millis();
        Serial.println(F("=== running (woken by base) ===\n"));
        return;
    }

    // ---- COLD BOOT / BUTTON WAKE: the full experience -----------------------
    if (buttonWake && rtcSleepMode) {
        // A human pressed the button on a sleeping node: stay awake long
        // enough to be looked at, even though the fleet is in sleep mode.
        sleepIgnoreUntil = millis() + BUTTON_WAKE_GRACE_MS;
        Serial.println(F("[sleep] button wake -- ignoring sleep cmds for 60 s"));
    }

    screen = ScreenFactory::create();
    if (screen) screen->begin();
    baroOk = baro.init();
    if (!gpsFixCached) gps.begin();
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
        nodeMask = b.node_mask;                    // v4 compact slot map
        mySlot   = beaconSlotOf(nodeMask, nodeId); // our rank-based slot (0 = not scheduled)
        synced = true; txThisCycle = false;
        rtcCycleMs = b.cycle_ms;               // keep the wake listen window sized

        // Sleep command from the base. We don't sleep HERE -- we set a flag so
        // the very next slot transmission carries FLAG_SLEEP_ACK, and sleep
        // right after that TX completes. Ack-then-sleep, always in that order:
        // the base needs to hear the confirmation to show "N nodes acked".
        //
        // v5: we sleep if EITHER the fleet-wide flag is set OR our own bit is set
        // in the beacon's per-node sleep_mask. The two are independent overrides;
        // a per-node command sleeps us even while the rest of the fleet is awake.
        bool sleepMe = (b.flags & BFLAG_SLEEP_CMD) || ((b.sleep_mask >> nodeId) & 1u);
        if (sleepMe && millis() >= sleepIgnoreUntil) {
            pendingSleepAck = true;
        } else if (!sleepMe) {
            pendingSleepAck = false;           // base changed its mind pre-TX
            rtcSleepMode = false;
        }
    }

    // ---- 2) transmit scheduling ----
    if (synced) {
        // v4: our slot is our RANK in the beacon's node_mask, not our raw id.
        // mySlot == 0 means the base hasn't scheduled us yet (it hasn't heard us
        // since we (re)joined) -- fall through to the jitter path so it adds us.
        uint32_t myOffset = (uint32_t)mySlot * slotMs;
        bool fits = (mySlot >= 1) && (myOffset + slotMs <= cycleMs);
        if (fits) {
            if (!txThisCycle && (millis() - beaconLocalMs) >= myOffset) {
                NodeReading r; readSensors(r);
                radio.sendData(r, nodeId, pendingSleepAck);
                txThisCycle = true;
                printReading(r);
                if (pendingSleepAck) {          // ack sent -> commit to sleep
                    rtcSleepMode = true;
                    rtcSilentWakes = 0;
                    rtcGpsFix = gpsFixCached; rtcLat = cachedLat; rtcLon = cachedLon;
                    enterDeepSleep(nextDuty(r));  // no return
                }
            }
        } else {
            // not yet in the schedule -> jitter in so the base hears us and adds
            // our bit to node_mask; next beacon we get a real slot.
            if (millis() - lastAutoTx > (uint32_t)cycleMs + random(0, 400)) {
                NodeReading r; readSensors(r);
                radio.sendData(r, nodeId, pendingSleepAck); lastAutoTx = millis();
                if (pendingSleepAck) {
                    rtcSleepMode = true;
                    rtcSilentWakes = 0;
                    rtcGpsFix = gpsFixCached; rtcLat = cachedLat; rtcLon = cachedLon;
                    enterDeepSleep(nextDuty(r));  // no return
                }
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
            rtcGpsFix = true; rtcLat = la; rtcLon = lo;   // survives deep sleep
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
