/**
 * Wind Receiver  -  LiLyGO T-Beam (ESP32)
 * =======================================
 * Central station. Listens for NodePackets from every wind node, keeps a table
 * of the most recent reading per node, and shows it on the OLED.
 *
 * Views (cycle with the middle/user button, GPIO 38):
 *   GENERAL  -> average wind speed + direction across all active nodes (startup).
 *   NODE n   -> that node's 2 s wind speed, direction, battery, distance, RSSI.
 *
 * Distance is computed from THIS receiver's GPS vs the node's GPS using the
 * Haversine helper in Location.h (returns yards).
 *
 * Compatibility: decodes the exact NodePacket the node transmitter sends.
 * Reuses your real Power.h (AXPManagement) and Location.h (GPSModule).
 *
 * Debug: every received packet and a periodic node table are printed to Serial.
 */

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <Preferences.h>

#include "Screen.h"
#include "Power.h"
#include "Environment.h"          // BMP085Sensor (baro) + GPSModule (GPS)
#include "Telemetry.h"
#include "Ballistics.h"            // standalone ballistic solver
#include "WindStats.h"             // rolling wind history + stability
// Cap the wind-sensitivity CURVE resolution. windSensitivity() runs one full
// trajectory integration per segment, so its cost is ~quadratic in range
// (segment count AND each shot's length both grow). The default cap (64) makes a
// 1500 yd curve ~57 s on this chip. Capping at 32 segments roughly halves that
// (~32 s) for ~2.6% curve error -- imperceptible on the bar chart, and it only
// affects the sensitivity CURVE, never the firing solution (solve()). Tune to
// taste: 24 -> ~24 s (~5% err), 16 -> ~16 s (~8% err), 64 -> full resolution.
// This overrides the #ifndef default inside Sensitivity.h; no header edit needed.
#define SENS_MAX_SEG 32
#include "Sensitivity.h"           // Phase 2: wind-sensitivity zones -> weights
#include "WebUI.h"                 // phone dashboard (WiFi AP)

// ============================================================================
// CONFIGURATION
// ============================================================================
#define I2C_SDA        21
#define I2C_SCL        22
#define GPS_RX_PIN     34
#define GPS_TX_PIN     12
#define BUTTON_PIN     38          // T-Beam user/middle button (active low)

#define MAX_NODES          16      // node ids 1..15
#define NODE_TIMEOUT_MS    60000UL // drop a node from "active" after 60 s silence

// ============================================================================
// FLEET SLEEP (piece 3 of 3 -- see DEEPSLEEP_v6.md)
// ============================================================================
// While fleetSleep is true every beacon carries BFLAG_SLEEP_CMD. Nodes ack and
// go down; each one then wakes on ITS OWN duty (20 s .. 5 min, chosen locally
// from its battery + temperature), listens for one beacon, transmits one
// reading, sleeps again. So during fleet sleep the beacon keeps running
// exactly as before -- it IS the check-in channel.
//
// Waking: pressing WAKE clears the flag and raises BFLAG_WAKE_CMD on every
// beacon for WAKE_BEACON_MS. 30 s covers a full default duty cycle with margin,
// so a 20 s node is caught on its very next check-in. Nodes that stretched to
// 5 min (cold/low battery) catch it later -- but the wake STATE is what nodes
// obey, not the 30 s burst: any check-in after WAKE is pressed finds the sleep
// flag gone and stays up. The burst just makes the common case feel instant
// and gives the UI a countdown to show.
//
// A sleeping node is silent for minutes at a time, so the normal 60 s activity
// timeout would evict the whole fleet from every list. Sleeping nodes get a
// timeout of 2x the max duty + margin instead.
#define WAKE_BEACON_MS        30000UL
#define SLEEP_NODE_TIMEOUT_MS 660000UL   // 2 x 300 s duty + 60 s margin

// ============================================================================
// AIMBOT: windage stepper controller (a SECOND ATtiny85, on THIS bus)
// ============================================================================
// Distinct from the node's wind-sensor ATtiny (0x42, different board). This one
// sits on the receiver's I2C bus and drives the scope's windage turret via a
// step/dir driver. The ESP32 sends it an absolute windage hold; the ATtiny does
// the click/step math and the motion. See attiny85_aimbot_stepper.ino.
#define AIMBOT_ADDR        0x33          // clear of PMU (0x34/0x35) and node (0x42)
#define AIM_CMD_SET_HOLD   0x01
#define AIM_CMD_GO         0x02
#define AIM_CMD_SET_AND_GO 0x03
#define AIM_CMD_ZERO       0x04
#define AIM_CMD_SET_ORIGIN 0x05
#define AIM_ST_MOVING      0x01
#define AIM_ST_AT_TARGET   0x02
#define AIM_ST_BADCSUM     0x04
#define AIM_ST_CLAMPED     0x08

bool aimbotPresent = false;              // probed at boot

// --- windage-stepper I2C helpers (were referenced but never defined) ---------
// Transport only; all motion/click math lives on the ATtiny (see the frame spec
// in attiny85_aimbot_stepper.ino): a fixed 4-byte little-endian frame
//   [0]=cmd  [1]=milliMOA low  [2]=milliMOA high  [3]=XOR of bytes 0..2
// and a single status byte read back. aimSend returns false on I2C NACK so the
// caller can report an error instead of assuming the turret moved.
void aimProbe() {
    Wire.beginTransmission(AIMBOT_ADDR);
    aimbotPresent = (Wire.endTransmission() == 0);
    Serial.println(aimbotPresent ? F("[aimbot] windage stepper FOUND at 0x33")
                                 : F("[aimbot] windage stepper absent"));
}

bool aimSend(uint8_t cmd, int16_t milliMoa) {
    uint8_t lo = (uint8_t)(milliMoa & 0xFF);
    uint8_t hi = (uint8_t)((milliMoa >> 8) & 0xFF);
    uint8_t frame[4] = { cmd, lo, hi, (uint8_t)(cmd ^ lo ^ hi) };  // XOR checksum
    Wire.beginTransmission(AIMBOT_ADDR);
    Wire.write(frame, sizeof(frame));
    return Wire.endTransmission() == 0;          // 0 = ACK
}

uint8_t aimStatus() {
    if (!aimbotPresent) return 0;                // don't poll an absent device
    if (Wire.requestFrom((uint8_t)AIMBOT_ADDR, (uint8_t)1) != 1) return 0;
    return (uint8_t)Wire.read();
}

#define SCREEN_REFRESH_MS  500UL
#define STATUS_PRINT_MS    5000UL
#define BATTERY_MS         2000UL
#define GPS_MS             1000UL
#define BUTTON_DEBOUNCE_MS 200UL

// ---- serial verbosity ------------------------------------------------------
// The per-packet human-readable block is ~700-1000 bytes. At 115200 baud that
// is ~60-90 ms of mostly-blocking TX *per received packet*, which is a big
// chunk of a 450 ms TDMA slot. That stall is a prime suspect for the GPS UART
// overflow described in the handoff (default ESP32 RX buffer is 256 B = ~250 ms
// at 9600 baud), and it delays beacon transmission.
//
// VERBOSE_SERIAL 0 = machine-parseable DATA line only (recommended for field
//                    use and for any GPS debugging)
//                1 = full human-readable block as before (bench use)
#ifndef VERBOSE_SERIAL
#define VERBOSE_SERIAL 0
#endif

// Phone dashboard access point (join this WiFi, open http://192.168.4.1)
#define AP_SSID  "WindStation"
#define AP_PASS  "windwind12"      // must be >= 8 chars

// ============================================================================
// NODE TABLE
// ============================================================================
struct NodeView {
    bool     seen = false;
    float    windSpeed = 0, windSpeedSd = 0;
    float    windDir = 0,   windDirSd = 0, windGust = 0;
    float    temperatureC = 0, pressurePa = 0;
    float    lightLux = 0;  bool lightValid = false;
    bool     windValid = true;   // v2 nodes don't report it -> assume valid
    float    batteryV = 0;
    double   lat = 0, lon = 0;
    bool     gpsValid = false;
    float    rssi = 0, snr = 0;
    uint32_t lastSeen = 0;
    bool     sleeping = false;    // last packet carried FLAG_SLEEP_ACK
};
NodeView nodes[MAX_NODES];

// ============================================================================
// GLOBALS
// ============================================================================
Screen*          screen = nullptr;
AXPManagement    pmu(I2C_SDA, I2C_SCL);
GPSModule        gps(Serial1, GPS_RX_PIN, GPS_TX_PIN);
BMP085Sensor     baro;                  // base-station BMP180 (optional)
LoRaReceiver     radio;
WebUI            webui;                  // phone dashboard
Ballistics       solver;                 // ballistic solver (standalone)
WindStats        windStats;              // rolling wind history + stability
Sensitivity      sens;                   // Phase 2: sensitivity-zone weighting

// --- async sensitivity worker (P0 fix) --------------------------------------
// windSensitivity() (called by sens.update) runs 40+ software-double trajectory
// integrations -- 2-4 s on this chip's single-precision FPU. Done inline in the
// HTTP handler it froze ALL of loop(): no radio drain, no beacons (fleet drops
// sync), no screen, for the whole sweep. We now run that one call on a one-shot
// task pinned to CORE 0, so loop() (core 1) keeps servicing everything. The
// handler answers immediately; the phone polls until the curve is ready.
//
// Concurrency contract (single producer / single consumer):
//   * Only handleSensitivity() (core 1, one request at a time) starts a job.
//   * Only sensWorker (core 0) writes `sens` -- and only while state==RUNNING.
//   * The handler reads `sens` only when state==DONE and the key matches, at
//     which point the worker has already deleted itself. No overlapping access.
enum SensJobState : uint8_t { SENS_IDLE = 0, SENS_RUNNING = 1, SENS_DONE = 2 };
static volatile SensJobState sensState  = SENS_IDLE;
static SemaphoreHandle_t     sensLock   = nullptr;   // guards state + key strings
static BallInput             sensPendingIn;          // inputs the worker chews on
static String                sensPendingKey = "";    // key of the in-flight job
static String                sensDoneKey    = "";    // key the ready curve is for
static volatile bool         sensDoneOk     = false; // did that finished job produce a valid curve?

// Key over the ballistic inputs that actually change the curve. Wind is excluded
// on purpose -- it does not affect the trajectory, only the windage output -- so
// the curve is reused across every wind edit (matches the client's sbSensKey).
static String sensKeyOf(const BallInput& in) {
    char b[176];
    snprintf(b, sizeof(b),
             "%.1f|%.4f|%d|%.1f|%.4f|%.3f|%.3f|%.3f|%.1f|%.1f|%.1f|%.2f",
             (double)in.mv_fps, (double)in.bc, (int)in.dragModel,
             (double)in.weight_gr, (double)in.cal_in, (double)in.twist_in,
             (double)in.blen_in, (double)in.sightHt_in, (double)in.zero_yd,
             (double)in.range_yd, (double)in.tempF, (double)in.presInHg);
    return String(b);
}

// Runs ONCE on core 0, then deletes itself. sensPendingIn / sensPendingKey were
// filled and state set to RUNNING before this task was created, so they're stable
// for the duration (the handler will not start another job while RUNNING).
static void sensWorker(void*) {
    // windSensitivity() is an uninterruptible multi-second block we cannot chunk
    // (it lives in Ballistics.h). Pinned to core 0 it starves the core-0 idle
    // task, and this build subscribes IDLE0 to the Task Watchdog -> the WDT
    // panics and reboots (exactly what happened on hardware). Exempt core 0's
    // idle WDT for just the span of the sweep, then restore it. The sweep is
    // bounded/deterministic (run() caps its step count), so the unwatched window
    // is finite. loop() on core 1 keeps its own watchdog and keeps running.
    disableCore0WDT();
    sens.update(sensPendingIn);            // the multi-second sweep -- off loop()
    enableCore0WDT();
    if (sensLock) xSemaphoreTake(sensLock, portMAX_DELAY);
    sensDoneKey = sensPendingKey;
    sensDoneOk  = sens.ready();            // false = solver could not converge
    sensState   = SENS_DONE;
    if (sensLock) xSemaphoreGive(sensLock);
    vTaskDelete(nullptr);
}

double  rxLat = 0, rxLon = 0;
int     rxBatteryPct = 0;
float   rxBatteryV = 0;
float   baseTempF = 0, basePresInHg = 0; // base-station atmosphere
bool    baroOk = false;
int     viewCursor = 0;                 // 0 = general, 1..N = nth active node
uint32_t lastRxMs = 0;                  // last valid packet (RX watchdog)

#define RX_WATCHDOG_MS  20000UL         // no packets this long -> re-arm receiver
#define RX_RESTART_MS   90000UL         // still nothing -> full radio restart

volatile bool buttonFlag = false;
void IRAM_ATTR onButton() { buttonFlag = true; }

uint32_t lastBeacon = 0;                // moved out of loop() so long-running
                                        // handlers can bracket themselves

// ---- fleet sleep state ----
Preferences rxPrefs;
bool     fleetSleep = false;            // persisted: survives a receiver reboot.
                                        // Critical -- if the base rebooted into
                                        // "awake" while the fleet slept, the
                                        // next check-ins would wake everyone.
uint32_t wakeUntil = 0;                 // WAKE_CMD broadcast window end

// ---- per-node sleep state (v5) ----
// Bit i set => node id i is individually commanded to sleep, LAYERED ON TOP of the
// fleet flag: effectiveSleep(i) = fleetSleep || (nodeSleepMask bit i). Persisted
// for the same reason fleetSleep is -- a base that rebooted into "all awake" would
// otherwise resurrect nodes the user had deliberately parked. The fleet WAKE burst
// does NOT clear these bits; a per-node override outlives a fleet wake by design.
uint16_t nodeSleepMask = 0;

void saveFleetSleep() {
    rxPrefs.begin("rxcfg", false);
    rxPrefs.putBool("fsleep", fleetSleep);
    rxPrefs.end();
}

void saveNodeSleep() {
    rxPrefs.begin("rxcfg", false);
    rxPrefs.putUShort("nsleep", nodeSleepMask);
    rxPrefs.end();
}

// ============================================================================
// HELPERS
// ============================================================================
// Per-node activity timeout. Only a node that has ACTUALLY acked sleep
// (nv.sleeping == true) earns the extended timeout -- it is genuinely expected
// to be silent for up to its max deep-sleep duty (5 min). All other nodes,
// including ones that haven't yet acked a fleet-sleep command, keep the normal
// 60 s awake timeout.
//
// BUG FIXED: the previous condition used (nv.sleeping || fleetSleep), which
// resurrected ANY previously-seen node into buildActive() the moment fleet sleep
// was enabled (their timeout jumped from 60 s to 11 min). This inflated nTot in
// the UI and showed ghost nodes in the "X/N acked" counter. Removing fleetSleep
// from the condition means only confirmed-sleeping nodes stay in the active set
// between their infrequent check-ins; stale nodes drop at the normal 60 s.
// The beacon sleep/wake commands are still broadcast to all nodes in range
// regardless, so no sleeping node is ever missed by the command.
static inline uint32_t nodeTimeoutFor(const NodeView& nv) {
    return nv.sleeping ? SLEEP_NODE_TIMEOUT_MS : NODE_TIMEOUT_MS;
}

uint8_t buildActive(uint8_t* out) {
    uint8_t n = 0;
    uint32_t now = millis();
    for (uint8_t id = 1; id < MAX_NODES; id++)
        if (nodes[id].seen && (now - nodes[id].lastSeen) < nodeTimeoutFor(nodes[id]))
            out[n++] = id;
    return n;
}

// Active nodes whose WIND is trustworthy. Used for every wind average; the
// plain buildActive() list is still used for display/telemetry, so a node with
// a dead anemometer stays visible on the UI (with its own tab and battery/RSSI)
// instead of vanishing -- it just stops voting on the wind call.
uint8_t buildActiveWind(uint8_t* out) {
    uint8_t n = 0;
    uint32_t now = millis();
    for (uint8_t id = 1; id < MAX_NODES; id++)
        if (nodes[id].seen && (now - nodes[id].lastSeen) < nodeTimeoutFor(nodes[id]) && nodes[id].windValid)
            out[n++] = id;
    return n;
}

int battPctFromVolts(float v) {
    if (v <= 0) return -1;
    int p = (int)((v - 3.3f) / (4.2f - 3.3f) * 100.0f + 0.5f);
    if (p < 0) p = 0; if (p > 100) p = 100;
    return p;
}

// Receiver<->node distance in yards.
//   >=0 : yards
//   -1  : receiver has no GPS fix
//   -2  : node has no GPS fix
long nodeDistanceYards(const NodeView& nv) {
    if (rxLat == 0 || rxLon == 0) return -1;
    if (!nv.gpsValid || nv.lat == 0 || nv.lon == 0) return -2;
    double d = GPSModule::calculateDistance(rxLat, rxLon, nv.lat, nv.lon);
    return (d >= 0) ? (long)d : -1;   // >= : a node AT the receiver is 0 yd, not an error
}

// Emit the COMPLETE record for one node: a human-readable block plus a single
// machine-parseable CSV line (the "DATA,..." line) for the ballistic-solver UI.
// Each record is self-contained: it carries the base-station atmosphere + GPS
// alongside the node's wind, so the solver gets everything in one line.
void emitNodeData(uint8_t id) {
    NodeView& nv = nodes[id];
    long dist      = nodeDistanceYards(nv);
    long distCsv   = (dist >= 0) ? dist : -1;   // CSV: <0 all mean "unknown"
    float nTempF   = nv.temperatureC * 9.0f / 5.0f + 32.0f;
    float nPresHg  = nv.pressurePa / 3386.389f;
    int   dirI     = (int)(nv.windDir   + 0.5f);
    int   dirSdI   = (int)(nv.windDirSd + 0.5f);
    bool  baseFix  = (rxLat != 0 && rxLon != 0);

#if VERBOSE_SERIAL
    Serial.printf("\n----- Node %d @ %lu ms -----\n", id, (unsigned long)millis());
    Serial.printf("  Wind   : %.2f mph (SD %.2f)  gust %.2f mph\n",
                  nv.windSpeed, nv.windSpeedSd, nv.windGust);
    Serial.printf("  Dir    : %d deg %s (SD %d)\n", dirI, cardinal(dirI), dirSdI);
    if (nv.lightValid)
        Serial.printf("  NodeEnv: %.1f F  %.2f inHg  %.1f lux\n", nTempF, nPresHg, nv.lightLux);
    else
        Serial.printf("  NodeEnv: %.1f F  %.2f inHg  light -- (sensor future)\n", nTempF, nPresHg);
    Serial.printf("  NodeGPS: %.6f, %.6f (%s)\n",
                  nv.lat, nv.lon, nv.gpsValid ? "fix" : "no fix");
    if (dist >= 0)
        Serial.printf("  Node   : bat %.2f V (%d%%)  dist %ld yd  sig %d dBm / %.1f dB\n",
                      nv.batteryV, battPctFromVolts(nv.batteryV), dist, (int)nv.rssi, nv.snr);
    else
        Serial.printf("  Node   : bat %.2f V (%d%%)  dist -- (%s)  sig %d dBm / %.1f dB\n",
                      nv.batteryV, battPctFromVolts(nv.batteryV),
                      (dist == -1) ? "rx no GPS" : "node no GPS", (int)nv.rssi, nv.snr);
    Serial.printf("  Base   : %.6f, %.6f (%s)  %.1f F  %.2f inHg  bat %d%%\n",
                  rxLat, rxLon, baseFix ? "fix" : "no fix",
                  baseTempF, basePresInHg, rxBatteryPct);
#endif // VERBOSE_SERIAL

    // ---- single parseable line for the web UI / ballistic solver ----
    Serial.printf("DATA,t=%lu,node=%d,wind_mph=%.2f,wind_sd=%.2f,gust_mph=%.2f,"
                  "dir_deg=%d,dir_sd=%d,node_tempF=%.1f,node_presInHg=%.2f,node_lux=%.1f,"
                  "node_bat_V=%.2f,node_fix=%d,node_lat=%.6f,node_lon=%.6f,dist_yd=%ld,"
                  "rssi=%d,snr=%.1f,base_fix=%d,base_lat=%.6f,base_lon=%.6f,"
                  "base_tempF=%.1f,base_presInHg=%.2f,base_bat_pct=%d\n",
                  (unsigned long)millis(), id, nv.windSpeed, nv.windSpeedSd, nv.windGust,
                  dirI, dirSdI, nTempF, nPresHg, nv.lightValid ? nv.lightLux : -1.0f,
                  nv.batteryV, nv.gpsValid ? 1 : 0, nv.lat, nv.lon, distCsv,
                  (int)nv.rssi, nv.snr, baseFix ? 1 : 0, rxLat, rxLon,
                  baseTempF, basePresInHg, rxBatteryPct);
}

void ingestPacket(const NodePacket& p, float rssi, float snr) {
    if (p.node_id == 0 || p.node_id >= MAX_NODES) return;
    NodeView& nv = nodes[p.node_id];
    nv.seen        = true;
    nv.windSpeed   = p.wind_speed    / 100.0f;
    nv.windSpeedSd = p.wind_speed_sd / 100.0f;
    nv.windDir     = p.wind_dir      / 10.0f;
    nv.windDirSd   = p.wind_dir_sd   / 10.0f;
    nv.windGust    = p.wind_gust     / 100.0f;
    nv.temperatureC= p.temperature   / 100.0f;
    nv.pressurePa  = p.pressure;
    nv.lightLux    = p.light         / 100.0f;
    nv.lightValid  = (p.flags & FLAG_LIGHT_VALID);
    // Packet v3+ carries FLAG_WIND_VALID. A v2 node never sets the bit, so
    // reading it blindly would mark every legacy node's wind invalid and zero
    // out the averages -- exactly the failure we're trying to prevent. Gate on
    // version so mixed-firmware networks degrade to the old behaviour instead.
    nv.windValid   = (p.version >= 3) ? ((p.flags & FLAG_WIND_VALID) != 0) : true;
    nv.batteryV    = (p.battery_mv > 0 && p.battery_mv < 5000) ? p.battery_mv / 1000.0f : 0;
    nv.gpsValid    = (p.flags & FLAG_GPS_VALID);
    nv.lat         = p.latitude  / 1e7;
    nv.lon         = p.longitude / 1e7;
    nv.rssi        = rssi;
    nv.snr         = snr;
    // FLAG_SLEEP_ACK is set on the packet a node sends immediately BEFORE
    // sleeping -- both the first ack and every subsequent check-in. So this
    // bit tracks sleep state exactly: present = "about to sleep again",
    // absent = "staying awake".
    nv.sleeping    = (p.flags & FLAG_SLEEP_ACK) != 0;
    nv.lastSeen    = millis();
    lastRxMs       = millis();          // feed the RX watchdog

    emitNodeData(p.node_id);
}

// ============================================================================
// VIEWS
// ============================================================================
void drawCurrentView() {
    uint8_t active[MAX_NODES];
    uint8_t n = buildActive(active);

    if (viewCursor > n) viewCursor = 0;      // clamp if a node dropped off

    bool rxGps = (rxLat != 0 && rxLon != 0);

    if (viewCursor == 0 || n == 0) {
        // GENERAL: averages across active nodes with TRUSTWORTHY wind only.
        // `n` (all active) still drives the tab list and node count; `nw` is
        // the subset that votes on the wind figure.
        uint8_t activeW[MAX_NODES];
        uint8_t nw = buildActiveWind(activeW);
        float sumSpeed = 0, sx = 0, sy = 0;
        float sumTempF = 0, sumPresHg = 0; int nEnv = 0;
        for (uint8_t i = 0; i < nw; i++) {
            NodeView& nv = nodes[activeW[i]];
            sumSpeed += nv.windSpeed;
            float r = nv.windDir * (float)DEG_TO_RAD;
            sx += cosf(r); sy += sinf(r);
            if (nv.pressurePa > 0) {                 // only nodes with a baro
                sumTempF  += nv.temperatureC * 9.0f / 5.0f + 32.0f;
                sumPresHg += nv.pressurePa / 3386.389f;
                nEnv++;
            }
        }
        float avgSpeed = (nw > 0) ? sumSpeed / nw : 0;
        int   avgDir   = 0;
        if (nw > 0) { avgDir = (int)(atan2f(sy, sx) * RAD_TO_DEG + 0.5f); if (avgDir < 0) avgDir += 360; }
        float avgTempF = (nEnv > 0) ? sumTempF  / nEnv : 0;
        float avgPresHg= (nEnv > 0) ? sumPresHg / nEnv : 0;
        screen->drawGeneral(nw, avgSpeed, avgDir, avgTempF, avgPresHg,
                            rxBatteryPct, rxBatteryV, rxGps);
    } else {
        uint8_t id = active[viewCursor - 1];
        NodeView& nv = nodes[id];
        int ageSec = (int)((millis() - nv.lastSeen) / 1000);
        screen->drawNode(id, nv.windSpeed, nv.windSpeedSd,
                         (int)(nv.windDir + 0.5f), (int)(nv.windDirSd + 0.5f),
                         nv.temperatureC * 9.0f / 5.0f + 32.0f, nv.pressurePa / 3386.389f,
                         battPctFromVolts(nv.batteryV), nv.batteryV,
                         nodeDistanceYards(nv), nv.rssi, ageSec);
    }
}

void printStatusSummary() {
    uint8_t active[MAX_NODES];
    uint8_t n = buildActive(active);
    Serial.printf("[status] %lu ms | active nodes: %d (", (unsigned long)millis(), n);
    for (uint8_t i = 0; i < n; i++) Serial.printf("%d%s", active[i], (i < n - 1) ? "," : "");
    Serial.printf(") | base: %.1fF %.2finHg %s bat %d%%\n",
                  baseTempF, basePresInHg, (rxLat != 0 && rxLon != 0) ? "GPSok" : "noGPS",
                  rxBatteryPct);
}

// --- v7 TDMA: compact slot assignment ---------------------------------------
// A node's slot is its RANK among the active ids, not its raw id. The beacon
// carries node_mask (bit i => id i is scheduled); the receiver and every node
// run the SAME beaconSlotOf()/beaconSlotCount() on that mask, so ids {1,3,10}
// take slots {1,2,3} -- three slots, not eleven. This removes the empty padding
// that a gapped id set used to create (which slowed every meter) AND, together
// with the trailing guard slot below, removes the beacon abutment that made the
// highest-id node freeze for ~60 s and re-join in a loop.
//
// The cycle is: slot 0 = beacon, slots 1..count = active nodes, then one empty
// GUARD slot so the last (compact) node never ends on top of the next beacon --
// beacon propagation plus the node's own pre-TX latency (readSensors(): I2C to
// the ATtiny, baro, battery) would otherwise slide it into the half-duplex
// beacon TX and the base would be deaf to it. A MIN_SLOTS floor keeps a minimum
// cadence and leaves room for a not-yet-scheduled node to jitter in cleanly.
//
// Wire change: BeaconPacket gained node_mask, so PACKET_VERSION is now 4. Data
// (NodePacket) is unchanged and still decodes across v3/v4; only the beacon
// differs, so a v3/v4 mix simply won't TDMA-sync (nodes fall to autonomous
// jittered TX -- the safe degradation). Reflash the whole network.
#define BEACON_GUARD_SLOTS 1

// Bitmap of node ids currently active (same liveness test as buildActive()).
uint16_t buildNodeMask() {
    uint16_t m = 0;
    uint32_t now = millis();
    for (uint8_t id = 1; id < MAX_NODES; id++)
        if (nodes[id].seen && (now - nodes[id].lastSeen) < nodeTimeoutFor(nodes[id]))
            m |= (uint16_t)(1u << id);
    return m;
}

// Beacon-to-beacon period for a given number of scheduled node slots.
static inline uint16_t cycleMsForSlots(uint8_t slotCount) {
    uint8_t cap = slotCount < MIN_SLOTS ? MIN_SLOTS : slotCount;   // cadence floor
    return (uint16_t)((cap + 1 + BEACON_GUARD_SLOTS) * SLOT_MS);   // +beacon +guard
}

// Everything the beacon coordinator needs, computed once from the active set.
struct BeaconSched { uint16_t mask; uint8_t count; uint16_t cycleMs; };
static inline BeaconSched computeSched() {
    BeaconSched s;
    // A per-node-slept node gives up its slot: clearing its bit here re-ranks the
    // survivors into a SHORTER cycle automatically (identical to that node having
    // gone offline). This is why per-node sleep needs no new slot math -- the
    // rank-based scheme already compacts around any missing id. The node still
    // receives its sleep command via the beacon's independent sleep_mask, which is
    // broadcast regardless of node_mask membership (see sendBeacon / loop()).
    s.mask    = (uint16_t)(buildNodeMask() & ~nodeSleepMask);
    s.count   = beaconSlotCount(s.mask);        // real number of scheduled nodes
    s.cycleMs = cycleMsForSlots(s.count);
    return s;
}

// Build the JSON the phone dashboard polls (/data.json). Imperial units.
String buildDataJson() {
    uint8_t active[MAX_NODES];
    uint8_t n = buildActive(active);

    // Wind average excludes nodes flagged wind-invalid; environment average
    // still uses every node with a working baro, since a dead anemometer says
    // nothing about its barometer.
    uint8_t activeW[MAX_NODES];
    uint8_t nw = buildActiveWind(activeW);

    float sumSpd = 0, sx = 0, sy = 0, sumSpdSd = 0, sumDirSd = 0, sumT = 0, sumP = 0;
    int   nEnv = 0;
    for (uint8_t i = 0; i < nw; i++) {
        NodeView& nv = nodes[activeW[i]];
        sumSpd += nv.windSpeed; sumSpdSd += nv.windSpeedSd; sumDirSd += nv.windDirSd;
        float r = nv.windDir * (float)DEG_TO_RAD; sx += cosf(r); sy += sinf(r);
    }
    for (uint8_t i = 0; i < n; i++) {
        NodeView& nv = nodes[active[i]];
        if (nv.pressurePa > 0) { sumT += nv.temperatureC*9.0f/5.0f+32.0f; sumP += nv.pressurePa/3386.389f; nEnv++; }
    }
    float avgSpeed = nw ? sumSpd/nw : 0;
    int   avgDir = 0;
    if (nw) { avgDir = (int)(atan2f(sy, sx) * RAD_TO_DEG + 0.5f); if (avgDir < 0) avgDir += 360; }
    float avgSpdSd = nw ? sumSpdSd/nw : 0, avgDirSd = nw ? sumDirSd/nw : 0;
    float avgT = nEnv ? sumT/nEnv : 0, avgP = nEnv ? sumP/nEnv : 0;
    bool baseFix = (rxLat != 0 && rxLon != 0);

    String j = "{\"base\":{";
    j += "\"lat\":" + String(rxLat,6) + ",\"lon\":" + String(rxLon,6);
    j += ",\"tempF\":" + String(baseTempF,1) + ",\"presInHg\":" + String(basePresInHg,2);
    j += ",\"battPct\":" + String(rxBatteryPct) + ",\"battV\":" + String(rxBatteryV,2);
    j += ",\"gps\":" + String(baseFix ? "true":"false") + "},";
    j += "\"avg\":{\"count\":" + String(nw) + ",\"seen\":" + String(n) + ",\"speed\":" + String(avgSpeed,2);
    j += ",\"speedSd\":" + String(avgSpdSd,2) + ",\"dir\":" + String(avgDir);
    j += ",\"dirSd\":" + String(avgDirSd,1) + ",\"tempF\":" + String(avgT,1);
    j += ",\"presInHg\":" + String(avgP,2) + "},\"nodes\":[";
    for (uint8_t i = 0; i < n; i++) {
        NodeView& nv = nodes[active[i]];
        long dist = nodeDistanceYards(nv);
        j += "{\"id\":" + String(active[i]);
        j += ",\"speed\":" + String(nv.windSpeed,2) + ",\"speedSd\":" + String(nv.windSpeedSd,2);
        j += ",\"dir\":" + String((int)(nv.windDir+0.5f)) + ",\"dirSd\":" + String(nv.windDirSd,1);
        j += ",\"gust\":" + String(nv.windGust,2);
        j += ",\"tempF\":" + String(nv.temperatureC*9.0f/5.0f+32.0f,1);
        j += ",\"presInHg\":" + String(nv.pressurePa/3386.389f,2);
        j += ",\"lux\":" + String(nv.lightValid ? nv.lightLux : -1.0f, 1);
        j += ",\"battV\":" + String(nv.batteryV,2) + ",\"battPct\":" + String(battPctFromVolts(nv.batteryV));
        j += ",\"lat\":" + String(nv.lat,6) + ",\"lon\":" + String(nv.lon,6);
        j += ",\"distYd\":" + String(dist) + ",\"rssi\":" + String((int)nv.rssi);
        j += ",\"age\":" + String((int)((millis()-nv.lastSeen)/1000));
        j += ",\"windOk\":" + String(nv.windValid ? "true" : "false");
        j += ",\"sleeping\":" + String(nv.sleeping ? "true" : "false");
        // sleepCmd = individually COMMANDED to sleep (distinct from "sleeping",
        // which is the node having ACKED and gone quiet). The UI toggle reflects
        // sleepCmd; "sleeping" still shows whether the node has actually acked.
        j += ",\"sleepCmd\":" + String(((nodeSleepMask >> active[i]) & 1u) ? "true" : "false") + "}";
        if (i < n-1) j += ",";
    }
    j += "],";
    // ---- stability: current (node 2s SDs) + 10/30/60 s windows ----
    float s10=0,d10=0,s30=0,d30=0,s60=0,d60=0;
    windStats.window(10,s10,d10); windStats.window(30,s30,d30); windStats.window(60,s60,d60);
    j += "\"stability\":{\"cur\":{\"spdSd\":"+String(avgSpdSd,2)+",\"dirSd\":"+String(avgDirSd,1)+"}";
    j += ",\"w10\":{\"spdSd\":"+String(s10,2)+",\"dirSd\":"+String(d10,1)+"}";
    j += ",\"w30\":{\"spdSd\":"+String(s30,2)+",\"dirSd\":"+String(d30,1)+"}";
    j += ",\"w60\":{\"spdSd\":"+String(s60,2)+",\"dirSd\":"+String(d60,1)+"}}";
    // ---- fleet sleep status for the phone UI ----
    {
        int acked = 0;
        for (uint8_t i = 0; i < n; i++) if (nodes[active[i]].sleeping) acked++;
        uint32_t nowMs = millis();
        long wr = (wakeUntil > nowMs) ? (long)((wakeUntil - nowMs) / 1000) : 0;
        j += ",\"sleep\":{\"mode\":" + String(fleetSleep ? "true" : "false");
        j += ",\"waking\":" + String(wr > 0 ? "true" : "false");
        j += ",\"wakeRemain\":" + String(wr);
        j += ",\"acked\":" + String(acked);
        // Full per-node commanded-sleep bitmap so the UI can render each node tab's
        // toggle correctly even for a slept node that has dropped out of active[]
        // (deep-sleeping nodes stop appearing in "nodes" once they time out).
        j += ",\"nodeMask\":" + String(nodeSleepMask) + "}";
    }
    {
        uint8_t st = aimStatus();
        j += ",\"aim\":{\"present\":" + String(aimbotPresent ? "true":"false");
        j += ",\"moving\":" + String((st & AIM_ST_MOVING) ? "true":"false");
        j += ",\"clamped\":" + String((st & AIM_ST_CLAMPED) ? "true":"false") + "}";
    }
    j += "}";
    return j;
}

// ---------------------------------------------------------------------------
// /solve result cache
// ---------------------------------------------------------------------------
// solve() runs two zero-search integrations plus the shot integration, all in
// double precision. The ESP32's FPU is SINGLE precision only, so every double
// op is a software library call -- expect ~100-400 ms per solve on-device, not
// the ~1 ms it measures on a host. That is most of a 450 ms TDMA slot, and the
// WebUI's Prediction tab used to request one every 350 ms poll, which starved
// the beacon/RX loop and dropped node packets.
//
// The client-side throttle (WebUI.h) is the primary fix. This cache is the
// second half: live temp/pressure/wind jitter constantly, so an exact-match
// cache would never hit. Quantizing the key to the precision that is
// physically meaningful -- 0.1 mph, 1 deg, 0.5 F, 0.01 inHg -- turns a stream
// of near-identical requests into cache hits without changing any answer the
// shooter can perceive. A cache hit costs microseconds.
static float qz(float v, float step) { return (float)(lroundf(v / step) * step); }

struct SolveKey {
    float mv, bc, wgt, cal, twist, blen, sh, zero, range;
    float tempF, pres, wind, windrel, lat, az;
    int   model, twistDir;
    bool  spin, jump, earth;
    bool  operator==(const SolveKey& o) const {
        return mv==o.mv && bc==o.bc && wgt==o.wgt && cal==o.cal && twist==o.twist &&
               blen==o.blen && sh==o.sh && zero==o.zero && range==o.range &&
               tempF==o.tempF && pres==o.pres && wind==o.wind && windrel==o.windrel &&
               lat==o.lat && az==o.az && model==o.model && twistDir==o.twistDir &&
               spin==o.spin && jump==o.jump && earth==o.earth;
    }
};
static SolveKey solveKeyCached;
static String   solveJsonCached;
static bool     solveCacheValid = false;

// Handle /solve?...: parse query params, run the standalone solver, return JSON.
String handleSolve(WebServer& s) {
    BallInput in;
    in.mv_fps      = s.arg("mv").toFloat();
    in.bc          = s.arg("bc").toFloat();
    in.dragModel   = s.arg("model").toInt();
    in.weight_gr   = s.arg("wgt").toFloat();
    in.cal_in      = s.arg("cal").toFloat();
    in.twist_in    = s.arg("twist").toFloat();
    in.blen_in     = s.arg("blen").toFloat();
    in.twistDir    = s.arg("twistDir").toInt();
    in.sightHt_in  = s.arg("sh").toFloat();
    in.zero_yd     = s.arg("zero").toFloat();
    in.range_yd    = s.arg("range").toFloat();
    in.tempF       = s.arg("tempF").toFloat();
    in.presInHg    = s.arg("pres").toFloat();
    in.wind_mph    = s.arg("windmph").toFloat();
    in.windRel_deg = s.arg("windrel").toFloat();
    in.lat_deg     = s.arg("lat").toFloat();
    in.azimuth_deg = s.arg("az").toFloat();
    in.useSpinDrift = (s.arg("spin") != "0");
    in.useAeroJump  = (s.arg("jump") != "0");
    in.useEarth     = (s.arg("earth") != "0");

    // ---- input sanity (bug #9) ----------------------------------------------
    // solve() checks mv/bc/range/cal/weight but NOT the atmosphere. An empty or
    // absent pressure arg gives presInHg = 0, and then:
    //   rho = P/(287.05*Tk) = 0  -> a drag-free vacuum trajectory, returned as
    //                               ok:true and looking entirely plausible
    //   sg *= 29.92/presInHg    -> inf, and String(inf) emits "inf" or "nan",
    //                               which is not valid JSON -> the phone's
    //                               JSON.parse throws and the UI shows
    //                               "Solve failed" with no clue why.
    // Clamping to physically sensible bounds turns both into an honest refusal.
    if (in.presInHg < 15.0f || in.presInHg > 35.0f) return "{\"ok\":false,\"err\":\"pressure\"}";
    if (in.tempF   < -80.0f || in.tempF   > 160.0f) return "{\"ok\":false,\"err\":\"temp\"}";

    // ---- cache lookup on quantized inputs ----
    SolveKey key;
    key.mv    = qz(in.mv_fps, 1.0f);     key.bc      = qz(in.bc, 0.001f);
    key.wgt   = qz(in.weight_gr, 0.1f);  key.cal     = qz(in.cal_in, 0.001f);
    key.twist = qz(in.twist_in, 0.1f);   key.blen    = qz(in.blen_in, 0.01f);
    key.sh    = qz(in.sightHt_in, 0.01f);key.zero    = qz(in.zero_yd, 1.0f);
    key.range = qz(in.range_yd, 1.0f);   key.tempF   = qz(in.tempF, 0.5f);
    key.pres  = qz(in.presInHg, 0.01f);  key.wind    = qz(in.wind_mph, 0.1f);
    key.windrel = qz(in.windRel_deg, 1.0f); key.lat  = qz(in.lat_deg, 0.5f);
    key.az    = qz(in.azimuth_deg, 1.0f);
    key.model = in.dragModel;            key.twistDir = in.twistDir;
    key.spin  = in.useSpinDrift; key.jump = in.useAeroJump; key.earth = in.useEarth;

    if (solveCacheValid && key == solveKeyCached) return solveJsonCached;

    // Solve using the QUANTIZED values, so the cached answer is exactly the
    // answer for its key -- otherwise the first request in a bucket would set
    // the value for every later request that rounds into the same bucket.
    in.mv_fps=key.mv; in.bc=key.bc; in.weight_gr=key.wgt; in.cal_in=key.cal;
    in.twist_in=key.twist; in.blen_in=key.blen; in.sightHt_in=key.sh;
    in.zero_yd=key.zero; in.range_yd=key.range; in.tempF=key.tempF;
    in.presInHg=key.pres; in.wind_mph=key.wind; in.windRel_deg=key.windrel;
    in.lat_deg=key.lat; in.azimuth_deg=key.az;

    BallOutput o = solver.solve(in);
    if (!o.ok) return "{\"ok\":false}";
    String j = "{\"ok\":true";
    j += ",\"elevMOA\":" + String(o.elevMOA,2) + ",\"elevMil\":" + String(o.elevMil,2);
    j += ",\"windMOA\":" + String(o.windMOA,2) + ",\"windMil\":" + String(o.windMil,2);
    j += ",\"spinDriftIn\":" + String(o.spinDriftIn,2) + ",\"aeroJumpMOA\":" + String(o.aeroJumpMOA,2);
    j += ",\"dropIn\":" + String(o.dropIn,1) + ",\"windDriftIn\":" + String(o.windDriftIn,1);
    j += ",\"tof\":" + String(o.tof,2) + ",\"vRemain\":" + String(o.vRemain_fps,0);
    j += ",\"sg\":" + String(o.sg,2) + "}";

    solveKeyCached  = key;
    solveJsonCached = j;
    solveCacheValid = true;
    return j;
}

// Handle /cmd?op=sleep|wake : fleet sleep control from the phone UI.
// Deliberately tiny -- flips state, persists it, returns the new state. The
// beacons in loop() do the actual commanding; there is nothing to "push" to
// the nodes from here.
String handleCmd(WebServer& s) {
    String op = s.arg("op");

    // Per-node targeting: /cmd?op=sleep&node=N or op=wake&node=N. Sets/clears one
    // bit in nodeSleepMask, layered on top of (and independent of) the fleet flag.
    // Absent node arg => the original fleet-wide behavior below, unchanged.
    if (s.hasArg("node")) {
        int node = s.arg("node").toInt();
        if (node < 1 || node > 15)
            return "{\"ok\":false,\"err\":\"node\"}";
        uint16_t bit = (uint16_t)(1u << node);
        if (op == "sleep") {
            nodeSleepMask |= bit;
            saveNodeSleep();
            Serial.printf("[sleep] node %d SLEEP commanded from UI\n", node);
        } else if (op == "wake") {
            nodeSleepMask &= (uint16_t)~bit;
            saveNodeSleep();
            Serial.printf("[sleep] node %d WAKE commanded from UI\n", node);
        } else {
            return "{\"ok\":false,\"err\":\"op\"}";
        }
        bool nodeSleeping = (nodeSleepMask & bit) != 0;
        return "{\"ok\":true,\"node\":" + String(node)
             + ",\"sleepCmd\":" + String(nodeSleeping ? "true" : "false")
             + ",\"mask\":" + String(nodeSleepMask) + "}";
    }

    if (op == "sleep") {
        fleetSleep = true;  wakeUntil = 0;
        saveFleetSleep();
        Serial.println(F("[sleep] fleet SLEEP commanded from UI"));
    } else if (op == "wake") {
        fleetSleep = false; wakeUntil = millis() + WAKE_BEACON_MS;
        saveFleetSleep();
        Serial.println(F("[sleep] fleet WAKE commanded from UI (30 s burst)"));
    } else {
        return "{\"ok\":false,\"err\":\"op\"}";
    }
    return "{\"ok\":true,\"mode\":\"" + String(fleetSleep ? "sleep" : "awake") + "\"}";
}

// Handle /aim?op=go|zero&moa=<float> : drive the windage stepper.
//   op=go   moa=<signed MOA>  -> SET_AND_GO to that absolute windage hold
//   op=zero                   -> return turret to origin, clear hold
//   op=origin                 -> adopt current turret position as new origin
// Returns the controller status so the UI can show moving / clamped / etc.
// The stepper moves ONLY when this is called, i.e. only on a button press.
String handleAim(WebServer& s) {
    if (!aimbotPresent) return "{\"ok\":false,\"err\":\"no_stepper\"}";
    String op = s.arg("op");
    bool sent = false;
    if (op == "go") {
        float moa = s.arg("moa").toFloat();
        if (moa >  32.0f) moa =  32.0f;
        if (moa < -32.0f) moa = -32.0f;
        int16_t milli = (int16_t)lroundf(moa * 1000.0f);
        sent = aimSend(AIM_CMD_SET_AND_GO, milli);
        Serial.printf("[aimbot] GO %.2f MOA -> %d milli-MOA\n", moa, milli);
    } else if (op == "zero") {
        sent = aimSend(AIM_CMD_ZERO, 0);
        Serial.println(F("[aimbot] ZERO (return to origin)"));
    } else if (op == "origin") {
        sent = aimSend(AIM_CMD_SET_ORIGIN, 0);
        Serial.println(F("[aimbot] SET ORIGIN (current = new zero)"));
    } else {
        return "{\"ok\":false,\"err\":\"op\"}";
    }
    if (!sent) return "{\"ok\":false,\"err\":\"i2c\"}";
    delay(2);
    uint8_t st = aimStatus();
    String j = "{\"ok\":true,\"status\":" + String(st);
    j += ",\"moving\":" + String((st & AIM_ST_MOVING) ? "true":"false");
    j += ",\"clamped\":" + String((st & AIM_ST_CLAMPED) ? "true":"false") + "}";
    return j;
}

// Handle /sensitivity?...: compute the wind-sensitivity weighting for every
// active node. Position source is GPS-by-default (base GPS + firing azimuth +
// node GPS, all already on the box) with a per-node manual downrange override
// (query arg drN=<yards>). Returns each node's downrange distance, the source
// used, and its normalized weight percentage (summing to 100 over positioned
// nodes). The heavy curve is cached inside Sensitivity and only recomputes when
// a ballistic input actually changes -- wind changes are free.
String handleSensitivity(WebServer& s) {
    BallInput in;
    in.mv_fps     = s.arg("mv").toFloat();
    in.bc         = s.arg("bc").toFloat();
    in.dragModel  = s.arg("model").toInt();
    in.weight_gr  = s.arg("wgt").toFloat();
    in.cal_in     = s.arg("cal").toFloat();
    in.twist_in   = s.arg("twist").toFloat();
    in.blen_in    = s.arg("blen").toFloat();
    in.sightHt_in = s.arg("sh").toFloat();
    in.zero_yd    = s.arg("zero").toFloat();
    in.range_yd   = s.arg("range").toFloat();
    in.tempF      = s.arg("tempF").toFloat();
    in.presInHg   = s.arg("pres").toFloat();
    // not used by the sensitivity curve, but keep the struct fully initialized
    in.twistDir = 1; in.wind_mph = 0; in.windRel_deg = 0; in.lat_deg = 0;
    in.azimuth_deg = 0; in.useSpinDrift = false; in.useAeroJump = false; in.useEarth = false;

    float az = s.arg("az").toFloat();          // firing azimuth (deg from north)

    if (in.presInHg < 15.0f || in.presInHg > 35.0f) return "{\"ok\":false,\"err\":\"pressure\"}";
    if (in.tempF   < -80.0f || in.tempF   > 160.0f) return "{\"ok\":false,\"err\":\"temp\"}";

    // ---- non-blocking curve compute (P0 fix) --------------------------------
    // The heavy sweep used to run right here, freezing loop() for 2-4 s. It now
    // runs on a core-0 worker (see sensWorker above). This handler just checks
    // the job state and answers immediately:
    //   * curve for THESE inputs is ready  -> fall through and serve it
    //   * a job is running / needs starting -> return {"computing":true} now;
    //     the phone keeps its approximate curve and polls again shortly.
    // Because loop() no longer stalls, beacons keep flowing and the fleet never
    // drops sync waiting on a curve. We still emit one beacon when KICKING a job
    // (cheap insurance that the nodes' timeout window is fresh at the start).
    String reqKey = sensKeyOf(in);
    bool   serveNow  = false;   // curve ready -> serve it below
    bool   failed    = false;   // job for these inputs finished but did NOT converge

    if (sensLock) xSemaphoreTake(sensLock, portMAX_DELAY);
    SensJobState st = sensState;
    bool keyMatch = (sensDoneKey == reqKey);
    if (st == SENS_DONE && keyMatch) {
        // A job for exactly these inputs has finished. Either it produced a curve
        // (serve it) or it genuinely couldn't converge (report failure). Do NOT
        // re-kick -- re-running the same failing inputs would loop forever.
        if (sensDoneOk && sens.ready()) serveNow = true;
        else                            failed   = true;
    } else if (st == SENS_RUNNING) {
        /* a job is in flight (for these or stale inputs); tell the client to poll */
    } else {
        // IDLE, or DONE for DIFFERENT inputs -> start a fresh job for the current ones.
        sensPendingIn  = in;                   // `in` is fully initialized by here
        sensPendingKey = reqKey;
        sensState      = SENS_RUNNING;
        {
            BeaconSched sc = computeSched();
            uint8_t bf = fleetSleep ? BFLAG_SLEEP_CMD : (millis() < wakeUntil ? BFLAG_WAKE_CMD : 0);
            radio.sendBeacon(sc.count, SLOT_MS, sc.cycleMs, sc.mask, bf, nodeSleepMask);
            lastBeacon = millis();
        }
        BaseType_t ok = xTaskCreatePinnedToCore(
            sensWorker, "sensWorker", 8192, nullptr, 1, nullptr, 0 /* core 0 */);
        if (ok != pdPASS) {
            // Spawn failed (OOM) -- fall back to the old inline behaviour so we
            // still return a result rather than spinning "computing" forever.
            // This runs on the loop task (core 1), so guard ITS watchdog for the
            // span of the sweep, same reasoning as the core-0 worker above.
            Serial.println(F("[sens] WARN: task spawn failed, running inline"));
            sensState = SENS_IDLE;
            if (sensLock) xSemaphoreGive(sensLock);
            disableLoopWDT();
            sens.update(in);
            enableLoopWDT();
            if (sensLock) xSemaphoreTake(sensLock, portMAX_DELAY);
            sensDoneKey = reqKey;
            sensDoneOk  = sens.ready();
            sensState   = SENS_DONE;
            serveNow    = sensDoneOk;
            failed      = !sensDoneOk;
        }
    }
    if (sensLock) xSemaphoreGive(sensLock);

    if (failed)   return "{\"ok\":false}";                     // converged-no / bad inputs
    if (!serveNow) return "{\"ok\":false,\"computing\":true}"; // still working -> poll
    if (!sens.ready()) return "{\"ok\":false}";

    uint8_t active[MAX_NODES];
    uint8_t n = buildActive(active);
    bool baseFix = (rxLat != 0 && rxLon != 0);

    float       raw[MAX_NODES];
    float       down[MAX_NODES];
    const char* src[MAX_NODES];
    float       sum = 0.0f;

    for (uint8_t i = 0; i < n; i++) {
        uint8_t id = active[i];
        NodeView& nv = nodes[id];
        String man = s.arg(String("dr") + String(id));   // manual override (yards)
        if (man.length() > 0) {
            down[i] = man.toFloat();
            src[i]  = "manual";
        } else if (baseFix && nv.gpsValid && nv.lat != 0 && nv.lon != 0) {
            down[i] = Sensitivity::projectDownrangeYd(rxLat, rxLon, az, nv.lat, nv.lon);
            src[i]  = "gps";
        } else {
            down[i] = -1.0f;
            src[i]  = "none";
        }
        raw[i] = (src[i][0] == 'n') ? 0.0f : sens.sensitivityAt(down[i]);
        sum   += raw[i];
    }

    String j = "{\"ok\":true,\"rangeYd\":" + String(sens.rangeYd(), 0)
             + ",\"seg\":" + String(sens.segments())
             + ",\"segYd\":" + String(sens.segWidthYd(), 1);
    // Raw per-segment crosswind sensitivity (inches/mph), muzzle->target. The
    // Sandbox uses this as the TRUE wind-importance curve instead of its offline
    // power-law approximation. Small array (<=64 floats); only fetched when a
    // ballistic input changes, so the extra bytes are cheap.
    j += ",\"coef\":[";
    for (int i = 0; i < sens.segments(); i++) {
        // coefAt(i) was not exposed on the Sensitivity class; sample the curve
        // at the segment's own centre instead -- sensitivityAt() interpolates
        // from the same coefficient array, and at a segment midpoint it returns
        // exactly the stored coefficient for that segment.
        float midYd = (i + 0.5f) * sens.segWidthYd();
        j += String(sens.sensitivityAt(midYd), 4);
        if (i < sens.segments() - 1) j += ",";
    }
    j += "],\"nodes\":[";
    for (uint8_t i = 0; i < n; i++) {
        float pct = (sum > 0.0f) ? (raw[i] / sum * 100.0f) : 0.0f;
        j += "{\"id\":" + String(active[i])
           + ",\"downYd\":" + String(down[i], 0)
           + ",\"src\":\"" + src[i] + "\""
           + ",\"wPct\":" + String(pct, 1) + "}";
        if (i < n - 1) j += ",";
    }
    j += "]}";
    return j;
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.println(F("\n=== Wind Receiver booting ==="));

    Wire.begin(I2C_SDA, I2C_SCL);

    pinMode(BUTTON_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), onButton, FALLING);

    screen = ScreenFactory::create();
    if (screen) screen->begin();

    if (pmu.init()) Serial.println(F("[Power] PMU ready"));
    else            Serial.println(F("[Power] PMU not found"));

    baroOk = baro.init();   // base-station BMP180 (prints its own status)

    gps.begin();
    Serial.println(F("[GPS] started"));

    if (!radio.begin()) Serial.println(F("[LoRa] receiver FAILED to start"));

    // Restore fleet-sleep across receiver reboots. Without this, a base that
    // browned out mid-sleep would come back "awake" and silently wake the
    // whole fleet at their next check-ins.
    rxPrefs.begin("rxcfg", true);
    fleetSleep    = rxPrefs.getBool("fsleep", false);
    nodeSleepMask = rxPrefs.getUShort("nsleep", 0);
    rxPrefs.end();
    if (fleetSleep) Serial.println(F("[sleep] restored: fleet is in SLEEP mode"));
    if (nodeSleepMask) Serial.printf("[sleep] restored: per-node sleep mask 0x%04X\n", nodeSleepMask);

    sensLock = xSemaphoreCreateMutex();   // guards the async sensitivity worker
    if (!sensLock) Serial.println(F("[sens] WARN: mutex alloc failed (will run inline)"));

    aimProbe();   // detect the windage stepper controller (0x33)

    webui.begin(AP_SSID, AP_PASS, buildDataJson, handleSolve, handleSensitivity, handleCmd, handleAim);   // + fleet sleep + aimbot stepper

    lastRxMs = millis();   // start the RX watchdog window
    Serial.println(F("=== listening ===\n"));
}

// ============================================================================
// LOOP
// ============================================================================
void loop() {
    // 1) drain radio
    NodePacket pkt; float rssi, snr;
    while (radio.poll(pkt, rssi, snr)) ingestPacket(pkt, rssi, snr);

    webui.handle();        // service phone dashboard requests

    // feed the rolling stability buffer with the unweighted aggregate wind
    {
        uint8_t act[MAX_NODES]; uint8_t na = buildActiveWind(act);   // wind-valid only
        float ss = 0, sx = 0, sy = 0;
        for (uint8_t i = 0; i < na; i++) {
            NodeView& nv = nodes[act[i]];
            ss += nv.windSpeed;
            float r = nv.windDir * (float)DEG_TO_RAD; sx += cosf(r); sy += sinf(r);
        }
        float aggSpd = na ? ss/na : 0;
        float aggDir = 0;
        if (na) { aggDir = atan2f(sy, sx) * RAD_TO_DEG; if (aggDir < 0) aggDir += 360; }
        windStats.update(aggSpd, aggDir, na > 0);
    }

    // 1a) BEACON coordinator: broadcast the compact schedule at each cycle start.
    //     slot 0 = beacon, slots 1..count = active nodes by rank, then one empty
    //     guard slot so the last node never abuts the next beacon (computeSched).
    {
        BeaconSched sc = computeSched();
        if (millis() - lastBeacon >= sc.cycleMs) {
            lastBeacon = millis();
            uint8_t bf = 0;
            if (fleetSleep)                 bf |= BFLAG_SLEEP_CMD;
            else if (millis() < wakeUntil)  bf |= BFLAG_WAKE_CMD;
            radio.sendBeacon(sc.count, SLOT_MS, sc.cycleMs, sc.mask, bf, nodeSleepMask);
        }
    }

    // 1b) RX watchdog -- recover if the SX127x quietly stops listening.
    //     (This is what caused "receiver stopped until I reset a node".)
    if (millis() - lastRxMs > RX_WATCHDOG_MS) {
        static uint32_t lastKick = 0;
        if (millis() - lastKick > RX_WATCHDOG_MS) {
            lastKick = millis();
            if (millis() - lastRxMs > RX_RESTART_MS) {
                Serial.println(F("[LoRa] watchdog: full radio restart"));
                radio.restart();
                lastRxMs = millis();          // give it a fresh window
            } else {
                Serial.println(F("[LoRa] watchdog: re-arming RX"));
                radio.rearm();
            }
        }
    }

    // 2) button -> next view
    if (buttonFlag) {
        static uint32_t lastBtn = 0;
        uint32_t now = millis();
        if (now - lastBtn > BUTTON_DEBOUNCE_MS) {
            uint8_t active[MAX_NODES];
            uint8_t n = buildActive(active);
            viewCursor = (viewCursor + 1) % (n + 1);   // wrap through general + nodes
            lastBtn = now;
            drawCurrentView();
            Serial.printf("[UI] view -> %s\n",
                          viewCursor == 0 ? "GENERAL" : "node detail");
        }
        buttonFlag = false;
    }

    // 3) receiver's own GPS (throttled -- GPS_MS was defined but never used;
    //    the fixed getCoordinates() drains the UART fully per call, so 1 Hz
    //    is plenty and keeps the loop tight)
    static uint32_t lastGps = 0;
    if (millis() - lastGps >= GPS_MS) {
        lastGps = millis();
        gps.getCoordinates(rxLat, rxLon);
    }

    // 4) receiver battery (throttled)
    static uint32_t lastBatt = 0;
    if (millis() - lastBatt >= BATTERY_MS) {
        lastBatt = millis();
        float pct = pmu.getBatteryPercentage();
        if (pct >= 0 && pct <= 100) rxBatteryPct = (int)pct;
        float v = pmu.getBatteryVoltage();
        if (v > 0) rxBatteryV = v;
    }

    // 4b) base-station atmosphere (throttled)
    static uint32_t lastBaro = 0;
    if (baroOk && millis() - lastBaro >= 1000) {
        lastBaro = millis();
        float t = baro.readTemperatureF();
        float p = baro.readPressureInHg();
        if (t > -50 && t < 150) baseTempF = t;
        if (p > 20  && p < 35)  basePresInHg = p;
    }

    // 5) screen refresh (throttled)
    static uint32_t lastDraw = 0;
    if (millis() - lastDraw >= SCREEN_REFRESH_MS) {
        lastDraw = millis();
        if (screen) drawCurrentView();
    }

    // 6) serial status summary (throttled)
    static uint32_t lastStat = 0;
    if (millis() - lastStat >= STATUS_PRINT_MS) {
        lastStat = millis();
        printStatusSummary();
    }
}
