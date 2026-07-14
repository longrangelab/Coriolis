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

#include "Screen.h"
#include "Power.h"
#include "Environment.h"          // BMP085Sensor (baro) + GPSModule (GPS)
#include "Telemetry.h"
#include "Ballistics.h"            // standalone ballistic solver
#include "WindStats.h"             // rolling wind history + stability
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
#define SCREEN_REFRESH_MS  500UL
#define STATUS_PRINT_MS    5000UL
#define BATTERY_MS         2000UL
#define GPS_MS             1000UL
#define BUTTON_DEBOUNCE_MS 200UL

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
    float    batteryV = 0;
    double   lat = 0, lon = 0;
    bool     gpsValid = false;
    float    rssi = 0, snr = 0;
    uint32_t lastSeen = 0;
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

// ============================================================================
// HELPERS
// ============================================================================
uint8_t buildActive(uint8_t* out) {
    uint8_t n = 0;
    uint32_t now = millis();
    for (uint8_t id = 1; id < MAX_NODES; id++)
        if (nodes[id].seen && (now - nodes[id].lastSeen) < NODE_TIMEOUT_MS)
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
    return (d > 0) ? (long)d : -1;
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
    nv.batteryV    = (p.battery_mv > 0 && p.battery_mv < 5000) ? p.battery_mv / 1000.0f : 0;
    nv.gpsValid    = (p.flags & FLAG_GPS_VALID);
    nv.lat         = p.latitude  / 1e7;
    nv.lon         = p.longitude / 1e7;
    nv.rssi        = rssi;
    nv.snr         = snr;
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
        // GENERAL: averages across active nodes
        float sumSpeed = 0, sx = 0, sy = 0;
        float sumTempF = 0, sumPresHg = 0; int nEnv = 0;
        for (uint8_t i = 0; i < n; i++) {
            NodeView& nv = nodes[active[i]];
            sumSpeed += nv.windSpeed;
            float r = nv.windDir * (float)DEG_TO_RAD;
            sx += cosf(r); sy += sinf(r);
            if (nv.pressurePa > 0) {                 // only nodes with a baro
                sumTempF  += nv.temperatureC * 9.0f / 5.0f + 32.0f;
                sumPresHg += nv.pressurePa / 3386.389f;
                nEnv++;
            }
        }
        float avgSpeed = (n > 0) ? sumSpeed / n : 0;
        int   avgDir   = 0;
        if (n > 0) { avgDir = (int)(atan2f(sy, sx) * RAD_TO_DEG + 0.5f); if (avgDir < 0) avgDir += 360; }
        float avgTempF = (nEnv > 0) ? sumTempF  / nEnv : 0;
        float avgPresHg= (nEnv > 0) ? sumPresHg / nEnv : 0;
        screen->drawGeneral(n, avgSpeed, avgDir, avgTempF, avgPresHg,
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

// Highest active node id sets the cycle length; clamp to [MIN_SLOTS, MAX_NODES-1].
uint8_t computeMaxSlot() {
    uint8_t hi = 0;
    uint32_t now = millis();
    for (uint8_t id = 1; id < MAX_NODES; id++)
        if (nodes[id].seen && (now - nodes[id].lastSeen) < NODE_TIMEOUT_MS) hi = id;
    if (hi < MIN_SLOTS)        hi = MIN_SLOTS;
    if (hi > MAX_NODES - 1)    hi = MAX_NODES - 1;
    return hi;
}

// Build the JSON the phone dashboard polls (/data.json). Imperial units.
String buildDataJson() {
    uint8_t active[MAX_NODES];
    uint8_t n = buildActive(active);

    float sumSpd = 0, sx = 0, sy = 0, sumSpdSd = 0, sumDirSd = 0, sumT = 0, sumP = 0;
    int   nEnv = 0;
    for (uint8_t i = 0; i < n; i++) {
        NodeView& nv = nodes[active[i]];
        sumSpd += nv.windSpeed; sumSpdSd += nv.windSpeedSd; sumDirSd += nv.windDirSd;
        float r = nv.windDir * (float)DEG_TO_RAD; sx += cosf(r); sy += sinf(r);
        if (nv.pressurePa > 0) { sumT += nv.temperatureC*9.0f/5.0f+32.0f; sumP += nv.pressurePa/3386.389f; nEnv++; }
    }
    float avgSpeed = n ? sumSpd/n : 0;
    int   avgDir = 0;
    if (n) { avgDir = (int)(atan2f(sy, sx) * RAD_TO_DEG + 0.5f); if (avgDir < 0) avgDir += 360; }
    float avgSpdSd = n ? sumSpdSd/n : 0, avgDirSd = n ? sumDirSd/n : 0;
    float avgT = nEnv ? sumT/nEnv : 0, avgP = nEnv ? sumP/nEnv : 0;
    bool baseFix = (rxLat != 0 && rxLon != 0);

    String j = "{\"base\":{";
    j += "\"lat\":" + String(rxLat,6) + ",\"lon\":" + String(rxLon,6);
    j += ",\"tempF\":" + String(baseTempF,1) + ",\"presInHg\":" + String(basePresInHg,2);
    j += ",\"battPct\":" + String(rxBatteryPct) + ",\"battV\":" + String(rxBatteryV,2);
    j += ",\"gps\":" + String(baseFix ? "true":"false") + "},";
    j += "\"avg\":{\"count\":" + String(n) + ",\"speed\":" + String(avgSpeed,2);
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
        j += ",\"age\":" + String((int)((millis()-nv.lastSeen)/1000)) + "}";
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
    j += "}";
    return j;
}

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

    BallOutput o = solver.solve(in);
    if (!o.ok) return "{\"ok\":false}";
    String j = "{\"ok\":true";
    j += ",\"elevMOA\":" + String(o.elevMOA,2) + ",\"elevMil\":" + String(o.elevMil,2);
    j += ",\"windMOA\":" + String(o.windMOA,2) + ",\"windMil\":" + String(o.windMil,2);
    j += ",\"spinDriftIn\":" + String(o.spinDriftIn,2) + ",\"aeroJumpMOA\":" + String(o.aeroJumpMOA,2);
    j += ",\"dropIn\":" + String(o.dropIn,1) + ",\"windDriftIn\":" + String(o.windDriftIn,1);
    j += ",\"tof\":" + String(o.tof,2) + ",\"vRemain\":" + String(o.vRemain_fps,0);
    j += ",\"sg\":" + String(o.sg,2) + "}";
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

    sens.update(in);                           // cached; recomputes only if changed
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
             + ",\"segYd\":" + String(sens.segWidthYd(), 1) + ",\"nodes\":[";
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

    webui.begin(AP_SSID, AP_PASS, buildDataJson, handleSolve, handleSensitivity);   // phone dashboard + solver + Phase 2 weighting

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
        uint8_t act[MAX_NODES]; uint8_t na = buildActive(act);
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

    // 1a) BEACON coordinator: broadcast the schedule at each cycle start.
    //     cycle = (maxSlot + 1) slots; slot 0 = beacon, slots 1..maxSlot = nodes.
    static uint32_t lastBeacon = 0;
    {
        uint8_t  maxSlot = computeMaxSlot();
        uint16_t cycleMs = (uint16_t)((maxSlot + 1) * SLOT_MS);
        if (millis() - lastBeacon >= cycleMs) {
            lastBeacon = millis();
            radio.sendBeacon(maxSlot, SLOT_MS, cycleMs);
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

    // 3) receiver's own GPS
    gps.getCoordinates(rxLat, rxLon);

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
