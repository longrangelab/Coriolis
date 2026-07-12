#ifndef TELEMETRY_H
#define TELEMETRY_H

/**
 * Node radio + packets  (TTGO T-Beam, SX1276)
 * ===========================================
 * The node now listens for the base station's BEACON (TDMA sync) and transmits
 * its NodePacket only in its assigned slot. Falls back to autonomous jittered
 * transmit if no beacon is heard (base off / out of range).
 *
 * >>> NodePacket + BeaconPacket + the radio profile MUST stay identical to the
 *     receiver's Telemetry.h. <<<
 */

#include <Arduino.h>
#include <RadioLib.h>

// ---- board pins (T-Beam v1.1/v1.2, SX1276) ----
#define RADIO_NSS   18
#define RADIO_IRQ   26
#define RADIO_RST   23
#define RADIO_GPIO  33

// ---- radio profile (SF9 = good margin at ~1 mile) ----
#define LORA_FREQ_MHZ   915.0
#define LORA_BW_KHZ     125.0
#define LORA_SF         9
#define LORA_CR         7
#define LORA_TX_DBM     17
#define LORA_PREAMBLE   8

// ---- TDMA timing (must match receiver) ----
#define SLOT_MS         450          // per-node slot (SF9 data airtime ~312ms + guard)
#define MIN_SLOTS       4            // smallest cycle capacity (bootstrapping)

// ---- packet identity ----
#define NODE_PACKET_MAGIC    0x57    // 'W' data packet
#define BEACON_PACKET_MAGIC  0x42    // 'B' beacon
#define PACKET_VERSION       2

#define FLAG_BARO_VALID   0x01
#define FLAG_LIGHT_VALID  0x02
#define FLAG_GPS_VALID    0x04

// ---------------------------------------------------------------------------
#pragma pack(push, 1)
struct NodePacket {
    uint8_t  magic;          // NODE_PACKET_MAGIC
    uint8_t  version;
    uint8_t  node_id;
    uint8_t  flags;
    uint16_t wind_speed;     // mph  * 100
    uint16_t wind_speed_sd;  // mph  * 100
    uint16_t wind_dir;       // deg  * 10
    uint16_t wind_dir_sd;    // deg  * 10
    uint16_t wind_gust;      // mph  * 100
    int16_t  temperature;    // degC * 100  (converted to F on the receiver)
    uint32_t pressure;       // Pa
    uint32_t light;          // lux * 100
    uint16_t battery_mv;     // millivolts
    int32_t  latitude;       // deg * 1e7
    int32_t  longitude;      // deg * 1e7
    uint16_t crc;
};

struct BeaconPacket {
    uint8_t  magic;          // BEACON_PACKET_MAGIC
    uint8_t  version;
    uint8_t  max_slot;       // cycle spans slots 1..max_slot
    uint8_t  flags;
    uint16_t slot_ms;        // per-node slot width
    uint16_t cycle_ms;       // beacon-to-beacon period
    uint32_t seq;            // sequence (debug)
    uint16_t crc;
};
#pragma pack(pop)

struct NodeReading {
    float   windSpeed = 0,  windSpeedSd = 0;
    float   windDir   = 0,  windDirSd   = 0;
    float   windGust  = 0;
    float   temperatureC = 0;
    float   pressurePa   = 0;
    float   lightLux     = 0;
    float   batteryVolts = 0;
    double  latitude = 0, longitude = 0;
    bool    baroValid = false, lightValid = false, gpsValid = false;
};

// ---- CRC-16/CCITT-FALSE ----
static inline uint16_t nodeCrc16(const uint8_t* d, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)d[i] << 8;
        for (uint8_t b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}

static inline size_t packReading(const NodeReading& r, uint8_t node_id, uint8_t* out) {
    NodePacket p;
    p.magic = NODE_PACKET_MAGIC; p.version = PACKET_VERSION; p.node_id = node_id;
    p.flags = (r.baroValid?FLAG_BARO_VALID:0)|(r.lightValid?FLAG_LIGHT_VALID:0)|(r.gpsValid?FLAG_GPS_VALID:0);
    p.wind_speed    = (uint16_t)(r.windSpeed   * 100.0f + 0.5f);
    p.wind_speed_sd = (uint16_t)(r.windSpeedSd * 100.0f + 0.5f);
    p.wind_dir      = (uint16_t)(r.windDir     * 10.0f  + 0.5f);
    p.wind_dir_sd   = (uint16_t)(r.windDirSd   * 10.0f  + 0.5f);
    p.wind_gust     = (uint16_t)(r.windGust    * 100.0f + 0.5f);
    p.temperature   = (int16_t)lroundf(r.temperatureC * 100.0f);
    p.pressure      = (uint32_t)lroundf(r.pressurePa);
    p.light         = (uint32_t)lroundf(r.lightLux * 100.0f);
    p.battery_mv    = (uint16_t)lroundf(r.batteryVolts * 1000.0f);
    p.latitude      = (int32_t)llround(r.latitude  * 1e7);
    p.longitude     = (int32_t)llround(r.longitude * 1e7);
    p.crc = nodeCrc16((uint8_t*)&p, sizeof(NodePacket) - sizeof(uint16_t));
    memcpy(out, &p, sizeof(NodePacket));
    return sizeof(NodePacket);
}

static inline bool unpackBeacon(const uint8_t* buf, size_t len, BeaconPacket& b) {
    if (len < sizeof(BeaconPacket)) return false;
    memcpy(&b, buf, sizeof(BeaconPacket));
    if (b.magic != BEACON_PACKET_MAGIC) return false;
    return nodeCrc16(buf, sizeof(BeaconPacket) - sizeof(uint16_t)) == b.crc;
}

// ===========================================================================
// NODE RADIO  (RX for beacons + TX for data)
// ===========================================================================
static volatile bool nodeRxFlag = false;
static void IRAM_ATTR nodeOnRxISR() { nodeRxFlag = true; }

class NodeRadio {
private:
    SX1276 radio = new Module(RADIO_NSS, RADIO_IRQ, RADIO_RST, RADIO_GPIO);
    bool initialized = false;
public:
    bool begin() {
        Serial.print(F("[LoRa] node init ... "));
        int st = radio.begin(LORA_FREQ_MHZ, LORA_BW_KHZ, LORA_SF, LORA_CR,
                             RADIOLIB_SX127X_SYNC_WORD, LORA_TX_DBM, LORA_PREAMBLE, 0);
        if (st != RADIOLIB_ERR_NONE) { Serial.printf("FAILED %d\n", st); return false; }
        radio.setPacketReceivedAction(nodeOnRxISR);
        radio.startReceive();
        initialized = true;
        Serial.printf("OK (SF%d)\n", LORA_SF);
        return true;
    }
    bool isReady() const { return initialized; }
    void listen() { if (initialized) radio.startReceive(); }

    // If a beacon arrived, decode it. Ignores data packets from other nodes.
    bool pollBeacon(BeaconPacket& b) {
        if (!initialized || !nodeRxFlag) return false;
        nodeRxFlag = false;
        uint8_t buf[64];
        size_t len = radio.getPacketLength();
        bool ok = false;
        if (len && len <= sizeof(buf) && radio.readData(buf, len) == RADIOLIB_ERR_NONE)
            ok = unpackBeacon(buf, len, b);
        radio.startReceive();
        return ok;
    }

    // Transmit our data, then return to listening for the next beacon.
    bool sendData(const NodeReading& r, uint8_t nodeId) {
        if (!initialized) return false;
        uint8_t buf[64];
        size_t len = packReading(r, nodeId, buf);
        int st = radio.transmit(buf, len);
        radio.startReceive();
        if (st == RADIOLIB_ERR_NONE) { Serial.printf("[LoRa] TX node %d (%u B)\n", nodeId, (unsigned)len); return true; }
        Serial.printf("[LoRa] TX fail %d\n", st);
        return false;
    }
};

#endif // TELEMETRY_H
