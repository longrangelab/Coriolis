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
// v3 adds FLAG_WIND_VALID. Wire format, struct sizes and CRC are UNCHANGED, so
// a v2 node still decodes perfectly on a v3 receiver -- the receiver just treats
// v2 packets as wind-valid (the old behaviour). Mixed networks are safe.
//
// v4 (compact TDMA slots): the BEACON gains a node_mask and slots are assigned by
// a node's RANK among active ids, not its raw id. The NodePacket (data) wire
// format is UNCHANGED, so data still decodes across v3/v4. The BeaconPacket grew,
// though, so a v3 node and a v4 receiver (or vice-versa) fail each other's beacon
// CRC and simply don't sync -- the node falls back to autonomous jittered TX,
// which is the safe degradation. Reflash the whole network to get TDMA back.
//
// v5 (per-node sleep): the BEACON gains a sleep_mask (bit i => node i must sleep),
// letting the base sleep/wake individual nodes on top of the fleet-wide command.
// The NodePacket (data) wire format is still UNCHANGED. The BeaconPacket grew by
// 2 bytes again, so a v4 node and v5 receiver fail each other's beacon CRC and
// don't sync -- node falls to autonomous jittered TX (stays awake) = safe. Reflash
// the whole network. Fleet BFLAG_SLEEP_CMD/WAKE_CMD behavior is unchanged.
#define PACKET_VERSION       5

#define FLAG_BARO_VALID   0x01
#define FLAG_LIGHT_VALID  0x02
#define FLAG_GPS_VALID    0x04
// FLAG_SLEEP_ACK: set on our NodePacket right BEFORE we enter deep sleep, so
// the base can count how many nodes acknowledged the sleep command it sent.
// (See design note in the sleep section of node_main.cpp.)
#define FLAG_SLEEP_ACK    0x08
// FLAG_WIND_VALID: the wind figures in this packet came from a live ATtiny read
// (or a recent one held over). CLEAR means the anemometer is absent, dead, or
// stale -- in which case wind_speed/dir are 0 and the receiver MUST NOT fold
// them into any average. Without this bit a failed meter silently reported
// 0.00 mph and dragged the weighted average toward calm, which is the single
// worst failure mode this system has: a confident, wrong, low wind call.
#define FLAG_WIND_VALID   0x10

// Bits inside BeaconPacket.flags. The flags byte was previously unused, so
// old firmware just ignores whatever we set here -- a node running pre-sleep
// firmware would stay awake, which is the SAFE failure mode.
#define BFLAG_SLEEP_CMD   0x01   // "after this cycle, enter deep sleep"
#define BFLAG_WAKE_CMD    0x02   // "wake broadcast -- stay awake and resync"

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
    uint8_t  max_slot;       // v4: NUMBER of node slots this cycle (= popcount(node_mask))
    uint8_t  flags;
    uint16_t slot_ms;        // per-node slot width
    uint16_t cycle_ms;       // beacon-to-beacon period
    uint32_t seq;            // sequence (debug)
    uint16_t node_mask;      // v4: bit i (1..15) set => node id i owns a slot this cycle
    uint16_t sleep_mask;     // v5: bit i set => node id i should deep-sleep (per-node)
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
    bool    windValid = false;
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

static inline size_t packReading(const NodeReading& r, uint8_t node_id, uint8_t* out,
                                 bool ackSleep = false) {
    NodePacket p;
    p.magic = NODE_PACKET_MAGIC; p.version = PACKET_VERSION; p.node_id = node_id;
    p.flags = (r.baroValid?FLAG_BARO_VALID:0)|(r.lightValid?FLAG_LIGHT_VALID:0)|(r.gpsValid?FLAG_GPS_VALID:0)
            | (ackSleep?FLAG_SLEEP_ACK:0) | (r.windValid?FLAG_WIND_VALID:0);
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

// ---- compact TDMA slot map (v4) -------------------------------------------
// The beacon's node_mask has bit i (1..15) set when node id i owns a slot this
// cycle. A node's slot is its RANK among the active ids, NOT its raw id, so ids
// {1,3,10} occupy slots {1,2,3} -- three slots, not ten. slot 0 is the beacon.
// The receiver and every node run this SAME code on the SAME mask, so both sides
// always agree on who sits where within a cycle. (bit 0 is unused/reserved.)
static inline uint8_t beaconSlotCount(uint16_t mask) {
    return (uint8_t)__builtin_popcount((unsigned)(mask & 0xFFFEu));
}
// 1-based slot index for nodeId, or 0 if nodeId isn't scheduled in this mask.
static inline uint8_t beaconSlotOf(uint16_t mask, uint8_t nodeId) {
    if (nodeId == 0 || nodeId > 15) return 0;
    if (!((mask >> nodeId) & 1u)) return 0;                 // not scheduled yet
    uint16_t below = (uint16_t)(mask & ((1u << nodeId) - 1u) & 0xFFFEu);
    return (uint8_t)(__builtin_popcount((unsigned)below) + 1);
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

    // Put the SX1276 into sleep mode (~1 uA) before ESP32 deep sleep. Without
    // this the radio idles in RX at ~12 mA and dominates the sleep current --
    // the deep-sleep feature would save almost nothing.
    void sleepRadio() { if (initialized) radio.sleep(); }

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
    // ackSleep=true sets FLAG_SLEEP_ACK so the base can count sleep confirmations.
    bool sendData(const NodeReading& r, uint8_t nodeId, bool ackSleep = false) {
        if (!initialized) return false;
        uint8_t buf[64];
        size_t len = packReading(r, nodeId, buf, ackSleep);
        int st = radio.transmit(buf, len);
        radio.startReceive();
        if (st == RADIOLIB_ERR_NONE) { Serial.printf("[LoRa] TX node %d (%u B)%s\n", nodeId, (unsigned)len, ackSleep?" [SLEEP-ACK]":""); return true; }
        Serial.printf("[LoRa] TX fail %d\n", st);
        return false;
    }
};

#endif // TELEMETRY_H
