#ifndef TELEMETRY_H
#define TELEMETRY_H

/**
 * Receiver radio + packets  (TTGO T-Beam, SX1276)  --  TDMA COORDINATOR
 * ====================================================================
 * Broadcasts a BEACON at each cycle start and receives NodePackets in between.
 *
 * >>> NodePacket + BeaconPacket + the radio profile MUST stay identical to the
 *     node's telemetry.h. <<<
 */

#include <Arduino.h>
#include <RadioLib.h>

#define RADIO_NSS   18
#define RADIO_IRQ   26
#define RADIO_RST   23
#define RADIO_GPIO  33

#define LORA_FREQ_MHZ   915.0
#define LORA_BW_KHZ     125.0
#define LORA_SF         9
#define LORA_CR         7
#define LORA_TX_DBM     17
#define LORA_PREAMBLE   8

#define SLOT_MS         450          // per-node slot (must match node)
#define MIN_SLOTS       4

#define NODE_PACKET_MAGIC    0x57
#define BEACON_PACKET_MAGIC  0x42
#define PACKET_VERSION       2

#define FLAG_BARO_VALID   0x01
#define FLAG_LIGHT_VALID  0x02
#define FLAG_GPS_VALID    0x04

#pragma pack(push, 1)
struct NodePacket {
    uint8_t  magic; uint8_t version; uint8_t node_id; uint8_t flags;
    uint16_t wind_speed; uint16_t wind_speed_sd;
    uint16_t wind_dir;   uint16_t wind_dir_sd; uint16_t wind_gust;
    int16_t  temperature; uint32_t pressure; uint32_t light; uint16_t battery_mv;
    int32_t  latitude; int32_t longitude; uint16_t crc;
};
struct BeaconPacket {
    uint8_t  magic; uint8_t version; uint8_t max_slot; uint8_t flags;
    uint16_t slot_ms; uint16_t cycle_ms; uint32_t seq; uint16_t crc;
};
#pragma pack(pop)

static inline uint16_t nodeCrc16(const uint8_t* d, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)d[i] << 8;
        for (uint8_t b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}

static inline bool unpackReading(const uint8_t* buf, size_t len, NodePacket& p) {
    if (len < sizeof(NodePacket)) return false;
    memcpy(&p, buf, sizeof(NodePacket));
    if (p.magic != NODE_PACKET_MAGIC) return false;
    return nodeCrc16(buf, sizeof(NodePacket) - sizeof(uint16_t)) == p.crc;
}

// ===========================================================================
// RECEIVER RADIO
// ===========================================================================
static volatile bool loraRxFlag = false;
static void IRAM_ATTR loraOnRxISR() { loraRxFlag = true; }

class LoRaReceiver {
private:
    SX1276 radio = new Module(RADIO_NSS, RADIO_IRQ, RADIO_RST, RADIO_GPIO);
    bool initialized = false;
    uint32_t beaconSeq = 0;
public:
    bool begin() {
        Serial.print(F("[LoRa] receiver init ... "));
        int st = radio.begin(LORA_FREQ_MHZ, LORA_BW_KHZ, LORA_SF, LORA_CR,
                             RADIOLIB_SX127X_SYNC_WORD, LORA_TX_DBM, LORA_PREAMBLE, 0);
        if (st != RADIOLIB_ERR_NONE) { Serial.printf("FAILED %d\n", st); return false; }
        radio.setPacketReceivedAction(loraOnRxISR);
        radio.startReceive();
        initialized = true;
        Serial.printf("OK (SF%d, coordinator)\n", LORA_SF);
        return true;
    }
    bool isReady() const { return initialized; }
    void rearm()  { if (initialized) radio.startReceive(); }
    bool restart(){ initialized = false; return begin(); }

    // Broadcast the schedule, then return to listening.
    bool sendBeacon(uint8_t maxSlot, uint16_t slotMs, uint16_t cycleMs) {
        if (!initialized) return false;
        BeaconPacket b;
        b.magic = BEACON_PACKET_MAGIC; b.version = PACKET_VERSION;
        b.max_slot = maxSlot; b.flags = 0;
        b.slot_ms = slotMs; b.cycle_ms = cycleMs; b.seq = ++beaconSeq;
        b.crc = nodeCrc16((uint8_t*)&b, sizeof(BeaconPacket) - sizeof(uint16_t));
        int st = radio.transmit((uint8_t*)&b, sizeof(b));
        radio.startReceive();
        return st == RADIOLIB_ERR_NONE;
    }

    // Non-blocking node-data receive.
    bool poll(NodePacket& pkt, float& rssi, float& snr) {
        if (!initialized || !loraRxFlag) return false;
        loraRxFlag = false;
        uint8_t buf[64];
        size_t len = radio.getPacketLength();
        if (len == 0 || len > sizeof(buf)) { radio.startReceive(); return false; }
        int st = radio.readData(buf, len);
        rssi = radio.getRSSI(); snr = radio.getSNR();
        radio.startReceive();
        if (st != RADIOLIB_ERR_NONE) return false;
        return unpackReading(buf, len, pkt);   // ignores beacons (wrong magic)
    }
};

#endif // TELEMETRY_H
