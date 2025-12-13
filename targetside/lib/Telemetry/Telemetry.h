#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <Arduino.h>
#include <RadioLib.h>
#include "telemetry.pb.h"
#include <pb_encode.h>
#include <pb_decode.h>

#define TTGO_T_BEAM_V1_1 // comment this line if you want to use TTGO_TBEAM_SUPREME

#ifdef TTGO_T_BEAM_V1_1
#define RADIO_NSS     (18)
#define RADIO_IRQ     (26)
#define RADIO_RST     (23)
#define RADIO_GPIO    (33)
#else
#define RADIO_NSS     (10)
#define RADIO_RST     (5)
#define RADIO_DIO1    (1)
#define RADIO_BUSY    (4)
#endif

// Message types
enum MessageType {
  MSG_ENVIRONMENT,
  MSG_ALERT,
  MSG_UNKNOWN
};

// Simple configuration
#define ENVIRONMENT_INTERVAL_MS 10000  // Send environment data every 10 seconds (SUPER FAST!)
#define MAX_MESSAGE_SIZE 128           // Maximum buffer size for serialized messages

// Abstract LoRa class
class LoRaModule {
public:
  virtual void begin() = 0;
  virtual bool sendAlert(const String &alertMessage, const String &targetAddress) = 0;
  virtual bool sendEnvironment(int windSpeed, int windMode, int windDirection, double latitude, double longitude, int IMUsensitivity, const String &targetAddress) = 0;
  virtual bool parseMessage(const String &message, String &senderAddress, String &messageType, String &payload) = 0;
  virtual bool parseEnvironmentPayload(const String &payload, int &windSpeed, int &windMode, int &windDirection, double &latitude, double &longitude, int &IMUsensitivity) = 0;
  virtual void getSignalStrength(float &rssi, float &snr) = 0;
  virtual bool sendIMUSensitivity(int IMUsensitivity, const String &targetAddress) = 0;  // Thêm phương thức mới
  
  // New protobuf methods
  virtual bool sendMessage(const uint8_t *buffer, size_t length) = 0;
  virtual bool receiveMessage(uint8_t *buffer, size_t &length) = 0;
  
  // Backward compatibility methods
  virtual bool sendMessage(const String &message) = 0;
  virtual bool receiveMessage(String &message) = 0;
};

// Implementation using RadioBoards
class LoRaRadioBoards : public LoRaModule {
private:
#ifdef TTGO_T_BEAM_V1_1
  SX1276 radio;
#else
  SX1262 radio;
#endif
  String deviceAddress;
  unsigned long lastEnvironmentSent;
  bool initialized;

  // Helper function to encode protobuf message
  bool encodeMessage(const telemetry_TelemetryPacket &packet, uint8_t *buffer, size_t &length) {
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, MAX_MESSAGE_SIZE);
    bool status = pb_encode(&stream, &telemetry_TelemetryPacket_msg, &packet);
    length = stream.bytes_written;
    
    Serial.print(F("[Target] Encoded message size: "));
    Serial.print(length);
    Serial.println(F(" bytes"));
    
    return status;
  }

  // Helper function to decode protobuf message  
  bool decodeMessage(const uint8_t *buffer, size_t length, telemetry_TelemetryPacket &packet) {
    pb_istream_t stream = pb_istream_from_buffer(buffer, length);
    return pb_decode(&stream, &telemetry_TelemetryPacket_msg, &packet);
  }

public:
#ifdef TTGO_T_BEAM_V1_1
  LoRaRadioBoards(const String &address) : radio(new Module(RADIO_NSS, RADIO_IRQ, RADIO_RST, RADIO_GPIO)), deviceAddress(address), lastEnvironmentSent(0), initialized(false) {
    // Simple constructor - no slot management needed
  }
#else
  LoRaRadioBoards(const String &address, SPIClass &spi, SPISettings spiSettings) 
    : radio(new Module(RADIO_NSS, RADIO_DIO1, RADIO_RST, RADIO_BUSY, spi, spiSettings)), deviceAddress(address), lastEnvironmentSent(0), initialized(false) {
    // Simple constructor - no slot management needed
  }
#endif

  void begin() override {
    Serial.print(F("[RadioBoards] Initializing ... "));
    int state = radio.begin(915.0, 125.0, 9, 7, RADIOLIB_SX127X_SYNC_WORD, 10, 8, 0);
    // int state = radio.begin();
    if (state == RADIOLIB_ERR_NONE) {
      Serial.println(F("success!"));
      Serial.printf("[RadioBoards] Device Address: %s\n", deviceAddress.c_str());
      initialized = true;
    } else {
      Serial.print(F("failed, code "));
      Serial.println(state);
      initialized = false;
    }
  }

  bool sendMessage(const String &message) override {
    if (!initialized) return false;
    int state = radio.transmit(message.c_str());
    if (state == RADIOLIB_ERR_NONE) {
      Serial.printf("[Target] String message sent successfully: %s\n", message.c_str());
      return true;
    } else {
      Serial.printf("[Target] Failed to send string message, code: %d\n", state);
      return false;
    }
  }

  bool sendMessage(const uint8_t *buffer, size_t length) override {
    if (!initialized) return false;
    
    Serial.print(F("[Target] Sending "));
    Serial.print(length);
    Serial.println(F(" bytes via protobuf..."));
    
    int state = radio.transmit(buffer, length);
    if (state == RADIOLIB_ERR_NONE) {
      Serial.print(F("[Target] Protobuf message sent successfully! Size: "));
      Serial.print(length);
      Serial.println(F(" bytes"));
      return true;
    } else {
      Serial.print(F("[Target] Failed to send protobuf message, code: "));
      Serial.println(state);
      return false;
    }
  }

  bool receiveMessage(String &message) override {
    if (!initialized) return false;
    int state = radio.receive(message);
    if (state == RADIOLIB_ERR_NONE) {
      Serial.printf("[Target] Received string message: %s\n", message.c_str());
      return true;
    }
    return false;
  }

  bool receiveMessage(uint8_t *buffer, size_t &length) override {
    if (!initialized) return false;
    
    int state = radio.receive(buffer, length);
    if (state == RADIOLIB_ERR_NONE) {
      Serial.print(F("[Target] Received protobuf message, size: "));
      Serial.println(length);
      return true;
    }
    return false;
  }

  bool sendAlert(const String &alertMessage, const String &targetAddress) override {
    if (!initialized) return false;
    
    // Create protobuf packet for alert
    telemetry_TelemetryPacket packet = telemetry_TelemetryPacket_init_zero;
    
    // Set source ID as integer (convert device address to integer)
    packet.source_id = deviceAddress.toInt();
    
    // Set alert indicator (use -1 for wind_speed to indicate alert)
    packet.wind_speed = -1;
    packet.wind_mode = 0;
    packet.wind_direction = 0;
    packet.latitude = 0.0;
    packet.longitude = 0.0;
    packet.imu_sensitivity = 0;
    
    // Encode and send
    uint8_t buffer[MAX_MESSAGE_SIZE];
    size_t length;
    
    if (encodeMessage(packet, buffer, length)) {
      bool result = sendMessage(buffer, length);
      if (result) {
        Serial.printf("[Target] Alert sent from %s: %s\n", deviceAddress.c_str(), alertMessage.c_str());
      }
      return result;
    } else {
      Serial.println("[Target] Failed to encode alert message");
      return false;
    }
  }

  bool sendIMUSensitivity(int IMUsensitivity, const String &targetAddress) override {
    if (!initialized) return false;
    
    // Create protobuf packet for IMU sensitivity
    telemetry_TelemetryPacket packet = telemetry_TelemetryPacket_init_zero;
    
    // Set source ID as integer
    packet.source_id = deviceAddress.toInt();
    
    // Giữ lại giá trị IMU và đặt các giá trị khác về 0
    packet.wind_speed = 0;
    packet.wind_mode = 0;
    packet.wind_direction = 0;
    packet.latitude = 0.0;
    packet.longitude = 0.0;
    packet.imu_sensitivity = IMUsensitivity;  // Gửi độ nhạy IMU
    
    // Encode và send
    uint8_t buffer[MAX_MESSAGE_SIZE];
    size_t length;
    
    if (encodeMessage(packet, buffer, length)) {
      bool result = sendMessage(buffer, length);
      if (result) {
        Serial.printf("[Target] IMU Sensitivity sent from %s: IMU=%d\n", 
                     deviceAddress.c_str(), IMUsensitivity);
      }
      return result;
    } else {
      Serial.println("[Target] Failed to encode IMU sensitivity message");
      return false;
    }
  }

  bool sendEnvironment(int windSpeed, int windMode, int windDirection, double latitude, double longitude, int IMUsensitivity, const String &targetAddress) override {
    if (!initialized) return false;
    
    // Simple rate limiting: don't send environment data too frequently
    unsigned long currentTime = millis();
    if (currentTime - lastEnvironmentSent < ENVIRONMENT_INTERVAL_MS) {
      Serial.println("[Target] Environment data sent too recently, skipping");
      return false;
    }

    // Create protobuf packet for environment data
    telemetry_TelemetryPacket packet = telemetry_TelemetryPacket_init_zero;
    
    // Set source ID as integer (convert device address to integer)
    packet.source_id = deviceAddress.toInt();
    
    // Set environment data
    packet.wind_speed = windSpeed;
    packet.wind_mode = windMode;
    packet.wind_direction = windDirection;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.imu_sensitivity = IMUsensitivity;
    
    // Encode and send
    uint8_t buffer[MAX_MESSAGE_SIZE];
    size_t length;
    
    if (encodeMessage(packet, buffer, length)) {
      bool result = sendMessage(buffer, length);
      if (result) {
        lastEnvironmentSent = currentTime;
        Serial.printf("[Target] Environment data sent from %s: Wind=%d°@%d, GPS=(%.6f,%.6f), IMU=%d\n", 
                     deviceAddress.c_str(), windDirection, windSpeed, latitude, longitude, IMUsensitivity);
      }
      return result;
    } else {
      Serial.println("[Target] Failed to encode environment message");
      return false;
    }
  }

  bool parseMessage(const String &message, String &senderAddress, String &messageType, String &payload) override {
    int firstColon = message.indexOf(':');
    int secondColon = message.indexOf(':', firstColon + 1);
    int thirdColon = message.indexOf(':', secondColon + 1);
    
    if (firstColon != -1 && secondColon != -1 && thirdColon != -1) {
      messageType = message.substring(0, firstColon);
      senderAddress = message.substring(firstColon + 1, secondColon);
      payload = message.substring(secondColon + 1, thirdColon);
      return true;
    } else {
      messageType = "UNKNOWN";
      senderAddress = "";
      payload = message;
      return false;
    }
  }

  bool parseEnvironmentPayload(const String &payload, int &windSpeed, int &windMode, int &windDirection, double &latitude, double &longitude, int &IMUsensitivity) override {
    int commaIndex[5];
    int found = 0;
    int startIndex = 0;
    
    // Find all comma positions
    for (int i = 0; i < payload.length() && found < 5; i++) {
      if (payload.charAt(i) == ',') {
        commaIndex[found] = i;
        found++;
      }
    }
    
    if (found == 5) {
      windSpeed = payload.substring(0, commaIndex[0]).toInt();
      windMode = payload.substring(commaIndex[0] + 1, commaIndex[1]).toInt();
      windDirection = payload.substring(commaIndex[1] + 1, commaIndex[2]).toInt();
      latitude = payload.substring(commaIndex[2] + 1, commaIndex[3]).toDouble();
      longitude = payload.substring(commaIndex[3] + 1, commaIndex[4]).toDouble();
      IMUsensitivity = payload.substring(commaIndex[4] + 1).toInt();
      return true;
    }
    return false;
  }

  void getSignalStrength(float &rssi, float &snr) override {
    rssi = radio.getRSSI();
    snr = radio.getSNR();
  }
};

#endif // TELEMETRY_H
