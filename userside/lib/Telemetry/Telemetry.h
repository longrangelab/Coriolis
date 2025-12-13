#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "RadioLib.h"
#include "telemetry.pb.h"
#include <pb_encode.h>
#include <pb_decode.h>

#define TTGO_T_BEAM_V1_1 // comment this line if you want to use TTGO_TBEAM_SUPREME
// #define TTGO_T_BEAM_SUPREME

// User-side specific configuration
#define MAX_TARGETS 10                   // Maximum number of targets in system
#define TARGET_TIMEOUT_MS 60000          // 60 seconds timeout for target data (FASTER)
#define ALERT_ACKNOWLEDGMENT_ENABLED     // Enable alert acknowledgments
#define TARGET_STATUS_UPDATE_INTERVAL 15000 // Update target status every 15s (FASTER)

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

// Maximum buffer size for serialized messages
#define MAX_MESSAGE_SIZE 128

// Message types for backward compatibility
enum MessageType {
  MSG_ALERT = 1,
  MSG_ENVIRONMENT = 2,
  MSG_ACK = 3
};

// Target information structure
struct TargetInfo {
  String targetId;
  unsigned long lastSeen;
  unsigned long lastAlertTime;
  int windSpeed;
  int windMode;
  int windDirection;
  double latitude;
  double longitude;
  int imuSensitivity;
  float rssi;
  float snr;
  bool isActive;
  uint32_t alertCount;
  uint32_t environmentCount;
};

// Abstract LoRa class with protobuf support for user-side
class LoRaModule {
public:
  virtual void begin() = 0;       // Initialize LoRa module
  virtual bool sendMessage(const uint8_t *buffer, size_t length) = 0; // Send binary message
  virtual bool receiveMessage(uint8_t *buffer, size_t &length) = 0;    // Receive binary message
  virtual bool receiveMessage(String &message) = 0;    // Receive string message (for compatibility)
  virtual bool sendAlert(const String &alertMessage, const String &targetAddress) = 0; // Send an alert
  virtual bool sendEnvironment(int windSpeed, int windMode, int windDirection, double latitude, double longitude,float temperatureF, float pressureInHg,int IMUsensitivity, const String &targetAddress) = 0; // Send environment data
  virtual bool parseMessage(const uint8_t *buffer, size_t length, MessageType &msgType, String &senderAddr) = 0; // Parse a binary message
  virtual void parseMessage(const String &message, String &senderAddress, String &messageType,String &payload) = 0; // Parse string message (for compatibility)
  virtual void parseEnvironmentPayload(const String &payload,int &windSpeed, int &windMode, int &windDirection, double &latitude, double &longitude, float &temperatureF, float &pressureInHg,int &IMUsensitivity) = 0; // Parse environment payload
  virtual void getSignalStrength(float &rssi, float &snr) =0; // Get signal strength
  
  // User-side specific methods
  virtual void updateTargetInfo(const String &targetId, const telemetry_TelemetryPacket &packet, float rssi, float snr) = 0;
  virtual TargetInfo* getTargetInfo(const String &targetId) = 0;
  virtual void printTargetStatus() = 0;
  virtual int getActiveTargetsCount() = 0;
  virtual void cleanupInactiveTargets() = 0;
  virtual void processPeriodicTasks() = 0;
  virtual bool hasRecentAlert(unsigned long withinMs = 5000) = 0;
};

// Implementation using RadioBoards for user-side with multi-target management
class LoRaRadioBoards : public LoRaModule {
private:
#ifdef TTGO_T_BEAM_V1_1
  SX1276 radio;
#else
  SX1262 radio;
#endif
  String deviceAddress;
  TargetInfo targets[MAX_TARGETS];
  int targetCount;
  unsigned long lastStatusUpdate;

  // Helper function to encode protobuf message
  bool encodeMessage(const telemetry_TelemetryPacket &packet, uint8_t *buffer, size_t &length) {
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, MAX_MESSAGE_SIZE);
    bool status = pb_encode(&stream, &telemetry_TelemetryPacket_msg, &packet);
    length = stream.bytes_written;
    
    Serial.print(F("[UserSide] Encoded message size: "));
    Serial.print(length);
    Serial.println(F(" bytes"));
    
    return status;
  }

  // Helper function to decode protobuf message
  bool decodeMessage(const uint8_t *buffer, size_t length, telemetry_TelemetryPacket &packet) {
    pb_istream_t stream = pb_istream_from_buffer(buffer, length);
    return pb_decode(&stream, &telemetry_TelemetryPacket_msg, &packet);
  }

  // Find target index by ID
  int findTargetIndex(const String &targetId) {
    for (int i = 0; i < targetCount; i++) {
      if (targets[i].targetId == targetId) {
        return i;
      }
    }
    return -1;
  }

  // Add new target or get existing one
  TargetInfo* getOrCreateTarget(const String &targetId) {
    int index = findTargetIndex(targetId);
    if (index >= 0) {
      return &targets[index];
    }
    
    // Add new target if space available
    if (targetCount < MAX_TARGETS) {
      targets[targetCount].targetId = targetId;
      targets[targetCount].lastSeen = 0;
      targets[targetCount].lastAlertTime = 0;
      targets[targetCount].isActive = false;
      targets[targetCount].alertCount = 0;
      targets[targetCount].environmentCount = 0;
      Serial.printf("[UserSide] Added new target: %s (slot %d)\n", targetId.c_str(), targetCount);
      return &targets[targetCount++];
    }
    
    Serial.printf("[UserSide] WARNING: Max targets reached, cannot add %s\n", targetId.c_str());
    return nullptr;
  }

public:
#ifdef TTGO_T_BEAM_V1_1
  LoRaRadioBoards(const String &address) : radio(new Module(RADIO_NSS, RADIO_IRQ, RADIO_RST, RADIO_GPIO)), 
                                          deviceAddress(address), 
                                          targetCount(0),
                                          lastStatusUpdate(0) {
    Serial.printf("[UserSide] User station %s initialized\n", address.c_str());
  }
#else
  LoRaRadioBoards(const String &address, SPIClass &spi, SPISettings spiSettings) 
    : radio(new Module(RADIO_NSS, RADIO_DIO1, RADIO_RST, RADIO_BUSY, spi, spiSettings)), 
      deviceAddress(address),
      targetCount(0),
      lastStatusUpdate(0) {
    Serial.printf("[UserSide] User station %s initialized\n", address.c_str());
  }
#endif

  void begin() override {
    Serial.print(F("[UserSide] Initializing LoRa receiver... "));
    int state = radio.begin(915.0, 125.0, 9, 7, RADIOLIB_SX127X_SYNC_WORD, 10, 8, 0);
    // int state = radio.begin();
    if (state == RADIOLIB_ERR_NONE) {
      Serial.println(F("success!"));
      // Set continuous receive mode
      radio.startReceive();
    } else {
      Serial.print(F("failed, code "));
      Serial.println(state);
      while (true) {
        delay(10);
      }
    }
  }

  bool sendMessage(const uint8_t *buffer, size_t length) override {
    Serial.print(F("[UserSide] Sending "));
    Serial.print(length);
    Serial.println(F(" bytes..."));
    
    int state = radio.transmit(buffer, length);
    if (state == RADIOLIB_ERR_NONE) {
      Serial.print(F("[UserSide] Message sent successfully! Size: "));
      Serial.print(length);
      Serial.println(F(" bytes"));
      radio.startReceive(); // Return to receive mode
      return true;
    } else {
      Serial.print(F("[UserSide] Transmit failed, code: "));
      Serial.println(state);
      radio.startReceive(); // Return to receive mode
      return false;
    }
  }

  bool receiveMessage(uint8_t *buffer, size_t &length) override {
    int state = radio.receive(buffer, length);
    if (state == RADIOLIB_ERR_NONE) {
      size_t packetLength = radio.getPacketLength();
      
      // Validate packet size to prevent buffer overflow
      if (packetLength > length) {
        Serial.printf("[UserSide] Warning: Packet too large (%d bytes), buffer only %d bytes\n", packetLength, length);
        return false;
      }
      
      length = packetLength;
      Serial.print(F("[UserSide] Binary message received: "));
      Serial.print(length);
      Serial.println(F(" bytes"));
      return true;
    } else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
      // Normal timeout, not an error for continuous receive
      return false;
    } else {
      Serial.print(F("[UserSide] Receive failed, code: "));
      Serial.println(state);
      return false;
    }
  }

  // String-based receive for compatibility
  bool receiveMessage(String &message) override {
    uint8_t buffer[MAX_MESSAGE_SIZE];
    size_t length = MAX_MESSAGE_SIZE;
    
    if (receiveMessage(buffer, length)) {
      // Try to decode as protobuf first
      telemetry_TelemetryPacket packet;
      if (decodeMessage(buffer, length, packet)) {
        // Successfully decoded protobuf - process it
        String targetId = "Unknown"; // We'll need to handle source_id properly later
        
        // Get signal strength
        float rssi = radio.getRSSI();
        float snr = radio.getSNR();
        
        // Update target info
        updateTargetInfo(targetId, packet, rssi, snr);
        
        // Format as string for compatibility
        if (packet.wind_speed == -1) {
          message = "ALERT|" + targetId + "|" + deviceAddress + "|Hit!!!";
        } else {
          message = "ENVIRONMENT|" + targetId + "|" + deviceAddress + "|" +
                   String(packet.wind_speed) + "|" + String(packet.wind_mode) + "|" +
                   String(packet.wind_direction) + "|" + String(packet.latitude, 6) + "|" +
                   String(packet.longitude, 6) + "|" + String(packet.imu_sensitivity);
        }
        return true;
      } else {
        // Fallback to string format for old messages
        message = "";
        for (size_t i = 0; i < length; i++) {
          message += (char)buffer[i];
        }
        return true;
      }
    }
    return false;
  }

  bool sendAlert(const String &alertMessage, const String &targetAddress) override {
    // User typically doesn't send alerts, but include for completeness
    telemetry_TelemetryPacket packet = telemetry_TelemetryPacket_init_zero;
    packet.wind_speed = -1; // Alert indicator
    
    uint8_t buffer[MAX_MESSAGE_SIZE];
    size_t length;
    if (encodeMessage(packet, buffer, length)) {
      return sendMessage(buffer, length);
    }
    return false;
  }

  bool sendEnvironment(int windSpeed, int windMode, int windDirection, double latitude, double longitude,float temperatureF, float pressureInHg,int IMUsensitivity, const String &targetAddress) override {
    // User typically doesn't send environment data, but include for completeness
    telemetry_TelemetryPacket packet = telemetry_TelemetryPacket_init_zero;
    packet.wind_speed = windSpeed;
    packet.wind_mode = windMode;
    packet.wind_direction = windDirection;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.imu_sensitivity = IMUsensitivity;
    
    uint8_t buffer[MAX_MESSAGE_SIZE];
    size_t length;
    if (encodeMessage(packet, buffer, length)) {
      return sendMessage(buffer, length);
    }
    return false;
  }

  bool parseMessage(const uint8_t *buffer, size_t length, MessageType &msgType, String &senderAddr) override {
    telemetry_TelemetryPacket packet;
    if (decodeMessage(buffer, length, packet)) {
      // Extract source ID from protobuf
      senderAddr = String(packet.source_id);
      
      if (packet.wind_speed == -1) {
        msgType = MSG_ALERT;
        Serial.printf("[UserSide] Received ALERT from: %d\n", packet.source_id);
      } else {
        msgType = MSG_ENVIRONMENT;
        Serial.printf("[UserSide] Received ENVIRONMENT from: %s\n", packet.source_id);
      }
      
      // Get signal strength and update target info
      float rssi = radio.getRSSI();
      float snr = radio.getSNR();
      updateTargetInfo(senderAddr, packet, rssi, snr);
      
      return true;
    }
    return false;
  }

  // String-based parse for compatibility
  void parseMessage(const String &message, String &senderAddress, String &messageType, String &payload) override {
    int firstPipe = message.indexOf('|');
    int secondPipe = message.indexOf('|', firstPipe + 1);
    int thirdPipe = message.indexOf('|', secondPipe + 1);

    if (firstPipe != -1 && secondPipe != -1 && thirdPipe != -1) {
      messageType = message.substring(0, firstPipe);
      senderAddress = message.substring(firstPipe + 1, secondPipe);
      // targetAddress = message.substring(secondPipe + 1, thirdPipe);
      payload = message.substring(thirdPipe + 1);
      
      Serial.printf("[UserSide] Parsed message: Type=%s, From=%s\n", 
                   messageType.c_str(), senderAddress.c_str());
    }
  }

  void parseEnvironmentPayload(const String &payload, int &windSpeed, int &windMode, int &windDirection, double &latitude, double &longitude, float &temperatureF, float &pressureInHg, int &IMUsensitivity) override {
    int indices[8];
    int count = 0;
    int start = 0;
    
    // Find all pipe separators
    for (int i = 0; i < payload.length() && count < 8; i++) {
      if (payload.charAt(i) == '|') {
        indices[count++] = i;
      }
    }
    
    if (count >= 5) {
      windSpeed = payload.substring(0, indices[0]).toInt();
      windMode = payload.substring(indices[0] + 1, indices[1]).toInt();
      windDirection = payload.substring(indices[1] + 1, indices[2]).toInt();
      latitude = payload.substring(indices[2] + 1, indices[3]).toDouble();
      longitude = payload.substring(indices[3] + 1, indices[4]).toDouble();
      
      if (count >= 7) {
        temperatureF = payload.substring(indices[4] + 1, indices[5]).toFloat();
        pressureInHg = payload.substring(indices[5] + 1, indices[6]).toFloat();
        if (count >= 8) {
          IMUsensitivity = payload.substring(indices[6] + 1, indices[7]).toInt();
        }
      }
    }
  }

  void getSignalStrength(float &rssi, float &snr) override {
    rssi = radio.getRSSI();
    snr = radio.getSNR();
  }

  // New method to parse protobuf message and extract all data including source ID
  bool parseProtobufMessage(const uint8_t *buffer, size_t length, 
                           String &sourceId, int &windSpeed, int &windMode, int &windDirection,
                           double &latitude, double &longitude, int &imuSensitivity) {
    
    // Add safety check for buffer length
    if (length == 0 || buffer == nullptr) {
      Serial.println("[UserSide] Invalid buffer for protobuf parsing");
      return false;
    }
    
    telemetry_TelemetryPacket packet;
    if (decodeMessage(buffer, length, packet)) {
      // Convert int32 source_id to String (handle both old char[] and new int32 formats)
      sourceId = String(packet.source_id);
      
      windSpeed = packet.wind_speed;
      windMode = packet.wind_mode;
      windDirection = packet.wind_direction;
      latitude = packet.latitude;
      longitude = packet.longitude;
      imuSensitivity = packet.imu_sensitivity;
      
      Serial.printf("[UserSide] Full protobuf data from %s:\n", sourceId.c_str());
      Serial.printf("  Wind: Speed=%d, Mode=%d, Direction=%d\n", windSpeed, windMode, windDirection);
      Serial.printf("  GPS: (%.6f, %.6f)\n", latitude, longitude);
      Serial.printf("  IMU: %d\n", imuSensitivity);
      
      return true;
    } else {
      Serial.println("[UserSide] Failed to decode protobuf message");
      return false;
    }
  }

  // User-side specific methods implementation
  void updateTargetInfo(const String &targetId, const telemetry_TelemetryPacket &packet, float rssi, float snr) override {
    TargetInfo* target = getOrCreateTarget(targetId);
    if (target) {
      target->lastSeen = millis();
      target->isActive = true;
      target->rssi = rssi;
      target->snr = snr;
      
      if (packet.wind_speed == -1) {
        // Alert message
        target->lastAlertTime = millis();
        target->alertCount++;
        Serial.printf("[UserSide] *** ALERT from %s *** (Count: %d)\n", 
                     targetId.c_str(), target->alertCount);
      } else {
        // Environment message
        target->windSpeed = packet.wind_speed;
        target->windMode = packet.wind_mode;
        target->windDirection = packet.wind_direction;
        target->latitude = packet.latitude;
        target->longitude = packet.longitude;
        target->imuSensitivity = packet.imu_sensitivity;
        target->environmentCount++;
        
        Serial.printf("[UserSide] Environment from %s: Wind=%d°@%d, GPS=(%.6f,%.6f), IMU=%d (Count: %d)\n", 
                     targetId.c_str(), target->windDirection, target->windSpeed,
                     target->latitude, target->longitude, target->imuSensitivity,
                     target->environmentCount);
      }
    }
  }

  TargetInfo* getTargetInfo(const String &targetId) override {
    int index = findTargetIndex(targetId);
    return (index >= 0) ? &targets[index] : nullptr;
  }

  void printTargetStatus() override {
    Serial.println(F("\n======== TARGET STATUS REPORT ========"));
    Serial.printf("Active Targets: %d/%d\n", getActiveTargetsCount(), targetCount);
    
    for (int i = 0; i < targetCount; i++) {
      unsigned long timeSinceLastSeen = millis() - targets[i].lastSeen;
      bool isRecent = timeSinceLastSeen < TARGET_TIMEOUT_MS;
      
      Serial.printf("Target %s: %s (Last seen: %lu ms ago)\n", 
                   targets[i].targetId.c_str(),
                   isRecent ? "ACTIVE" : "TIMEOUT",
                   timeSinceLastSeen);
      
      if (isRecent) {
        Serial.printf("  Wind: %d°@%d (Mode: %d), GPS: (%.6f,%.6f)\n",
                     targets[i].windDirection, targets[i].windSpeed, targets[i].windMode,
                     targets[i].latitude, targets[i].longitude);
        Serial.printf("  IMU: %d, RSSI: %.1f dBm, SNR: %.1f dB\n",
                     targets[i].imuSensitivity, targets[i].rssi, targets[i].snr);
        Serial.printf("  Messages: %d alerts, %d environment\n",
                     targets[i].alertCount, targets[i].environmentCount);
      }
    }
    Serial.println(F("=====================================\n"));
  }

  int getActiveTargetsCount() override {
    int activeCount = 0;
    unsigned long currentTime = millis();
    
    for (int i = 0; i < targetCount; i++) {
      if (currentTime - targets[i].lastSeen < TARGET_TIMEOUT_MS) {
        activeCount++;
      }
    }
    return activeCount;
  }

  void cleanupInactiveTargets() override {
    unsigned long currentTime = millis();
    
    for (int i = 0; i < targetCount; i++) {
      if (currentTime - targets[i].lastSeen > TARGET_TIMEOUT_MS) {
        if (targets[i].isActive) {
          Serial.printf("[UserSide] Target %s went inactive\n", targets[i].targetId.c_str());
          targets[i].isActive = false;
        }
      }
    }
  }

  // Additional user-side methods
  void processPeriodicTasks() override {
    unsigned long currentTime = millis();
    
    if (currentTime - lastStatusUpdate > TARGET_STATUS_UPDATE_INTERVAL) {
      cleanupInactiveTargets();
      printTargetStatus();
      lastStatusUpdate = currentTime;
    }
  }
  
  bool hasRecentAlert(unsigned long withinMs = 5000) override {
    unsigned long currentTime = millis();
    for (int i = 0; i < targetCount; i++) {
      if (currentTime - targets[i].lastAlertTime < withinMs) {
        return true;
      }
    }
    return false;
  }
};

#endif