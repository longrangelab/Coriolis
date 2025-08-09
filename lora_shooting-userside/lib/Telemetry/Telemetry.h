#include "RadioLib.h"
#define TTGO_T_BEAM_V1_1 // comment this line if you want to use TTGO_TBEAM_SUPREME
// #define TTGO_T_BEAM_SUPREME
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
// Abstract LoRa class
class LoRaModule {
public:
  virtual void begin() = 0;       // Initialize LoRa module
  virtual bool sendMessage(const String &message) = 0; // Send a message
  virtual bool receiveMessage(String &message) = 0;    // Receive a message
  virtual bool sendAlert(const String &alertMessage, const int &targetAddress) = 0; // Send an alert
  virtual bool sendEnvironment(int windSpeed, int windMode, int windDirection, double latitude, double longitude,int IMUsensitivity, const int &targetAddress) = 0; // Send environment data
  virtual void parseMessage(const String &message, String &senderAddress, String &messageType,String &payload) = 0; // Parse a message
  virtual void parseEnvironmentPayload(const String &payload,int &windSpeed, bool &hitFlag, int &windDirection, double &latitude, double &longitude, int &IMUsensitivity) = 0; // Parse environment payload
  virtual void getSignalStrength(float &rssi, float &snr) =0; // Get signal strength
};

// Implementation using RadioBoards for auto-detect
class LoRaRadioBoards : public LoRaModule {
private:
#ifdef TTGO_T_BEAM_V1_1
  SX1276 radio;
#else
  SX1262 radio;
#endif
  String deviceAddress;

  // inline static constexpr float frequencyList[11] = {
  // 0, 433.1F, 433.35F, 433.6F, 433.85F, 434.1F,
  // 434.35F, 434.6F, 434.85F, 435.6F, 435.8F
  // };
// inline static constexpr float frequencyList[11] = {
//   0.00F, 902.0F, 904.9F, 907.8F, 910.7F, 913.6F,
//   916.5F, 919.4F, 922.3F, 925.2F, 928.0F
// };
  inline static constexpr float frequencyList[11] = {
    0.00F, 914.1F, 914.35F, 914.6F, 914.85F, 915.1F,
    915.35F, 915.6F, 915.85F, 916.1F, 916.35F
  };
  int deviceID; // index from 0 to 9

public:
#ifdef TTGO_T_BEAM_V1_1
  LoRaRadioBoards(const String &address, int id) : radio(new Module(RADIO_NSS, RADIO_IRQ, RADIO_RST, RADIO_GPIO)), deviceAddress(address), deviceID(id) {}
#else
  // LoRaRadioBoards(const String &address) : radio(new Module(RADIO_NSS, RADIO_DIO1, RADIO_RST,RADIO_BUSY)), deviceAddress(address) {}
  LoRaRadioBoards(const String &address, SPIClass &spi, SPISettings spiSettings) 
    : radio(new Module(RADIO_NSS, RADIO_DIO1, RADIO_RST, RADIO_BUSY, spi, spiSettings)), deviceAddress(address), deviceID(id) {}
#endif
  void begin() override {
    Serial.print(F("[RadioBoards] Initializing ... "));
    //int state = radio.begin(915.0, 125.0, 9, 7, RADIOLIB_SX127X_SYNC_WORD, 10, 8, 0);
    //int state = radio.begin();
    float freq = frequencyList[deviceID];
    //     int state = radio.begin(
    //   434.0F,   // freq in MHz
    //   125.0F,   // bw in kHz
    //   9,        // sf (Spreading Factor)
    //   7,        // cr (Coding Rate)
    //   18,       // syncWord
    //   10,       // power in dBm
    //   8,        // preambleLength
    //   0         // gain
    // );

    int state =  radio.begin(
      freq,
      125.0F,
      9,
      7,
      18,
      10,
      8,
      0
    );

    if (state == RADIOLIB_ERR_NONE) {
      Serial.println(F("success!"));
    } else {
      Serial.print(F("failed, code "));
      Serial.println(state);
      while (true) {
        delay(10);
      }
    }
  }

  bool sendMessage(const String &message) override {
    int state = radio.transmit(message.c_str());
    if (state == RADIOLIB_ERR_NONE) {
      Serial.println(F("[RadioBoards] Message sent successfully!"));
      return true;
    } else {
      Serial.print(F("[RadioBoards] Transmit failed, code: "));
      Serial.println(state);
      return false;
    }
  }


  bool receiveMessage(String &message) override {
    String receivedMessage;
    int state = radio.receive(receivedMessage);
    if (state == RADIOLIB_ERR_NONE) {
      message = receivedMessage;
      Serial.println(F("[RadioBoards] Received successfully!"));
      return true;
    } else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
      Serial.println(F("[RadioBoards] Receive timeout!"));
    } else {
      Serial.print(F("[RadioBoards] Receive failed, code: "));
      Serial.println(state);
    }
    return false;
  }

  bool sendAlert(const String &alertMessage, const int &targetAddress) override {
    //unsigned long timestamp = (millis() + 10000) / 50;;
    String message = "ALERT|" + deviceAddress + "|" + String(targetAddress) + "|" + alertMessage;
    return sendMessage(message);
  }

  bool sendEnvironment(int windSpeed, int windMode,int windDirection, double latitude, double longitude, int IMUsensitivity, const int &targetAddress) override {
    //unsigned long timestamp = (millis() + 10000) / 50; // Timestamp in seconds
    String message = "ENVIRONMENT|" + deviceAddress + "|" + String(targetAddress) + "|" +
                     String(windSpeed) + "|"+ String(windMode) + "|" + String(windDirection) + "|" + String(latitude, 6) + "|" + String(longitude, 6) + "|" + String(IMUsensitivity);
    return sendMessage(message);
  }


  void parseMessage(const String &message, String &senderAddress, String &messageType,String &payload) override {
    Serial.println(message);
    int firstDelim = message.indexOf('|');
    int secondDelim = message.indexOf('|', firstDelim + 1);
    messageType = message.substring(0, firstDelim);
    // Serial.println(messageType);
    senderAddress = message.substring(firstDelim + 1, secondDelim);
    // Serial.println(secondDelim);
    payload = message.substring(secondDelim + 1,message.length());
    // Serial.println("[Telemetry] playload :"+payload);
  }

   void parseEnvironmentPayload(const String &payload,int &windSpeed, bool &hitFlag,int &windDirection, double &latitude, double &longitude, int &IMUsensitivity) override {
    
    int p[9]; // Position of 9 '|' 
    int pos = 0;
    for (int i = 0; i < 8; ++i) {
        p[i] = payload.indexOf('|', pos);
        pos = p[i] + 1;
    }

    // 0|0|0|12|0.000000|0.000000|86.72|29.61|20
    windSpeed      = payload.substring(p[0] + 1, p[1]).toInt();         // 0
    hitFlag       = payload.substring(p[1] + 1, p[2]).toInt();         // 0
    windDirection  = payload.substring(p[2] + 1, p[3]).toInt();         // 12
    latitude       = payload.substring(p[3] + 1, p[4]).toDouble();      // 0.000000
    longitude      = payload.substring(p[4] + 1, p[5]).toDouble();      // 0.000000
    // temperatureF   = payload.substring(p[5] + 1, p[6]).toFloat();       // 86.72
    // pressureInHg   = payload.substring(p[6] + 1, p[7]).toFloat();       // 29.61
    IMUsensitivity = payload.substring(p[5] + 1).toInt();               // 20
    //timestamp      = payload.substring(p[6] + 1).toInt();
  }

   void getSignalStrength(float &rssi, float &snr) override {
    rssi = radio.getRSSI(); // Received Signal Strength Indicator in dBm
    snr = radio.getSNR();   // Signal-to-Noise Ratio in dB
  }
};