#include <Arduino.h>
#include <Adafruit_I2CDevice.h>
#include "Motion.h"
#include <SPI.h>
#include "Screen.h"
#include <Adafruit_Sensor.h>
#include "Environment.h"
#include "Telemetry.h"
#include "Location.h"
#include "ExternalNotification.h"
#include "Power.h"
#include <Wire.h>
#include "SensorQMI8658.hpp"
#include "BarometricPressure.h"
#define TTGO_T_BEAM_V1_1 // comment this line if you want to use TTGO_TBEAM_SUPREME

#define MAX_TARGETS 2
// float frequencyList[] = {
//   433.1F, 433.35F
//   };

// const int numFrequencies = sizeof(frequencyList) / sizeof(frequencyList[0]);
const int TARGET_ADDRESSES[MAX_TARGETS] = { 1, 2};

int TARGET_INDEX = 0; // Default target index
int TARGET_ADDRESS = TARGET_ADDRESSES[TARGET_INDEX];
// String myAddress = "NODE" + String(TARGET_ADDRESS); // Set my address
unsigned long lastTimeSync = 0;
const unsigned long frameDuration = 500;  // "2 seconds per frame"
unsigned long lastHitTime = 0;
unsigned long frame_id = 0;

#define USER_ADDRESS (0) // Set user device address
// GPS_PORT
#ifdef TTGO_T_BEAM_V1_1
#define GPS_RX_PIN 34
#define GPS_TX_PIN 12
#else
#define GPS_RX_PIN 9
#define GPS_TX_PIN 8
#define GPS_WAKEUP_PIN 7
#define GPS_1PPS_PIN 6
#endif
// BUZZER_PORT
#define BUZZER_PORT 4
// ENCODER_PORT
#ifdef TTGO_T_BEAM_V1_1
#define CHANNEL_A 15
#define CHANNEL_B 35
// BUTTON_PORT
#define button_channel 38 // Button change channel
// #define button_channel 25
#else
#define CHANNEL_A 48
#define CHANNEL_B 21
#endif
// ENCODER_PORT
#ifdef TTGO_T_BEAM_V1_1
#define PMU_SCL 22
#define PMU_SDA 21
#else
#define PMU_SCL 41
#define PMU_SDA 42
#endif

#ifdef TTGO_T_BEAM_V1_1
#else
#define LORA_SCK 12
#define LORA_MISO 13
#define LORA_MOSI 11
#define LORA_CS 10
#endif

// USE SPI
#define SPI_MOSI (35)

#define SPI_SCK (36)

#define SPI_MISO (37)

#define IMU_CS (34)

#define IMU_INT (33)

// Screen object
Screen *screen = nullptr;
// Global motion object
Motion *motion = nullptr;
bool hitFlag = false;
// Wind object
WindEnvironment *environment = nullptr;
// GPS object
GPSModule *gps = nullptr;
double current_longtitude = 0.0;
double current_latitude = 0.0;
// Telemetry object
LoRaModule *loraModule = nullptr;

// External Notification object
ExternalNotification *externalNotification = nullptr;
// Power object
PowerManagement *pmu = nullptr;

//Loan edit: Barometric Pressure Sensor
BarometricSensor* baroSensor = nullptr;  // BMP085 pressure & temperature sensor
float temperatureF = 0.0;
float pressureInHg = 0.0;
//Loan edit: Barometric Pressure Sensor

#define SPI_FREQ 4000000
#ifndef TTGO_T_BEAM_V1_1
SPIClass LoRaSPI(HSPI);
#else
SPIClass LoRaSPI(HSPI);
#endif
SPISettings spiSettings(SPI_FREQ, MSBFIRST, SPI_MODE0);
#ifdef IS_TARGET
// SensorQMI8658 qmi;
#endif
#ifndef TTGO_T_BEAM_V1_1
bool interruptFlag = false;

void setFlag(void)
{
    interruptFlag = true;
}
#endif
volatile unsigned switch_channel = 0;
unsigned long lastSensitivityChange = 0;

// Interrupt function to change channel 1-10

void IRAM_ATTR handleSensitivityButton() {
    unsigned long now = millis();
    if (now - lastSensitivityChange > 200) {
        switch_channel++;
        if(switch_channel >= MAX_TARGETS) switch_channel = 0;
        lastSensitivityChange = now;
        TARGET_ADDRESS = TARGET_ADDRESSES[switch_channel];
        Serial.print("Target address changed to: ");
        Serial.println(TARGET_ADDRESS);
        Serial.println("button_channel pressed!");
    }
}

void setup()
{
    delay(2000);
    Serial.begin(115200);
    pinMode(button_channel, INPUT_PULLUP); // Set button pin as input with pull-up resistor
    attachInterrupt(digitalPinToInterrupt(button_channel), handleSensitivityButton, FALLING);
    TARGET_ADDRESS = TARGET_ADDRESSES[switch_channel];
#ifdef IS_TARGET
#ifndef TTGO_T_BEAM_V1_1
    qmi.setPins(IMU_INT);
    if (!qmi.begin(IMU_CS, SPI_MOSI, SPI_MISO, SPI_SCK))
    {
        Serial.println("Failed to find QMI8658 - check your wiring!");
        while (1)
        {
            delay(1000);
        }
    }
    /* Get chip id*/
    Serial.print("Device ID:");
    Serial.println(qmi.getChipID(), HEX);

    //** The recommended output data rate for detection is higher than 500HZ
    qmi.configAccelerometer(SensorQMI8658::ACC_RANGE_4G, SensorQMI8658::ACC_ODR_500Hz);

    // Enable the accelerometer
    qmi.enableAccelerometer();

    //* Configure the motion detection axis direction
    uint8_t modeCtrl = SensorQMI8658::ANY_MOTION_EN_X |
                       SensorQMI8658::ANY_MOTION_EN_Y |
                       SensorQMI8658::ANY_MOTION_EN_Z |
                       SensorQMI8658::NO_MOTION_EN_X |
                       SensorQMI8658::NO_MOTION_EN_Y |
                       SensorQMI8658::NO_MOTION_EN_Z;

    //* Define the slope threshold of the x-axis for arbitrary motion detection
    float AnyMotionXThr = 100.0; //  x-axis 100mg threshold
    //* Define the slope threshold of the y-axis for arbitrary motion detection
    float AnyMotionYThr = 100.0; //  y-axis 100mg threshold
    //* Define the slope threshold of the z-axis for arbitrary motion detection
    float AnyMotionZThr = 1.0; //  z-axis 1mg threshold
    //* Defines the minimum number of consecutive samples (duration) that the absolute
    //* of the slope of the enabled axis/axes data should keep higher than the threshold
    uint8_t AnyMotionWindow = 1; //  1 samples

    // TODO: No motion detection does not work
    //* Defines the slope threshold of the x-axis for no motion detection
    float NoMotionXThr = 0.1;
    //* Defines the slope threshold of the y-axis for no motion detection
    float NoMotionYThr = 0.1;
    //* Defines the slope threshold of the z-axis for no motion detection
    float NoMotionZThr = 0.1;

    //* Defines the minimum number of consecutive samples (duration) that the absolute
    //* of the slope of the enabled axis/axes data should keep lower than the threshold
    uint8_t NoMotionWindow = 1; //  1 samples
    //* Defines the wait window (idle time) starts from the first Any-Motion event until
    //* starting to detecting another Any-Motion event form confirmation
    uint16_t SigMotionWaitWindow = 1; //  1 samples
    //* Defines the maximum duration for detecting the other Any-Motion
    //* event to confirm Significant-Motion, starts from the first Any -Motion event
    uint16_t SigMotionConfirmWindow = 1; //  1 samples

    qmi.configMotion(modeCtrl,
                     AnyMotionXThr, AnyMotionYThr, AnyMotionZThr, AnyMotionWindow,
                     NoMotionXThr, NoMotionYThr, NoMotionZThr, NoMotionWindow,
                     SigMotionWaitWindow, SigMotionConfirmWindow);

    // Enable the Motion Detection and enable the interrupt
    qmi.enableMotionDetect(SensorQMI8658::INTERRUPT_PIN_1);

    /*
     * When the QMI8658 is configured as Wom, the interrupt level is arbitrary,
     * not absolute high or low, and it is in the jump transition state
     */
    attachInterrupt(IMU_INT, setFlag, CHANGE);
#else
    Wire.begin(21, 22);
#endif
#endif
    Wire.begin(21, 22);
    externalNotification = new Buzzer(BUZZER_PORT);
    // Create the appropriate screen instance

    screen = ScreenFactory::createScreen();
    screen->begin();
    Serial.println("Screen instance created");
    // Initialize the Power Management Unit (AXP202X)
    pmu = new AXPManagement(PMU_SDA, PMU_SCL);
    Serial.println("Power Management Unit initialized");
    if (pmu->init())
    {
        Serial.println("Power Management Unit initialized successfully.");
    }
    else
    {
        Serial.println("Failed to initialize Power Management Unit.");
    }

// Initialize barometric pressure sensor
    baroSensor = new BMP085Sensor();
    if (!baroSensor->init()) {
        Serial.println("Failed to initialize BMP085 sensor.");
    } else {
        Serial.println("BMP085 sensor initialized successfully.");
    }

// Initialize the GPS module (use appropriate pins for your board)
    gps = new GPSModule(Serial1, GPS_RX_PIN, GPS_TX_PIN); // Replace with your GPS module pins
    Serial.println("GPS module initialized");
    gps->begin();
    gps->getCoordinates(current_latitude, current_longtitude);

#ifdef IS_TARGET
    // Choose between MPU6050 (I2C) or QMI8658 (SPI)
#ifdef TTGO_T_BEAM_V1_1
    bool useMPU6050 = true; // Set to true to use MPU6050, false for QMI8658
#else
    bool useMPU6050 = false; // Set to true to use MPU6050, false for QMI8658
#endif
    if (useMPU6050)
    {
        motion = MotionFactory::createMotion(true);
    }
    else
    {
        // Pins for QMI8658
        int csPin = 34;  // CS pin
        int intPin = 33; // Interrupt pin
        motion = MotionFactory::createMotion(false, csPin, intPin);
    }
    Serial.println("Motion instance created");
    // Initialize the chosen motion sensor
    motion->begin();
    // Create the wind environment object with encoder pins
    Serial.println("Setting Wind Sensor");
    environment = new WindEnvironment(CHANNEL_A, CHANNEL_B); // Replace with your encoder pin numbers
    environment->begin();
    Serial.println("Environment instance initialized");
#endif
#ifndef TTGO_T_BEAM_V1_1
    // Initialize the LoRa module (use appropriate pins for your board)
    LoRaSPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
    Serial.println("Config LORASPI ...");
    LoRaSPI.setFrequency(4000000);
    Serial.println("Setting LORASPI successfully");
#endif
#ifdef IS_TARGET
#ifndef TTGO_T_BEAM_V1_1
    loraModule = new LoRaRadioBoards(TARGET_ADDRESS, LoRaSPI, spiSettings);
#else
    loraModule = new LoRaRadioBoards(TARGET_ADDRESS);
#endif
#else
#ifndef TTGO_T_BEAM_V1_1
    loraModule = new LoRaRadioBoards(USER_ADDRESS, LoRaSPI, spiSettings);
#else
    loraModule = new LoRaRadioBoards(String(USER_ADDRESS), 1);
#endif
#endif
    loraModule->begin();
    Serial.println("LoRa module initialized");
}
int windSpeed = 0;
double latitude = 0.0, longitude = 0.0;
unsigned long previousMillis = 0;
const unsigned long interval = 10000; // Update GPS and battery status every 10s
float batteryPercentage = 0;
float batteryVoltage = 0;
int windDirection = 0;
int windMode = 0;
int IMUsensitivity = 0; // Initialize IMU sensitivity
void loop()
{
// --------------------------------------USER---------------------------------------------
    unsigned long currentMillis = millis();
    unsigned long now = millis();
// ---------------------------------------Battery----------------------------------------
    if (currentMillis - previousMillis >= interval)
    {
        previousMillis = currentMillis; //  Update the previous time

        batteryPercentage = pmu->getBatteryPercentage();
        batteryVoltage = pmu->getBatteryVoltage();

        Serial.printf("Battery: %.2f%%, Voltage: %.2fV\n", batteryPercentage, batteryVoltage);
    }
// ---------------------------------------Time sync every 2 seconds ------------------------------
    // if (now - lastTimeSync >= frameDuration) {
    //     lastTimeSync = now;
    //     frame_id++;
    //     if(frame_id > 99) frame_id = 0;
    //     String timeSyncPayload = "TimeSync|" + String(frame_id) + "|0";
    //     bool sent = loraModule->sendMessage(timeSyncPayload);
    //     Serial.printf("[TimeSync] Frame %d sent at %lu ms -> %s\n", frame_id, now, sent ? "OK" : "FAILED");
    //     Serial.printf("Send payload: %s\n", timeSyncPayload);
    // }
// ----------------------------------------GPS----------------------------------------
    if (gps->getCoordinates(current_latitude, current_longtitude))
    {
        Serial.print("Latitude: ");
        Serial.print(current_latitude, 6);
        Serial.print(", Longitude: ");
        Serial.println(current_longtitude, 6);
    }

    
    // switch channel 1 - 10
    TARGET_ADDRESS = TARGET_ADDRESSES[switch_channel];
   
#ifdef IS_TARGET
    if (motion->detectMotion())
    {
#ifndef TTGO_T_BEAM_V1_1
        interruptFlag = false;
        uint8_t status = qmi.getStatusRegister();
        Serial.printf("status:0x%X BIN:", status);
        Serial.println(status, BIN);
        hitFlag = true; // Set hitFlag to true if motion is detected
        Serial.println("Motion detected! Hit flag set to true.");
        loraModule->sendAlert("Hit!!!", USER_ADDRESS);
        delay(100);
        loraModule->sendAlert("Hit!!!", USER_ADDRESS);
        externalNotification->notify();
        if (status & SensorQMI8658::EVENT_SIGNIFICANT_MOTION)
        {
            Serial.println("Significant motion");
        }
        if (status & SensorQMI8658::EVENT_NO_MOTION)
        {
            Serial.println("No Motion");
        }
        if (status & SensorQMI8658::EVENT_ANY_MOTION)
        {
            Serial.println("Any Motion");
        }
#else
        hitFlag = true; // Set hitFlag to true if motion is detected
        Serial.println("Motion detected! Hit flag set to true.");
        loraModule->sendAlert("Hit!!!", USER_ADDRESS);
        delay(100);
        loraModule->sendAlert("Hit!!!", USER_ADDRESS);
        externalNotification->notify();
#endif
    }
    else
    {
        hitFlag = false;
    }
    windDirection = environment->getWindDirection();
    Serial.print("Wind Direction: ");
    Serial.println(windDirection);
    screen->drawInterface(batteryPercentage, 0, hitFlag, 0, "...", windDirection);
    // Send a message
    if (loraModule->sendEnvironment(windDirection, current_latitude, current_longtitude, USER_ADDRESS))
    {
        Serial.println("Message sent!");
    }
#else

// ---------------------------------------- Pressure and temperature --------------------------
    if (baroSensor)
    {
        temperatureF = baroSensor->readTemperatureF();
        pressureInHg = baroSensor->readPressureInHg();
        Serial.printf("Temperature: %.2f F, Pressure: %.2f inHg\n", temperatureF, pressureInHg);
    }
    else
    {
        Serial.println("Barometric sensor not initialized.");
    }
for (int i = 1; i <= MAX_TARGETS; i++) {
    // float freq = frequencyList[i-1];
    // Serial.print("Scanning channel at ");
    // Serial.print(freq);
    // Serial.println(" MHz...");
    loraModule = new LoRaRadioBoards(String(USER_ADDRESS), i);
    // Switch to this frequency
    loraModule->begin();

    // Listen for a short period (~80ms)
    long startTime = millis();
    
    // while (millis() - startTime < 50)
    {
        // hitFlag = false;
        static unsigned long hitNotificationTime = 0;
        String receivedMessage;
        if (loraModule->receiveMessage(receivedMessage)) 
        {
            Serial.printf("Current target: %d\n", TARGET_ADDRESS);
            String senderAddress, payload, messageType;
            loraModule->parseMessage(receivedMessage, senderAddress, messageType, payload);

            Serial.printf("\n[Received] From Node: %s\n", senderAddress.c_str());
            Serial.printf("Message Type: %s\n", messageType.c_str());
            Serial.printf("Payload: %s\n", payload.c_str());

            float rssi, snr;
            loraModule->getSignalStrength(rssi, snr);
            Serial.printf("RSSI: %.2f dBm\n", rssi);
            Serial.printf("SNR: %.2f dB\n", snr);

            // if (messageType == "ALERT" && hitFlag == false) {
            //     // loraModule->ack_hit_send("user");
            //     unsigned long now_hit = millis();
            //     if (now_hit - lastHitTime > 5000)
            //     {
            //         hitFlag = true;
                    
            //         Serial.println("Target was hit!!");
            //         screen->drawHitNotification(senderAddress.toInt());
            //         externalNotification->notify();
            //         delay(1000);
            //         hitFlag = false;
            //         lastHitTime = now_hit;
            //     }
            // }
            loraModule->parseEnvironmentPayload(payload, windSpeed, hitFlag, windDirection, latitude, longitude, IMUsensitivity);
            if(hitFlag)
            {
                Serial.println("Target was hit!!");
                screen->drawHitNotification(senderAddress.toInt());
                externalNotification->notify();
                delay(1000);
                hitFlag = false;
            }
    // ---------------- If the address doesn't match the currently selected target → skip it ----------------
        // myAddress = "NODE" + String(TARGET_ADDRESS);    
            if (senderAddress.toInt() != TARGET_ADDRESS)
            {
                Serial.printf("SenderAddress: %s\n", senderAddress);
                Serial.printf("Current Address: %d\n", TARGET_ADDRESS);
                Serial.println("Skipped: Not current target!!!");
                // return;
                break;
            }
            Serial.print("Payload: ");
            Serial.println(payload);



            Serial.printf("Wind Speed: %d\n", windSpeed);
            Serial.printf("HitFlag: %d\n", hitFlag);
            Serial.printf("Wind Direction: %d\n", windDirection);
            Serial.printf("Latitude: %.6f\n", latitude);
            Serial.printf("Longitude: %.6f\n", longitude);
            Serial.printf("Temperature: %.2f °F\n", temperatureF);
            Serial.printf("Pressure: %.2f inHg\n", pressureInHg);
            Serial.printf("IMU Sensitivity: %d\n", IMUsensitivity);
            
            
            // float rssi, snr;
            // loraModule->getSignalStrength(rssi, snr);
            // Serial.printf("RSSI: %.2f dBm\n", rssi);
            // Serial.printf("SNR: %.2f dB\n", snr);

            double distance = GPS::calculateDistance(current_latitude, current_longtitude, latitude, longitude);
            screen->drawInterface(batteryPercentage, rssi, hitFlag, distance, senderAddress.toInt(), windDirection, windSpeed, temperatureF, pressureInHg, IMUsensitivity);
            //hitFlag = false;
        }
    //   int state = radio.receive(msg);

    //   if (state == RADIOLIB_ERR_NONE) {
    //     Serial.print("[Received @ ");
    //     Serial.print(freq);
    //     Serial.print(" MHz] ");
    //     Serial.println(msg);
    //   }
    }
  }


    // TARGET_ADDRESS = TARGET_ADDRESSES[switch_channel];
    // if (hitFlag && (millis() - hitNotificationTime >= 500)) {
    //     hitFlag = false;
    // }

    }
#endif
#ifdef TARGET
    delay(500); // Small delay to reduce processing load
#endif



