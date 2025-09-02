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

#define TTGO_T_BEAM_V1_1 // comment this line if you want to use TTGO_TBEAM_SUPREME

#define IS_TARGET               // Uncomment this line if this device is a target device
#define TARGET_ADDRESS 1 // Set target device address  Node address (1-10)
// String myAddress = "NODE" + String(TARGET_ADDRESS); // Set my address
#define USER_ADDRESS 0     // Set user device address
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
// #define CHANNEL_A 15
// #define CHANNEL_B 35
#define DIRA 2 //
#define DIRB 13
#define SPDA 14
#define SPDB 25
#define BTN2 38 //Button to change IMU sensitivity

// TDMA CONFIGURATION
// #define SLOT_DURATION 200         //each slot lasts 200ms 
// #define FRAME_DURATION 2000       //each frame lasts 2s
// #define MY_NODE_SLOT (TARGET_ADDRESS)   //node sends in the slot corresponding to address
// unsigned long syncTime = (TARGET_ADDRESS - 1) * SLOT_DURATION;       //start time of the current frame 0 - 1800ms (0-node1, 200-node2, ..., 1800-node10)
// int currentFrame = 0;             //current frame ID

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
LoRaRadioBoards *loraModule = nullptr; // LoRa module object
// External Notification object
ExternalNotification *externalNotification = nullptr;
// Power object
PowerManagement *pmu = nullptr;

#define SPI_FREQ 4000000
#ifndef TTGO_T_BEAM_V1_1
SPIClass LoRaSPI(HSPI);
#else
SPIClass LoRaSPI(HSPI);
#endif
SPISettings spiSettings(SPI_FREQ, MSBFIRST, SPI_MODE0);
#ifdef IS_TARGET
//Change winMode Interrupt
volatile int windMode = 0;
int hit_count = 0;
// SensorQMI8658 qmi;

// added methods for sensitivity control
volatile bool changeSensitivityFlag = false;
unsigned long lastSensitivityChange = 0;
void IRAM_ATTR handleSensitivityButton() {
    unsigned long now = millis();
    if (now - lastSensitivityChange > 200) {
        changeSensitivityFlag = true;
        lastSensitivityChange = now;
        Serial.println("BTN2 pressed!");
    }
}

#endif
#ifndef TTGO_T_BEAM_V1_1
bool interruptFlag = false;

void setFlag(void)
{
    interruptFlag = true;
}
#endif
void setup()
{
    delay(2000);
    Serial.begin(115200);

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

    // // Initialize the GPS module (use appropriate pins for your board)
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
    
    if (motion != nullptr) {
    //motion->begin();  // 
    Serial.print("Initial IMU Sensitivity: ");
    Serial.println(motion->getSensitivity());
    }

    // Create the wind environment object with encoder pins
    Serial.println("Setting Wind Sensor");
    environment = new WindEnvironment(DIRA, DIRB, SPDA, SPDB); // Replace with your encoder pin numbers -> Ngọc fix
    environment->begin();
    Serial.println("Environment instance initialized");
   
    // Set up button for changing IMU sensitivity
    pinMode(BTN2, INPUT_PULLUP); // Button to change IMU sensitivity
    attachInterrupt(digitalPinToInterrupt(BTN2), handleSensitivityButton, FALLING);
    Serial.println("Ready to detect BTN2 press...");
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
    loraModule = new LoRaRadioBoards(String(TARGET_ADDRESS), TARGET_ADDRESS);
#endif
#else
#ifndef TTGO_T_BEAM_V1_1
    loraModule = new LoRaRadioBoards(USER_ADDRESS, LoRaSPI, spiSettings);
#else
    loraModule = new LoRaRadioBoards(USER_ADDRESS);
#endif
#endif
    loraModule->begin();
    Serial.println("LoRa module initialized");
}
int windSpeed = 0;
double latitude = 0, longitude = 0;
unsigned long previousMillis = 0;
const unsigned long interval = 10000; // Update GPS and battery status every 10s
float batteryPercentage = 0;
float batteryVoltage = 0;
int windDirection = 0;
//bool hitFlag = false;
unsigned long hitTimestamp = 0;

String receivedMessage;
void loop()
{
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval)
    {
        previousMillis = currentMillis; //

        batteryPercentage = pmu->getBatteryPercentage();
        batteryVoltage = pmu->getBatteryVoltage();

        Serial.printf("Battery: %.2f%%, Voltage: %.2fV\n", batteryPercentage, batteryVoltage);
    }
    if (gps->getCoordinates(current_latitude, current_longtitude))
    {
        Serial.print("Latitude: ");
        Serial.print(current_latitude, 6);
        Serial.print(", Longitude: ");
        Serial.println(current_longtitude, 6);
    }

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
        // hitTimestamp = millis();
    }

#endif
    windDirection = environment->getWindDirection();
windSpeed = environment->getWindSpeed();

// Convert RPM to MPH
float windSpeedMPH = 0.0384 * windSpeed - 1.613;
if (windSpeedMPH < 0) windSpeedMPH = 0; // Avoid negative values

Serial.print("Wind Direction: ");
Serial.println(windDirection);
Serial.print("Wind Speed: ");
Serial.print(windSpeedMPH);
Serial.println("mph");

    
     //Check if the motion object is initialized before using it
    if (changeSensitivityFlag && motion != nullptr) {
        changeSensitivityFlag = false;
        motion->increaseSensitivity();
        Serial.print("New IMU Sensitivity: ");
        Serial.println(motion->getSensitivity());
    }
    //Get the current IMU sensitivity
    int currentIMUSensitivity = motion->getSensitivity();
    Serial.print("Current IMU Sensitivity: ");
    Serial.println(currentIMUSensitivity);

    char targetAddressStr[4];  // enough for 3 digits plus null terminator
    sprintf(targetAddressStr, "%d", TARGET_ADDRESS);
    if(hit_count) hitFlag = true;
    screen->drawInterface(batteryPercentage, 0, hitFlag, 0, targetAddressStr, windDirection, windSpeed, currentIMUSensitivity);
    Serial.println("Sending environment data...");
    loraModule->sendEnvironment(windSpeedMPH, hitFlag, windDirection, current_latitude, current_longtitude, currentIMUSensitivity, USER_ADDRESS);
    Serial.println("Message sent!");
    hitTimestamp = millis();
    if(hitFlag & hit_count < 4)
    {
        hit_count++;
        Serial.println("Motion detected! Hit flag set to true.");
        hitFlag = true;
    }
    else
    {
        hitFlag = false;
        hit_count = 0;
    }
    // while(hitFlag)
    // {
    //     if(hit_count < 4)
    //     {
            
    //         loraModule->sendAlert("Hit!!!", USER_ADDRESS);
    //         screen->drawInterface(batteryPercentage, 0, hitFlag, 0, targetAddressStr, windDirection, windSpeed, currentIMUSensitivity);
    //         hit_count++;
    //     } else hitFlag = false;
    //     externalNotification->notify();
    // }
#else

    String receivedMessage;
    hitFlag = false;
    if (loraModule->receiveMessage(receivedMessage))
    {
        String senderAddress, payload, messageType;
        loraModule->parseMessage(receivedMessage, senderAddress, messageType, payload);
        Serial.print("Tbeam:Received message from: ");
        Serial.println(senderAddress);
        if (senderAddress != TARGET_ADDRESS)
            return;
        Serial.println(messageType);
        if (messageType == "ALERT")
        {
            hitFlag = true; // Set hitFlag to true if an alert is received
            Serial.println("Target was hit!!");
            screen->drawHitNotification(); //Comment this line if you want to test
            hitFlag = false; // Reset hitFlag after notification
            delay(1000);
            
        }
        Serial.print("Hit Flag: ");
        Serial.println(hitFlag);
        Serial.print("Payload: ");
        Serial.println(payload);
        // Parse environment payload if applicable

        loraModule->parseEnvironmentPayload(payload, windSpeed, latitude, longitude);
        Serial.print("Wind Speed: ");
        Serial.println(windSpeed);
        Serial.print("Latitude: ");
        Serial.println(latitude, 6);
        Serial.print("Longitude: ");
        Serial.println(longitude, 6);

        // Retrieve and print signal strength
        float rssi, snr;
        loraModule->getSignalStrength(rssi, snr);
        Serial.print("RSSI: ");
        Serial.print(rssi);
        Serial.println(" dBm");
        Serial.print("SNR: ");
        Serial.print(snr);
        Serial.println(" dB");

        double distance = GPS::calculateDistance(current_latitude, current_longtitude, latitude, longitude);
        // Update the screen with the received data
        screen->drawInterface(batteryPercentage, rssi, hitFlag, distance, senderAddress.c_str(), windSpeed, 0, temperatureF, pressureInHg);
        // Clear the received message for the next iteration
        receivedMessage = "";
    }
    // if (hitFlag == true)
    // {
    //     Serial.println("Target was hit!!");
    //     // screen->drawHitNotification(); //Comment this line if you want to test
    //     externalNotification->notify();
    //     delay(100);
    //     hitFlag = false; // Reset hitFlag after notification
    // }
#endif
#ifdef TARGET
    delay(500); // Small delay to reduce processing load
#endif
    // delay(500); // Small delay to reduce processing load

        // Main loop for dynamic updates
}

