#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_Sensor.h>
#include "SensorQMI8658.hpp"
#include "Telemetry.h"  // Include Telemetry.h early to ensure classes are defined
#include "Motion.h"
#include "Screen.h"
#include "Environment.h"
#include "Location.h"
#include "ExternalNotification.h"
#include "Power.h"

#define TTGO_T_BEAM_V1_1 // comment this line if you want TTGO_TBEAM_SUPREME

#define IS_TARGET               // Uncomment this line if this device is a target device

// Configure target ID (1-10) - change this for each target device
#define TARGET_ID 1             // Change this to 1, 2, 3, 4, 5, 6, 7, 8, 9, or 10

// Auto-generate target address based on TARGET_ID
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
#define TARGET_ADDRESS TOSTRING(TARGET_ID)  // Will become "1", "2", "3", etc.

#define USER_ADDRESS "User"     // Set user device address
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
#define BUZZER_PORT 32
// ENCODER_PORT
#ifdef TTGO_T_BEAM_V1_1
// #define CHANNEL_A 15
// #define CHANNEL_B 35
#define DIRA 2 //Ngọc fix
#define DIRB 13
#define SPDA 14
#define SPDB 25 //Ngọc fix
#define BUTTON_IMU_SENSITIVITY 38 //Button to change IMU sensitivity

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
// // Telemetry object
// LoRaModule *loraModule = nullptr;

LoRaModule *loraModule = nullptr;
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
unsigned long lastInterruptTime = 0;
// SensorQMI8658 qmi;
SensorQMI8658 qmi;

// Loan added methods for sensitivity control
volatile bool changeSensitivityFlag = false;
unsigned long lastSensitivityChange = 0;

// Hit display timer - keep hit displayed for 2 seconds
unsigned long hitDisplayTime = 0;
const unsigned long HIT_DISPLAY_DURATION = 2000; // 2 seconds

// Screen update timer - update screen every 500ms
unsigned long lastScreenUpdate = 0;
const unsigned long SCREEN_UPDATE_INTERVAL = 500; // 500ms

// Battery status variables
float batteryPercentage = 0;
float batteryVoltage = 0;

void IRAM_ATTR handleSensitivityButton() {
    // Simplified ISR - minimal operations only
    static unsigned long lastPress = 0;
    unsigned long now = millis();
    
    if (now - lastPress > 500) {  // Simple debounce
        changeSensitivityFlag = true;
        lastPress = now;
    }
}
//Loan added methods for sensitivity control

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
    
    // Display target configuration
    Serial.println(" ================= TARGET CONFIGURATION =================");
    Serial.printf("TARGET ID: %d\n", TARGET_ID);
    Serial.printf("TARGET ADDRESS: %s\n", TARGET_ADDRESS);
    Serial.printf("USER ADDRESS: %s\n", USER_ADDRESS);
    Serial.println("========================================================");
    

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
    //motion->begin();  // GỌI LẠI LẦN 2
    Serial.print("Initial IMU Sensitivity: ");
    Serial.println(motion->getSensitivity());
    }

    // Create the wind environment object with encoder pins
    Serial.println("Setting Wind Sensor");

    environment = new WindEnvironment(DIRA, DIRB, SPDA, SPDB); // Replace with your encoder pin numbers -> Ngọc fix
    environment->begin();
    Serial.println("Environment instance initialized");
    // pinMode(BTN1, INPUT_PULLUP); //Change wind mode
    // attachInterrupt(digitalPinToInterrupt(BTN1), button1, FALLING);
    
    // Loan edit: Set up button for changing IMU sensitivity
    pinMode(BUTTON_IMU_SENSITIVITY, INPUT_PULLUP); // Đặt chế độ input cho BUTTON_IMU_SENSITIVITY
    attachInterrupt(digitalPinToInterrupt(BUTTON_IMU_SENSITIVITY), handleSensitivityButton, FALLING);
    Serial.println("Ready to detect BUTTON_IMU_SENSITIVITY press...");
    // Loan edit: Set up button for changing IMU sensitivity
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
    loraModule = new LoRaRadioBoards(USER_ADDRESS);
#endif
#endif
    loraModule->begin();
    Serial.println("LoRa module initialized");
    
    // Initialize screen with default values to make it light up immediately
    batteryPercentage = pmu->getBatteryPercentage();
    screen->drawInterface(batteryPercentage, 0, false, 0, TARGET_ADDRESS, 0, 0, 1);
    lastScreenUpdate = millis(); // Initialize screen update timer
    Serial.println("Screen initialized with default display");
}

// Time slot management for environment messages to avoid collision
int windSpeed = 0;
double latitude = 0, longitude = 0;
unsigned long previousMillis = 0;
const unsigned long interval = 10000; // Update GPS and battery status every 10s
int windDirection = 0;
int currentIMUSensitivity = 0; // Default IMU sensitivity
unsigned long lastMotionTime = 0; // For motion debouncing
const unsigned long motionDebounceDelay = 2000; // 2 seconds debounce

// Time slot management for environment messages to avoid collision
unsigned long lastEnvironmentAttempt = 0;
const unsigned long ENVIRONMENT_CHECK_INTERVAL = 250; // Check every 250ms (EVEN FASTER!)
const unsigned long TIME_SLOT_DURATION = 1000; // Each target gets 1-second slot (10s/10 targets)
const unsigned long TIME_SLOT_OFFSET = (TARGET_ID - 1) * TIME_SLOT_DURATION; // Offset based on TARGET_ID

// Fast update flags for immediate transmission when values change
bool forceEnvironmentUpdate = false;
int lastSentIMUSensitivity = -1;
int lastSentWindSpeed = -1;
void loop()
{
    // Feed watchdog to prevent timeout
    yield();
    
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval)
    {
        previousMillis = currentMillis; // Cập nhật thời gian lần chạy cuối

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
    // =========== TARGET DEVICE MAIN LOGIC ===========
    // PRIORITY SYSTEM:
    // 1. ALERT messages: Sent immediately when motion detected (highest priority)
    // 2. ENVIRONMENT messages: Sent every 10s with time slot management (SUPER FAST!)
    
    // HIGH PRIORITY: Motion detection and immediate alert
    if (motion->detectMotion())
    {
        unsigned long currentTime = millis();
        // Debounce: only send alert if enough time has passed since last motion
        if (currentTime - lastMotionTime > motionDebounceDelay) {
            lastMotionTime = currentTime;
            
#ifndef TTGO_T_BEAM_V1_1
            interruptFlag = false;
            uint8_t status = qmi.getStatusRegister();
            Serial.printf("status:0x%X BIN:", status);
            Serial.println(status, BIN);
            hitFlag = true; // Set hitFlag to true if motion is detected
            Serial.println("Motion detected! Hit flag set to true.");
            Serial.printf("🚨 PROTOBUF ALERT! TARGET %s sending alert to %s\n", TARGET_ADDRESS, USER_ADDRESS);
            
            if (loraModule->sendAlert("Hit!!!", USER_ADDRESS)) {
                Serial.println("✅ Alert sent successfully via protobuf");
            } else {
                Serial.println("❌ Failed to send alert");
            }
            
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
            hitDisplayTime = millis(); // Record time when hit was detected
            Serial.println("Motion detected! Hit flag set to true.");
            Serial.printf("🚨 PROTOBUF ALERT! TARGET %s sending alert to %s\n", TARGET_ADDRESS, USER_ADDRESS);
            
            if (loraModule->sendAlert("Hit!!!", USER_ADDRESS)) {
                Serial.println("Alert sent successfully via protobuf");
            } else {
                Serial.println("Failed to send alert");
            }
            
            externalNotification->notify();
#endif
        } else {
            Serial.printf("Motion detected but debounced (last: %lu ms ago)\n", currentTime - lastMotionTime);
        }
    }
    
    // Check if hit display duration has expired
    if (hitFlag && (millis() - hitDisplayTime > HIT_DISPLAY_DURATION)) {
        hitFlag = false;
        Serial.println("Hit display timer expired, clearing hit flag");
    }
    windDirection = environment->getWindDirection();
    int newWindSpeed = environment->getWindSpeed();
    
    // Check for significant wind speed change to trigger immediate update
    if (abs(newWindSpeed - windSpeed) >= 5) { // Threshold: 5 rpm change
        forceEnvironmentUpdate = true;
        Serial.printf("🌪️ Wind speed changed significantly: %d -> %d, forcing update!\n", windSpeed, newWindSpeed);
    }
    
    windSpeed = newWindSpeed;
    Serial.print("Wind Direction: ");
    Serial.println(windDirection);
    Serial.print("Wind Speed: ");
    Serial.print(windSpeed);
    Serial.println(" rpm");

    if (changeSensitivityFlag && motion != nullptr) {
        changeSensitivityFlag = false;
        Serial.println("BUTTON_IMU_SENSITIVITY pressed! Processing sensitivity change...");
        motion->increaseSensitivity();
        currentIMUSensitivity = motion->getSensitivity(); // Update current sensitivity
        Serial.print("New IMU Sensitivity: ");
        Serial.println(currentIMUSensitivity);
        
        // Force immediate environment update when sensitivity changes
        forceEnvironmentUpdate = true;
        Serial.println("🚀 Forcing immediate environment update for IMU sensitivity change!");
    }
    
    // Update current IMU sensitivity from motion sensor
    if (motion != nullptr) {
        currentIMUSensitivity = motion->getSensitivity();
    }
    
    Serial.print("Current IMU Sensitivity: ");
    Serial.println(currentIMUSensitivity);
    // Loan edit: Get the current IMU sensitivity

    // Update screen regularly - every 500ms
    unsigned long currentScreenTime = millis();
    if (currentScreenTime - lastScreenUpdate >= SCREEN_UPDATE_INTERVAL) {
        lastScreenUpdate = currentScreenTime;
        screen->drawInterface(batteryPercentage, 0, hitFlag, 0, TARGET_ADDRESS, windDirection, windSpeed, currentIMUSensitivity);
    }

    // Environment data transmission with time slot management and immediate updates
    unsigned long currentMillisEnv = millis();
    bool shouldSendEnvironment = false;
    
    // Check for immediate update conditions
    if (forceEnvironmentUpdate || 
        (currentIMUSensitivity != lastSentIMUSensitivity) ||
        (windSpeed != lastSentWindSpeed)) {
        shouldSendEnvironment = true;
        forceEnvironmentUpdate = false;
        Serial.println("🚀 [TARGET] Immediate environment update triggered!");
    }
    // Check for regular timed update
    else if (currentMillisEnv - lastEnvironmentAttempt >= ENVIRONMENT_CHECK_INTERVAL) {
        lastEnvironmentAttempt = currentMillisEnv;
        
        // Calculate if we're in our assigned time slot
        unsigned long cycleTime = currentMillisEnv % 10000; // 10-second cycle (SUPER FAST!)
        unsigned long slotStart = TIME_SLOT_OFFSET;
        unsigned long slotEnd = (slotStart + TIME_SLOT_DURATION) % 10000;
        
        bool inTimeSlot = false;
        if (slotStart < slotEnd) {
            // Normal case: slot doesn't wrap around
            inTimeSlot = (cycleTime >= slotStart && cycleTime < slotEnd);
        } else {
            // Wrap around case: slot crosses cycle boundary
            inTimeSlot = (cycleTime >= slotStart || cycleTime < slotEnd);
        }
        
        shouldSendEnvironment = inTimeSlot;
    }
    
    if (shouldSendEnvironment) {
        // Try to send environment data (internal rate limiting still applies)
        if (loraModule->sendEnvironment(windSpeed, windMode, windDirection, current_latitude, current_longtitude, currentIMUSensitivity, USER_ADDRESS)) {
            // Update last sent values
            lastSentIMUSensitivity = currentIMUSensitivity;
            lastSentWindSpeed = windSpeed;
            
            Serial.printf("📡 PROTOBUF Environment data sent from TARGET %s to %s!\n", 
                         TARGET_ADDRESS, USER_ADDRESS);
            Serial.printf("Data: Wind=%d°@%d (Mode:%d), GPS=(%.6f,%.6f), IMU=%d\n", 
                         windDirection, windSpeed, windMode, current_latitude, current_longtitude, currentIMUSensitivity);
        }
    }
    
    // Removed time slot debugging - simplified system
    
#else
    // USER-SIDE LOGIC: This section should not run on target devices
    // but keeping for reference and backward compatibility
    
    String receivedMessage;
    hitFlag = false;
    
    Serial.println("WARNING: User-side code running on target device!");
    Serial.println("This target device should not be receiving messages!");
    
    // Try to receive string message (for backward compatibility)
    if (loraModule->receiveMessage(receivedMessage))
    {
        String senderAddress, payload, messageType;
        loraModule->parseMessage(receivedMessage, senderAddress, messageType, payload);
        Serial.print("Received message from: ");
        Serial.println(senderAddress);
        
        if (senderAddress == TARGET_ADDRESS) {
            Serial.println("Ignoring message from self");
            return;
        }
        
        Serial.println(messageType);
        if (messageType == "ALERT")
        {
            hitFlag = true;
            Serial.println("Target was hit!!");
            screen->drawHitNotification(senderAddress.c_str());
            hitFlag = false;
            delay(1000);
        }
        
        // Parse environment payload if applicable
        int receivedWindSpeed, receivedWindMode, receivedWindDirection, receivedIMU;
        double receivedLat, receivedLon;
        
        if (loraModule->parseEnvironmentPayload(payload, receivedWindSpeed, receivedWindMode, 
                                              receivedWindDirection, receivedLat, receivedLon, receivedIMU)) {
            Serial.printf("Environment: Wind=%d°@%d (Mode:%d), GPS=(%.6f,%.6f), IMU=%d\n", 
                         receivedWindDirection, receivedWindSpeed, receivedWindMode, 
                         receivedLat, receivedLon, receivedIMU);
        }

        // Retrieve and print signal strength
        float rssi, snr;
        loraModule->getSignalStrength(rssi, snr);
        Serial.printf("Signal: RSSI=%.1fdBm, SNR=%.1fdB\n", rssi, snr);

        double distance = GPS::calculateDistance(current_latitude, current_longtitude, receivedLat, receivedLon);
        
        // Update the screen with the received data
        screen->drawInterface(batteryPercentage, rssi, hitFlag, distance, senderAddress.c_str(), 
                             receivedWindSpeed, receivedWindMode, receivedIMU);
    }
#endif
#ifdef TARGET
    delay(100); // Increased delay to reduce processing load and prevent watchdog timeout
    yield(); // Allow other tasks to run
#endif
    // delay(500); // Small delay to reduce processing load

        // Main loop for dynamic updates
}
