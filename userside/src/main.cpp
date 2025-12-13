/*
 * T-Beam User-Side Device - OPTIMIZED VERSION
 * Multi-target monitoring system with LoRa communication
 * Opti// GLOBAL OBJECTS - Direct types for performance
SystemState g_state;
TargetData targetCache[10];  // Cache data for 10 targets (indexed by channel)

Screen *screen = nullptr;
LoRaRadioBoards *loraModule = nullptr;  // Direct type - no casting needed
ExternalNotification *externalNotification = nullptr;
PowerManagement *pmu = nullptr;
GPSModule *gps = nullptr;
BarometricSensor* baroSensor = nullptr;r maximum responsiveness to HIT alerts
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "Screen.h"
#include "Motion.h"
#include "Telemetry.h"
#include "Environment.h"
#include "Location.h"
#include "Power.h"
#include "ExternalNotification.h"
#include "BarometricPressure.h"

// TARGET CONFIGURATION - Optimized constants
constexpr const char *TARGET_ADDRESSES[10] = {
    "1", "2", "3", "4", "5", "6", "7", "8", "9", "10"};
constexpr const char *USER_ADDRESS = "User";

// PERFORMANCE OPTIMIZATION CONSTANTS
constexpr uint32_t BATTERY_UPDATE_INTERVAL = 2000; // 2s
constexpr uint32_t BUTTON_DEBOUNCE_TIME = 100;     // 100ms
constexpr uint32_t ALERT_DISPLAY_TIME = 2000;      // 2s
constexpr uint32_t SCREEN_UPDATE_INTERVAL = 1000;  // 1s for normal display
constexpr uint32_t HIT_TIMEOUT = 5000;             // 5s timeout for hit flag
constexpr uint8_t LORA_BUFFER_SIZE = 128;
constexpr uint8_t MAIN_LOOP_DELAY = 5;        // Optimized for hit responsiveness
constexpr uint32_t GPS_READ_INTERVAL = 1000;  // GPS update interval
constexpr uint32_t BARO_READ_INTERVAL = 1000; // Barometric sensor interval

// HARDWARE PIN DEFINITIONS
#define GPS_RX_PIN 34
#define GPS_TX_PIN 12
#define BUZZER_PORT 32
#define CHANNEL_A 15
#define CHANNEL_B 35
#define button_channel 38 // Button channel for change slave 38/25
#define PMU_SCL 22
#define PMU_SDA 21
#define SPI_MOSI (35)
#define SPI_SCK (36)
#define SPI_MISO (37)
#define IMU_CS (34)
#define IMU_INT (33)
#define SPI_FREQ 4000000
float currentRssi = 0.0f, currentSnr = 0.0f;

// TARGET DATA CACHE STRUCTURE
struct TargetData {
    // Environment data
    int windSpeed = 0;
    int windDirection = 0;
    int windMode = 0;
    int imuSensitivity = 0;
    
    // GPS and distance
    double targetLatitude = 0.0;
    double targetLongitude = 0.0;
    int distanceToTarget = 0;
    
    // Signal info
    float lastRssi = -50.0f;
    
    // Data validity
    bool hasData = false;
    uint32_t lastUpdateTime = 0;
};

// OPTIMIZED SYSTEM STATE STRUCTURE
struct SystemState
{
    // Channel management
    uint8_t currentChannel = 0;
    String currentTargetAddress = TARGET_ADDRESSES[0];

    // Hit/Alert state - HIGHEST PRIORITY
    bool hitFlag = false;
    String hitTargetAddress = "";
    uint32_t lastHitTime = 0;

    // Button handling
    bool buttonPressed = false;
    uint32_t lastButtonTime = 0;

    // Timing control
    uint32_t lastBatteryUpdate = 0;
    uint32_t lastGpsRead = 0;
    uint32_t lastBaroRead = 0;
    uint32_t lastScreenUpdate = 0;

    // Sensor data (local)
    float batteryPercentage = 0.0f;
    float batteryVoltage = 0.0f;
    double currentLatitude = 0.0;
    double currentLongitude = 0.0;
    float temperatureF = 0.0f;
    float pressureInHg = 0.0f;

    // Current display data (copied from selected target cache)
    int windSpeed = 0;
    int windDirection = 0;
    int windMode = 0;
    int imuSensitivity = 0;
    
    // Target GPS and distance calculation
    double targetLatitude = 0.0;
    double targetLongitude = 0.0;
    int distanceToTarget = 0;  // in yards
    float lastRssi = -50.0f;   // Last received RSSI value

    // System flags
    bool systemInitialized = false;
};

// GLOBAL OBJECTS - Direct types for performance
SystemState g_state;
TargetData targetCache[10];  // Cache data for 10 targets

Screen *screen = nullptr;
LoRaRadioBoards *loraModule = nullptr; // Direct type - no casting needed
ExternalNotification *externalNotification = nullptr;
PowerManagement *pmu = nullptr;
GPSModule *gps = nullptr;
BarometricSensor *baroSensor = nullptr;

// SPI Configuration
SPIClass LoRaSPI(HSPI);
SPISettings spiSettings(SPI_FREQ, MSBFIRST, SPI_MODE0);

// Interrupt variables
volatile bool buttonPressed = false;

// Helper functions for target data management
void saveTargetData(uint8_t targetIndex, int windSpeed, int windDirection, int windMode, 
                   int imuSensitivity, double lat, double lon, int distance, float rssi) {
    if (targetIndex >= 10) return;
    
    targetCache[targetIndex].windSpeed = windSpeed;
    targetCache[targetIndex].windDirection = windDirection;
    targetCache[targetIndex].windMode = windMode;
    targetCache[targetIndex].imuSensitivity = imuSensitivity;
    targetCache[targetIndex].targetLatitude = lat;
    targetCache[targetIndex].targetLongitude = lon;
    targetCache[targetIndex].distanceToTarget = distance;
    targetCache[targetIndex].lastRssi = rssi;
    targetCache[targetIndex].hasData = true;
    targetCache[targetIndex].lastUpdateTime = millis();
}

void loadTargetData(uint8_t targetIndex) {
    if (targetIndex >= 10) return;
    
    if (targetCache[targetIndex].hasData) {
        // Load cached data to current display state
        g_state.windSpeed = targetCache[targetIndex].windSpeed;
        g_state.windDirection = targetCache[targetIndex].windDirection;
        g_state.windMode = targetCache[targetIndex].windMode;
        g_state.imuSensitivity = targetCache[targetIndex].imuSensitivity;
        g_state.targetLatitude = targetCache[targetIndex].targetLatitude;
        g_state.targetLongitude = targetCache[targetIndex].targetLongitude;
        g_state.distanceToTarget = targetCache[targetIndex].distanceToTarget;
        g_state.lastRssi = targetCache[targetIndex].lastRssi;
        
        Serial.printf("📦 Loaded cached data for target %d: Wind=%d°@%dmph, Distance=%d yards\n",
                     targetIndex + 1, g_state.windDirection, g_state.windSpeed, g_state.distanceToTarget);
    } else {
        // No cached data - reset to defaults
        g_state.windSpeed = 0;
        g_state.windDirection = 0;
        g_state.windMode = 0;
        g_state.imuSensitivity = 0;
        g_state.targetLatitude = 0.0;
        g_state.targetLongitude = 0.0;
        g_state.distanceToTarget = 0;
        g_state.lastRssi = -50.0f;
        
        Serial.printf("📦 No cached data for target %d - using defaults\n", targetIndex + 1);
    }
}

// OPTIMIZED INTERRUPT HANDLER
void IRAM_ATTR handleSensitivityButton()
{
    buttonPressed = true; // Simple flag setting - no complex operations in ISR
}

// OPTIMIZED SETUP FUNCTION
void setup()
{
    delay(1000); // Reduced startup delay
    Serial.begin(115200);
    Serial.println("T-Beam User-Side Starting (OPTIMIZED)...");

    // Button setup
    pinMode(button_channel, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(button_channel), handleSensitivityButton, FALLING);
    Serial.printf("Button initialized on pin %d\n", button_channel);

    // I2C setup
    Wire.begin(PMU_SDA, PMU_SCL);

    // Initialize critical components first
    externalNotification = new Buzzer(BUZZER_PORT);
    screen = ScreenFactory::createScreen();
    screen->begin();
    Serial.println("Screen initialized");

    // Power management
    pmu = new AXPManagement(PMU_SDA, PMU_SCL);
    if (pmu->init())
    {
        Serial.println("PMU initialized successfully");
    }
    else
    {
        Serial.println("PMU initialization failed");
    }

    // Barometric sensor
    baroSensor = new BMP085Sensor();
    if (baroSensor->init())
    {
        g_state.temperatureF = baroSensor->readTemperatureF();
        g_state.pressureInHg = baroSensor->readPressureInHg();
        Serial.printf("BMP085 initialized: %.2f°F, %.2f inHg\n",
                      g_state.temperatureF, g_state.pressureInHg);
    }
    else
    {
        Serial.println("BMP085 initialization failed");
    }

    // GPS module
    gps = new GPSModule(Serial1, GPS_RX_PIN, GPS_TX_PIN);
    gps->begin();
    Serial.println("GPS initialized");

    // LoRa module - CRITICAL for hit detection
    loraModule = new LoRaRadioBoards(USER_ADDRESS);
    loraModule->begin();
    Serial.println("LoRa initialized - Ready for hit detection");

    g_state.systemInitialized = true;
    Serial.println("System initialization complete");
}

// OPTIMIZED MESSAGE PROCESSING FUNCTION
inline bool processLoRaMessage(uint32_t currentMillis)
{
    static uint8_t buffer[LORA_BUFFER_SIZE];
    size_t length = LORA_BUFFER_SIZE;

    if (!loraModule->receiveMessage(buffer, length))
    {
        return false; // No message received
    }

    // Parse message data
    String sourceId = "";
    int protobufWindSpeed = 0, protobufWindMode = 0, protobufWindDirection = 0, protobufIMU = 0;
    double protobufLatitude = 0.0, protobufLongitude = 0.0;

    float currentRssi, currentSnr;
    loraModule->getSignalStrength(currentRssi, currentSnr);
    
    // Store RSSI for screen display
    g_state.lastRssi = currentRssi;

    if (!loraModule->parseProtobufMessage(buffer, length, sourceId,
                                          protobufWindSpeed, protobufWindMode, protobufWindDirection,
                                          protobufLatitude, protobufLongitude, protobufIMU))
    {
        return false;
    }

    // PRIORITY CHECK: HIT ALERT (windSpeed = -1)
    if (protobufWindSpeed == -1)
    {
        // Immediate hit processing
        g_state.hitFlag = true;
        g_state.hitTargetAddress = sourceId;
        g_state.lastHitTime = currentMillis;

        // Immediate alert
        if (externalNotification)
        {
            externalNotification->notify();
        }

        // Force screen update
        screen->drawHitNotification(sourceId.c_str());
        return true;
    }

    // ENVIRONMENT DATA: Process and cache data from ALL targets
    // Find target index from sourceId
    int targetIndex = -1;
    for (int i = 0; i < 10; i++) {
        if (sourceId == TARGET_ADDRESSES[i]) {
            targetIndex = i;
            break;
        }
    }
    
    if (targetIndex >= 0) {
        Serial.printf("� Processing environment data from target %s (index %d)\n", sourceId.c_str(), targetIndex);
        Serial.printf("🔍 Raw values: Speed=%d, Dir=%d, Mode=%d, IMU=%d\n",
                      protobufWindSpeed, protobufWindDirection, protobufWindMode, protobufIMU);

        // Validate and store data in cache for this target
        int validWindSpeed = 0, validWindDirection = 0, validWindMode = 0, validIMU = 0;
        double validLat = 0.0, validLon = 0.0;
        int validDistance = 0;
        
        // Validate data ranges
        if (protobufWindSpeed >= 0 && protobufWindSpeed <= 100000) {
            validWindSpeed = protobufWindSpeed;
        }
        if (protobufWindDirection >= 0 && protobufWindDirection <= 10000000) {
            validWindDirection = protobufWindDirection;
        }
        if (protobufWindMode >= 0 && protobufWindMode <= 10) {
            validWindMode = protobufWindMode;
        }
        if (protobufIMU >= 0 && protobufIMU <= 100) {
            validIMU = protobufIMU;
        }
        if (protobufLatitude != 0.0 && protobufLongitude != 0.0) {
            validLat = protobufLatitude;
            validLon = protobufLongitude;
            
            // Calculate distance if user GPS is valid
            if (g_state.currentLatitude != 0.0 && g_state.currentLongitude != 0.0) {
                double distanceYards = GPSModule::calculateDistance(
                    g_state.currentLatitude, g_state.currentLongitude,
                    validLat, validLon
                );
                if (distanceYards > 0) {
                    validDistance = (int)distanceYards;
                }
            }
        }
        
        // Save data to target cache
        saveTargetData(targetIndex, validWindSpeed, validWindDirection, validWindMode,
                      validIMU, validLat, validLon, validDistance, g_state.lastRssi);
        
        Serial.printf("� Cached data for target %s: Wind=%d°@%dmph, Distance=%d yards\n",
                     sourceId.c_str(), validWindDirection, validWindSpeed, validDistance);
        
        // If this is the currently displayed target, update display immediately
        if (targetIndex == g_state.currentChannel) {
            loadTargetData(targetIndex);  // Load cached data into g_state
            g_state.lastScreenUpdate = 0; // Force immediate screen update
            Serial.printf("🖥️ Updated display for current target %s\n", sourceId.c_str());
        }
    } else {
        Serial.printf("❌ Unknown target ID: %s\n", sourceId.c_str());
    }

    return true;
}

// OPTIMIZED BUTTON HANDLING
inline void handleButtonPress(uint32_t currentMillis)
{
    if (!buttonPressed)
        return;

    if (currentMillis - g_state.lastButtonTime > BUTTON_DEBOUNCE_TIME)
    {
        g_state.currentChannel = (g_state.currentChannel + 1) % 10;
        g_state.currentTargetAddress = TARGET_ADDRESSES[g_state.currentChannel];
        g_state.lastButtonTime = currentMillis;

        Serial.printf("🔄 Switched to channel %d (Target: %s)\n",
                      g_state.currentChannel, g_state.currentTargetAddress.c_str());

        // Clear screen when switching channels
        screen->clear();
        
        // Load cached data for the new target (or defaults if no cached data)
        loadTargetData(g_state.currentChannel);
        
        // Force immediate screen update to show new channel with cached/default values
        g_state.lastScreenUpdate = 0;
    }

    buttonPressed = false;
}

// OPTIMIZED SENSOR UPDATES
inline void updateSensors(uint32_t currentMillis)
{
    // Battery update (every 2s)
    if (currentMillis - g_state.lastBatteryUpdate >= BATTERY_UPDATE_INTERVAL)
    {
        float tempBatteryPercentage = pmu->getBatteryPercentage();
        float tempBatteryVoltage = pmu->getBatteryVoltage();

        // Validate readings
        if (tempBatteryPercentage >= 0.0 && tempBatteryPercentage <= 100.0)
        {
            g_state.batteryPercentage = tempBatteryPercentage;
        }
        if (tempBatteryVoltage >= 0.0 && tempBatteryVoltage <= 5.0)
        {
            g_state.batteryVoltage = tempBatteryVoltage;
        }

        g_state.lastBatteryUpdate = currentMillis;
    }

    // GPS update (every 1s)
    if (currentMillis - g_state.lastGpsRead >= GPS_READ_INTERVAL)
    {
        if (gps->getCoordinates(g_state.currentLatitude, g_state.currentLongitude))
        {
            // Validate GPS coordinates
            if (g_state.currentLatitude >= -90.0 && g_state.currentLatitude <= 90.0 &&
                g_state.currentLongitude >= -180.0 && g_state.currentLongitude <= 180.0)
            {
                // Valid GPS data - no action needed, just stored
            }
            else
            {
                g_state.currentLatitude = 0.0;
                g_state.currentLongitude = 0.0;
            }
        }
        g_state.lastGpsRead = currentMillis;
    }

    // Barometric sensor update (every 1s)
    if (currentMillis - g_state.lastBaroRead >= BARO_READ_INTERVAL && baroSensor)
    {
        float tempF = baroSensor->readTemperatureF();
        float pressInHg = baroSensor->readPressureInHg();

        if (tempF > -50.0 && tempF < 150.0)
        {
            g_state.temperatureF = tempF;
        }
        if (pressInHg > 20.0 && pressInHg < 35.0)
        {
            g_state.pressureInHg = pressInHg;
        }

        g_state.lastBaroRead = currentMillis;
    }
}

// OPTIMIZED SCREEN UPDATE
inline void updateScreen(uint32_t currentMillis)
{
    // Check for hit timeout
    if (g_state.hitFlag && (currentMillis - g_state.lastHitTime > HIT_TIMEOUT))
    {
        g_state.hitFlag = false;
        g_state.hitTargetAddress = "";
        Serial.println("Hit timeout - clearing alert");
        // Force immediate normal screen update after clearing hit
        g_state.lastScreenUpdate = 0;
    }

    // Priority display: Hit alert (but still allow periodic normal updates)
    if (g_state.hitFlag)
    {
        // Show hit notification for first 2 seconds, then alternate with normal display
        if ((currentMillis - g_state.lastHitTime) < ALERT_DISPLAY_TIME)
        {
            if (currentMillis - g_state.lastScreenUpdate >= 100)
            { // Fast update for alerts
                screen->drawHitNotification(g_state.hitTargetAddress.c_str());
                g_state.lastScreenUpdate = currentMillis;
            }
            return;
        }
        // After 2 seconds, show normal display but keep hit flag for buzzer/logging
    }

    // Normal display update (every 1s) OR forced update
    if (currentMillis - g_state.lastScreenUpdate >= SCREEN_UPDATE_INTERVAL || g_state.lastScreenUpdate == 0)
    {
        // Use the correct drawInterface method signature
        Serial.printf("🖥️ SCREEN UPDATE: Wind=%d°@%dmph, IMU=%d, Temp=%.1f°F, Batt=%d%%\n",
                      g_state.windDirection, g_state.windSpeed, g_state.imuSensitivity,
                      g_state.temperatureF, (int)g_state.batteryPercentage);

        screen->drawInterface(
            (int)g_state.batteryPercentage,       // batteryPercent
            (int)g_state.lastRssi,                // signalStrength (stored RSSI value)
            g_state.hitFlag,                      // hitFlag
            g_state.distanceToTarget,             // distance (in yards)
            g_state.currentTargetAddress.c_str(), // targetAddress
            g_state.windDirection,                // windDirection
            g_state.windSpeed,                    // windSpeed
            g_state.temperatureF,                 // temperatureF
            g_state.pressureInHg,                 // pressureInHg
            g_state.imuSensitivity                // imuSensitivity
        );
        g_state.lastScreenUpdate = currentMillis;
    }
}

// OPTIMIZED MAIN LOOP - PRIORITIZED FOR HIT DETECTION
void loop()
{
    if (!g_state.systemInitialized)
    {
        delay(100);
        return;
    }

    uint32_t currentMillis = millis();

    // PRIORITY #1: LoRa message processing (Hit alerts have highest priority)
    processLoRaMessage(currentMillis);

    // PRIORITY #2: Button handling (user interaction)
    handleButtonPress(currentMillis);

    // PRIORITY #3: Sensor updates (background tasks)
    updateSensors(currentMillis);

    // PRIORITY #4: Screen updates (visual feedback)
    updateScreen(currentMillis);

    // Minimal delay for maximum responsiveness
    delay(MAIN_LOOP_DELAY);
}
