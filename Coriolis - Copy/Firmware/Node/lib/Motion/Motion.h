#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <math.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "SensorQMI8658.hpp"
#include "ICM_20948.h"

// QMI8658 I2C registers
#define QMI8658_I2C_ADDR   0x6B
#define QMI8658_WHO_AM_I   0x00
#define QMI8658_REVISION   0x01
#define QMI8658_CTRL1      0x02
#define QMI8658_CTRL2      0x03
#define QMI8658_CTRL7      0x08
#define QMI8658_STATUS0    0x2D
#define QMI8658_AX_L       0x35

// ======================================================
// Abstract Motion class
// ======================================================
class Motion
{
public:
    virtual ~Motion() {}
    virtual bool begin() = 0;
    virtual bool detectMotion() = 0;
    virtual void displayAccelerometer() = 0;
    virtual void increaseSensitivity() = 0;
    virtual int  getSensitivity() const = 0;
    virtual void calibrate() {}
    virtual bool isConnected() const { return true; }
};

// ======================================================
// QMI8658 I2C implementation - STANDARD DEVIATION detection
// ======================================================
class QMI8658_I2C_Motion : public Motion
{
private:
    int sensitivity = 60; // 20..100 (HIGHER number = LESS sensitive)
    bool initialized = false;
    bool calibrated = false;

    // Circular buffer for variance calculation
    static const int BUFFER_SIZE = 15;
    float magBuffer[BUFFER_SIZE];
    int bufferIndex = 0;
    bool bufferFilled = false;
    
    // Detection state
    unsigned long lastMotionTime = 0;
    unsigned long motionCooldown = 1000;  // 1 second - prevents any repeat detections

    // I2C error tracking
    int consecutiveI2CErrors = 0;
    unsigned long lastErrorLog = 0;
    const int MAX_CONSECUTIVE_ERRORS = 3;
    const unsigned long ERROR_LOG_INTERVAL = 5000;

public:
    bool begin() override
    {
        Serial.println("[QMI8658-I2C] ========== Starting I2C Init ==========");
        Serial.printf("[QMI8658-I2C] I2C Address: 0x%02X\n", QMI8658_I2C_ADDR);

        Wire.beginTransmission(QMI8658_I2C_ADDR);
        uint8_t error = Wire.endTransmission();

        if (error != 0) {
            Serial.println("[QMI8658-I2C] Device not responding");
            return false;
        }

        uint8_t whoami = readReg(QMI8658_WHO_AM_I);
        Serial.printf("[QMI8658-I2C] WHO_AM_I = 0x%02X (expect 0x05)\n", whoami);

        if (whoami != 0x05) {
            Serial.println("[QMI8658-I2C] Wrong chip ID");
            return false;
        }

        // Disable sensors, configure accel, then enable
        writeReg(QMI8658_CTRL7, 0x00);
        delay(10);

        // ±4g, 500Hz
        writeReg(QMI8658_CTRL2, 0x17);
        delay(10);

        writeReg(QMI8658_CTRL7, 0x01);
        delay(50);

        // Reset buffer
        for (int i = 0; i < BUFFER_SIZE; i++) {
            magBuffer[i] = 0;
        }
        bufferIndex = 0;
        bufferFilled = false;
        calibrated = false;
        initialized = true;

        Serial.println("[QMI8658-I2C] ========== Init Complete ==========\n");
        return true;
    }

    void calibrate() override
    {
        if (!initialized) return;

        Serial.println("[QMI8658-I2C] Calibrating (filling buffer)... keep device still");
        
        // Fill buffer with initial readings
        for (int i = 0; i < BUFFER_SIZE; i++) {
            float ax, ay, az;
            if (readAccel(ax, ay, az)) {
                float mag = sqrtf(ax*ax + ay*ay + az*az);
                magBuffer[i] = mag;
            }
            delay(10);
        }
        
        bufferIndex = 0;
        bufferFilled = true;
        calibrated = true;
        
        Serial.println("[QMI8658-I2C] Calibration complete");
        Serial.println("[QMI8658-I2C] Using STANDARD DEVIATION impact detection");
    }

    bool detectMotion() override
    {
        if (!initialized || !calibrated) return false;

        unsigned long now = millis();
        if (now - lastMotionTime < motionCooldown) return false;

        float ax, ay, az;
        if (!readAccel(ax, ay, az)) return false;

        // Calculate magnitude
        float mag = sqrtf(ax*ax + ay*ay + az*az);
        
        // Add to circular buffer
        magBuffer[bufferIndex] = mag;
        bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
        if (bufferIndex == 0) bufferFilled = true;
        
        if (!bufferFilled) return false; // Need full buffer first
        
        // Calculate mean
        float sum = 0;
        for (int i = 0; i < BUFFER_SIZE; i++) {
            sum += magBuffer[i];
        }
        float mean = sum / BUFFER_SIZE;
        
        // Calculate standard deviation
        float variance = 0;
        for (int i = 0; i < BUFFER_SIZE; i++) {
            float diff = magBuffer[i] - mean;
            variance += diff * diff;
        }
        variance /= BUFFER_SIZE;
        float stdDev = sqrtf(variance);
        
        // REVERSED sensitivity mapping (lower number = more sensitive)
        // sensitivity 20: 0.200g std dev (MOST sensitive - detects lighter hits)
        // sensitivity 60: 0.500g std dev (default middle)
        // sensitivity 100: 0.800g std dev (LEAST sensitive - only hard hits)
        float minThreshold = 0.200f;  // At sensitivity 20 (most sensitive)
        float maxThreshold = 0.800f;  // At sensitivity 100 (least sensitive)
        float threshold = minThreshold + (maxThreshold - minThreshold) * (sensitivity - 20) / 80.0f;
        
        // Detect if std dev exceeds threshold
        if (stdDev > threshold) {
            lastMotionTime = now;
            
            // CRITICAL: Clear the buffer to prevent continuous triggering
            // Fill with current stable magnitude to reset variance to near-zero
            for (int i = 0; i < BUFFER_SIZE; i++) {
                magBuffer[i] = mag;
            }
            
            Serial.printf("[QMI8658-I2C] Impact detected: stdDev=%.4f (threshold=%.4f, sens=%d)\n",
                         stdDev, threshold, sensitivity);
            return true;
        }
        
        return false;
    }

    void displayAccelerometer() override
    {
        if (!initialized) return;
        float x, y, z;
        if (readAccel(x, y, z)) {
            float mag = sqrtf(x*x + y*y + z*z);
            Serial.printf("[QMI8658-I2C] Accel: X=%.3f Y=%.3f Z=%.3f g | |a|=%.3f\n", x, y, z, mag);
        }
    }

    void increaseSensitivity() override
    {
        sensitivity += 10;
        if (sensitivity > 100) sensitivity = 20;
        Serial.printf("[QMI8658-I2C] Sensitivity: %d\n", sensitivity);
    }

    int getSensitivity() const override { return sensitivity; }

    bool isConnected() const override
    {
        if (!initialized) return false;
        if (consecutiveI2CErrors >= MAX_CONSECUTIVE_ERRORS) return false;

        Wire.beginTransmission(QMI8658_I2C_ADDR);
        uint8_t error = Wire.endTransmission();

        if (error == 0) {
            const_cast<QMI8658_I2C_Motion*>(this)->consecutiveI2CErrors = 0;
            return true;
        }
        return false;
    }

private:
    uint8_t readReg(uint8_t reg)
    {
        Wire.beginTransmission(QMI8658_I2C_ADDR);
        Wire.write(reg);
        Wire.endTransmission(false);
        Wire.requestFrom(QMI8658_I2C_ADDR, (uint8_t)1);
        if (Wire.available()) return Wire.read();
        return 0xFF;
    }

    void writeReg(uint8_t reg, uint8_t value)
    {
        Wire.beginTransmission(QMI8658_I2C_ADDR);
        Wire.write(reg);
        Wire.write(value);
        Wire.endTransmission();
    }

    bool readAccel(float &x, float &y, float &z)
    {
        if (consecutiveI2CErrors >= MAX_CONSECUTIVE_ERRORS) {
            unsigned long now = millis();
            if (now - lastErrorLog > ERROR_LOG_INTERVAL) {
                Serial.println("[QMI8658-I2C] Skipping reads due to I2C errors");
                lastErrorLog = now;
            }
            return false;
        }

        // Single register read fallback (most reliable)
        auto read8 = [&](uint8_t reg, uint8_t &out)->bool {
            Wire.beginTransmission(QMI8658_I2C_ADDR);
            Wire.write(reg);
            if (Wire.endTransmission(false) != 0) return false;
            delayMicroseconds(10);
            Wire.requestFrom(QMI8658_I2C_ADDR, (uint8_t)1);
            if (Wire.available()) { out = Wire.read(); return true; }
            return false;
        };

        uint8_t b0, b1, b2, b3, b4, b5;
        bool ok = true;
        ok &= read8(QMI8658_AX_L, b0);
        ok &= read8(QMI8658_AX_L + 1, b1);
        ok &= read8(QMI8658_AX_L + 2, b2);
        ok &= read8(QMI8658_AX_L + 3, b3);
        ok &= read8(QMI8658_AX_L + 4, b4);
        ok &= read8(QMI8658_AX_L + 5, b5);

        if (!ok) {
            consecutiveI2CErrors++;
            return false;
        }

        int16_t ax = (int16_t)((b1 << 8) | b0);
        int16_t ay = (int16_t)((b3 << 8) | b2);
        int16_t az = (int16_t)((b5 << 8) | b4);

        x = ax / 8192.0f;
        y = ay / 8192.0f;
        z = az / 8192.0f;
        
        float mag = sqrtf(x*x + y*y + z*z);

        if (!isfinite(x) || !isfinite(y) || !isfinite(z) || mag > 16.0f || mag < 0.2f) {
            consecutiveI2CErrors++;
            return false;
        }

        consecutiveI2CErrors = 0;
        return true;
    }
};

// ======================================================
// MPU6050 implementation (UNCHANGED)
// ======================================================
class MPU6050Motion : public Motion
{
private:
    Adafruit_MPU6050 mpu;
    int sensitivity = 20;

public:
    bool begin() override
    {
        Serial.println("[MPU6050] Attempting detection...");
        if (!mpu.begin())
        {
            Serial.println("[MPU6050] Not detected");
            return false;
        }

        Serial.println("[MPU6050] Detected successfully");

        mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
        mpu.setMotionDetectionThreshold(20);
        mpu.setMotionDetectionDuration(20);
        mpu.setInterruptPinLatch(true);
        mpu.setInterruptPinPolarity(true);

        return true;
    }

    bool detectMotion() override
    {
        return mpu.getMotionInterruptStatus();
    }

    void displayAccelerometer() override {}

    void increaseSensitivity() override
    {
        sensitivity += 10;
        if (sensitivity > 100) sensitivity = 20;
        uint8_t threshold = map(sensitivity, 20, 100, 50, 10);
        mpu.setMotionDetectionThreshold(threshold);
        Serial.printf("[MPU6050] Sensitivity: %d\n", sensitivity);
    }

    int getSensitivity() const override { return sensitivity; }

    bool isConnected() const override
    {
        sensors_event_t a, g, temp;
        if (!const_cast<Adafruit_MPU6050&>(mpu).getEvent(&a, &g, &temp)) return false;
        return isfinite(a.acceleration.x) && isfinite(a.acceleration.y) && isfinite(a.acceleration.z);
    }
};

// ======================================================
// ICM-20948 implementation (UNCHANGED)
// ======================================================
class ICM20948Motion : public Motion
{
private:
    ICM_20948_I2C icm;
    int sensitivity = 20;
    float biasX = 0, biasY = 0, biasZ = 0;
    bool calibrated = false;
    float lastAx = 0, lastAy = 0, lastAz = 0;
    float filteredDelta = 0;
    unsigned long candidateTime = 0;
    bool candidate = false;
    float smoothingAlpha = 0.4f;
    unsigned long confirmWindowMs = 120;
    float fastThresholdLow = 300.0f;
    float fastThresholdHigh = 900.0f;
    float confirmThresholdLow = 600.0f;
    float confirmThresholdHigh = 1200.0f;
    int i2cErrorCount = 0;
    const int I2C_ERROR_THRESHOLD = 3;

public:
    bool begin() override
    {
        Serial.println("[ICM-20948] Attempting detection...");
        icm.begin(Wire, 0x68);

        if (icm.status != ICM_20948_Stat_Ok)
        {
            Serial.println("[ICM-20948] Not detected");
            return false;
        }

        Serial.println("[ICM-20948] Detected successfully");

        icm.setSampleMode(ICM_20948_Internal_Acc,
                          ICM_20948_Sample_Mode_Continuous);

        ICM_20948_fss_t fss;
        fss.a = gpm4;
        icm.setFullScale(ICM_20948_Internal_Acc, fss);

        calibrated = false;
        biasX = biasY = biasZ = 0;
        lastAx = lastAy = lastAz = 0;
        filteredDelta = 0;
        candidate = false;
        candidateTime = 0;
        i2cErrorCount = 0;

        return true;
    }

    void calibrate() override
    {
        const int N = 50;
        float sumX = 0, sumY = 0, sumZ = 0;
        Serial.println("[ICM-20948] Calibrating...");
        
        for (int i = 0; i < N; ++i)
        {
            icm.getAGMT();
            sumX += icm.accX();
            sumY += icm.accY();
            sumZ += icm.accZ();
            delay(5);
        }
        biasX = sumX / N;
        biasY = sumY / N;
        biasZ = sumZ / N;
        lastAx = biasX; lastAy = biasY; lastAz = biasZ;
        calibrated = true;
        i2cErrorCount = 0;
        Serial.println("[ICM-20948] Calibration complete");
    }

    bool detectMotion() override
    {
        if (i2cErrorCount >= I2C_ERROR_THRESHOLD) {
            return false;
        }

        icm.getAGMT();

        float ax = icm.accX();
        float ay = icm.accY();
        float az = icm.accZ();

        if (!isfinite(ax) || !isfinite(ay) || !isfinite(az)) {
            i2cErrorCount++;
            return false;
        } else {
            i2cErrorCount = 0;
        }

        if (!calibrated)
        {
            biasX = ax; biasY = ay; biasZ = az;
            lastAx = ax; lastAy = ay; lastAz = az;
            calibrated = true;
            return false;
        }

        ax -= biasX; ay -= biasY; az -= biasZ;

        float dx = ax - lastAx;
        float dy = ay - lastAy;
        float dz = az - lastAz;

        lastAx = ax; lastAy = ay; lastAz = az;

        float delta = sqrtf(dx*dx + dy*dy + dz*dz);

        filteredDelta = (1.0f - smoothingAlpha) * filteredDelta + smoothingAlpha * delta;

        float fastThreshold = map(sensitivity, 20, 100, (int)fastThresholdHigh, (int)fastThresholdLow);
        float confirmThreshold = map(sensitivity, 20, 100, (int)confirmThresholdHigh, (int)confirmThresholdLow);

        unsigned long now = millis();

        if (filteredDelta > fastThreshold)
        {
            candidate = true;
            candidateTime = now;
        }

        if (candidate)
        {
            if (filteredDelta > confirmThreshold)
            {
                candidate = false;
                return true;
            }
            if (now - candidateTime > confirmWindowMs)
            {
                candidate = false;
            }
        }

        return false;
    }

    void displayAccelerometer() override {}

    void increaseSensitivity() override
    {
        sensitivity += 10;
        if (sensitivity > 100) sensitivity = 20;
        Serial.printf("[ICM-20948] Sensitivity: %d\n", sensitivity);
    }

    int getSensitivity() const override { return sensitivity; }

    bool isConnected() const override
    {
        return (icm.status == ICM_20948_Stat_Ok && i2cErrorCount < I2C_ERROR_THRESHOLD);
    }
};

// ======================================================
// AUTO-DETECT FACTORY
// ======================================================
class MotionFactory
{
public:
    static Motion* autoDetect()
    {
        Serial.println("\n[MotionFactory] ========== IMU AUTO-DETECTION ==========");

        // Try QMI8658 on I2C FIRST (0x6B address)
        Serial.println("[MotionFactory] Attempting QMI8658 on I2C (0x6B)...");
        {
            QMI8658_I2C_Motion* m = new QMI8658_I2C_Motion();
            if (m->begin()) {
                Serial.println("[MotionFactory] ✓ QMI8658 detected on I2C!");
                Serial.println("[MotionFactory] ==========================================\n");
                return m;
            }
            delete m;
        }

        // Try ICM-20948 (I2C at 0x68)
        Serial.println("[MotionFactory] Attempting ICM-20948 (I2C)...");
        {
            ICM20948Motion* m = new ICM20948Motion();
            if (m->begin()) {
                Serial.println("[MotionFactory] ✓ ICM-20948 detected!");
                Serial.println("[MotionFactory] ==========================================\n");
                return m;
            }
            delete m;
        }

        // Try MPU6050 (I2C at 0x68)
        Serial.println("[MotionFactory] Attempting MPU6050 (I2C)...");
        {
            MPU6050Motion* m = new MPU6050Motion();
            if (m->begin()) {
                Serial.println("[MotionFactory] ✓ MPU6050 detected!");
                Serial.println("[MotionFactory] ==========================================\n");
                return m;
            }
            delete m;
        }

        Serial.println("[MotionFactory] ✗✗✗ NO IMU DETECTED ✗✗✗");
        Serial.println("[MotionFactory] ==========================================\n");
        
        return nullptr;
    }
};