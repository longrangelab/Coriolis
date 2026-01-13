#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <math.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "SensorQMI8658.hpp"
#include "ICM_20948.h"   // SparkFun ICM-20948 library

#define SPI_MOSI (35)
#define SPI_SCK  (36)
#define SPI_MISO (37)
#define IMU_CS   (34)
#define IMU_INT  (33)

// ======================================================
// Abstract Motion class
// ======================================================
class Motion
{
public:
    virtual ~Motion() {}
    virtual bool begin() = 0;                // return false if device not present
    virtual bool detectMotion() = 0;         // called at steady cadence; return true if hit
    virtual void displayAccelerometer() = 0;
    virtual void increaseSensitivity() = 0;
    virtual int  getSensitivity() const = 0;
    virtual void calibrate() {}              // optional calibration hook
    virtual bool isConnected() const { return true; } // default: assume connected
};

// ======================================================
// MPU6050 implementation
// ======================================================
class MPU6050Motion : public Motion
{
private:
    Adafruit_MPU6050 mpu;
    int sensitivity = 20;

public:
    bool begin() override
    {
        if (!mpu.begin())
        {
            Serial.println("MPU6050 not detected.");
            return false;
        }

        Serial.println("MPU6050 detected.");

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
        Serial.print("MPU6050 Sensitivity: ");
        Serial.println(sensitivity);
    }

    int getSensitivity() const override { return sensitivity; }

    bool isConnected() const override
    {
        // Lightweight sanity check: request a sample and ensure values are finite
        sensors_event_t a, g, temp;
        if (!const_cast<Adafruit_MPU6050&>(mpu).getEvent(&a, &g, &temp)) return false;
        return isfinite(a.acceleration.x) && isfinite(a.acceleration.y) && isfinite(a.acceleration.z);
    }
};

// ======================================================
// QMI8658 implementation
// ======================================================
// Note: qmiInterruptFlag is defined here for simplicity; main.cpp attaches the ISR that sets it.
volatile bool qmiInterruptFlag = false;
void qmiSetFlag() { qmiInterruptFlag = true; }

class QMI8658Motion : public Motion
{
private:
    SensorQMI8658 qmi;
    int csPin, intPin;
    int sensitivity = 20;

public:
    QMI8658Motion(int cs = IMU_CS, int irq = IMU_INT) : csPin(cs), intPin(irq) {}

    bool begin() override
    {
        qmi.setPins(IMU_INT);
        if (!qmi.begin(csPin, SPI_MOSI, SPI_MISO, SPI_SCK))
        {
            Serial.println("QMI8658 not detected.");
            return false;
        }

        Serial.println("QMI8658 detected.");

        qmi.configAccelerometer(SensorQMI8658::ACC_RANGE_4G,
                                SensorQMI8658::ACC_ODR_500Hz);
        qmi.enableAccelerometer();

        uint8_t modeCtrl = SensorQMI8658::ANY_MOTION_EN_X |
                           SensorQMI8658::ANY_MOTION_EN_Y |
                           SensorQMI8658::ANY_MOTION_EN_Z |
                           SensorQMI8658::NO_MOTION_EN_X |
                           SensorQMI8658::NO_MOTION_EN_Y |
                           SensorQMI8658::NO_MOTION_EN_Z;

        qmi.configMotion(
            modeCtrl,
            100.0, 100.0, 1.0, 1,
            0.1, 0.1, 0.1, 1,
            1, 1
        );

        qmi.enableMotionDetect(SensorQMI8658::INTERRUPT_PIN_1);
        // ISR attach is done in main.cpp to keep single definition of ISR

        return true;
    }

    bool detectMotion() override
    {
        if (qmiInterruptFlag)
        {
            qmiInterruptFlag = false;
            uint8_t status = qmi.getStatusRegister();
            if (status & SensorQMI8658::EVENT_ANY_MOTION) return true;
            if (status & SensorQMI8658::EVENT_WOM_MOTION) return true;
        }
        return false;
    }

    void displayAccelerometer() override {}

    void increaseSensitivity() override
    {
        sensitivity += 10;
        if (sensitivity > 100) sensitivity = 20;
        Serial.print("QMI8658 Sensitivity: ");
        Serial.println(sensitivity);
    }

    int getSensitivity() const override { return sensitivity; }

    bool isConnected() const override
    {
        // Use chip ID or status register if available
        uint8_t id = const_cast<SensorQMI8658&>(qmi).getChipID();
        // Treat 0x00 or 0xFF as invalid/disconnected
        return (id != 0x00 && id != 0xFF);
    }
};

// ======================================================
// ICM-20948 implementation (tuned for robust hit detection)
// ======================================================
class ICM20948Motion : public Motion
{
private:
    ICM_20948_I2C icm;
    int sensitivity = 20;

    // Calibration bias (captured at boot)
    float biasX = 0, biasY = 0, biasZ = 0;
    bool calibrated = false;

    // Smoothing and two-stage detection state
    float lastAx = 0, lastAy = 0, lastAz = 0;
    float filteredDelta = 0;
    unsigned long candidateTime = 0;
    bool candidate = false;

    // Tunable parameters
    float smoothingAlpha = 0.4f; // 0..1 (higher = faster response)
    unsigned long confirmWindowMs = 120; // confirmation window
    float fastThresholdLow = 300.0f;
    float fastThresholdHigh = 900.0f;
    float confirmThresholdLow = 600.0f;
    float confirmThresholdHigh = 1200.0f;

    // I2C error tracking for robust disconnect detection
    int i2cErrorCount = 0;
    const int I2C_ERROR_THRESHOLD = 3;

public:
    bool begin() override
    {
        icm.begin(Wire, 0x68);

        if (icm.status != ICM_20948_Stat_Ok)
        {
            Serial.println("ICM-20948 not detected.");
            return false;
        }

        Serial.println("ICM-20948 detected.");

        icm.setSampleMode(ICM_20948_Internal_Acc,
                          ICM_20948_Sample_Mode_Continuous);

        ICM_20948_fss_t fss;
        fss.a = gpm4; // +/-4g
        icm.setFullScale(ICM_20948_Internal_Acc, fss);

        // reset state
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
        // Average a number of samples at boot to capture bias
        const int N = 50;
        float sumX = 0, sumY = 0, sumZ = 0;
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
        Serial.println("ICM-20948 calibrated.");
    }

    bool detectMotion() override
    {
        // Called at steady cadence from main loop
        // If previous I2C errors exceeded threshold, skip reads and return false
        if (i2cErrorCount >= I2C_ERROR_THRESHOLD) {
            return false;
        }

        // Attempt read
        icm.getAGMT();

        float ax = icm.accX();
        float ay = icm.accY();
        float az = icm.accZ();

        // Sanity check for I2C read success
        if (!isfinite(ax) || !isfinite(ay) || !isfinite(az)) {
            i2cErrorCount++;
            Serial.print("[IMU] I2C read failed, count=");
            Serial.println(i2cErrorCount);
            return false;
        } else {
            // successful read, reset error counter
            i2cErrorCount = 0;
        }

        if (!calibrated)
        {
            // if not calibrated, use first sample as baseline
            biasX = ax; biasY = ay; biasZ = az;
            lastAx = ax; lastAy = ay; lastAz = az;
            calibrated = true;
            return false;
        }

        // remove bias
        ax -= biasX; ay -= biasY; az -= biasZ;

        // delta from last sample
        float dx = ax - lastAx;
        float dy = ay - lastAy;
        float dz = az - lastAz;

        lastAx = ax; lastAy = ay; lastAz = az;

        float delta = sqrtf(dx*dx + dy*dy + dz*dz);

        // exponential smoothing
        filteredDelta = (1.0f - smoothingAlpha) * filteredDelta + smoothingAlpha * delta;

        // map sensitivity to thresholds
        float fastThreshold = map(sensitivity, 20, 100, (int)fastThresholdHigh, (int)fastThresholdLow);
        float confirmThreshold = map(sensitivity, 20, 100, (int)confirmThresholdHigh, (int)confirmThresholdLow);

        unsigned long now = millis();

        // two-stage detection
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
                // minimal debug
                Serial.print("ICM-20948 motion confirmed: filt=");
                Serial.print(filteredDelta);
                Serial.print(" fastThr=");
                Serial.print(fastThreshold);
                Serial.print(" confThr=");
                Serial.println(confirmThreshold);
                return true;
            }
            if (now - candidateTime > confirmWindowMs)
            {
                candidate = false;
            }
        }

        return false;
    }

    void displayAccelerometer() override
    {
        icm.getAGMT();
        Serial.print("ICM Accel X:");
        Serial.print(icm.accX());
        Serial.print(" Y:");
        Serial.print(icm.accY());
        Serial.print(" Z:");
        Serial.println(icm.accZ());
    }

    void increaseSensitivity() override
    {
        sensitivity += 10;
        if (sensitivity > 100) sensitivity = 20;
        Serial.print("ICM-20948 Sensitivity: ");
        Serial.println(sensitivity);
    }

    int getSensitivity() const override { return sensitivity; }

    bool isConnected() const override
    {
        // Check driver status and a quick sample sanity check
        if (icm.status != ICM_20948_Stat_Ok) return false;
        // call getAGMT and read values; remove const for call
        const_cast<ICM_20948_I2C&>(icm).getAGMT();
        float x = const_cast<ICM_20948_I2C&>(icm).accX();
        float y = const_cast<ICM_20948_I2C&>(icm).accY();
        float z = const_cast<ICM_20948_I2C&>(icm).accZ();
        if (!isfinite(x) || !isfinite(y) || !isfinite(z)) return false;
        // also ensure we haven't seen repeated I2C errors
        return (i2cErrorCount < I2C_ERROR_THRESHOLD);
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
        Serial.println("Auto-detecting IMU...");

        // 1. Try MPU6050
        {
            MPU6050Motion* m = new MPU6050Motion();
            if (m->begin()) return m;
            delete m;
        }

        // 2. Try ICM-20948
        {
            ICM20948Motion* m = new ICM20948Motion();
            if (m->begin()) return m;
            delete m;
        }

        // 3. Try QMI8658
        {
            QMI8658Motion* m = new QMI8658Motion(IMU_CS, IMU_INT);
            if (m->begin()) return m;
            delete m;
        }

        Serial.println("ERROR: No IMU detected!");
        return nullptr;
    }
};
