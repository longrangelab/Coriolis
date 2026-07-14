#ifndef BAROMETRIC_PRESSURE_H
#define BAROMETRIC_PRESSURE_H

#include <Adafruit_BMP085.h>
#include <Wire.h>

/*
  BarometricPressure.h

  - Implements BarometricSensor and BMP085Sensor.
  - Applies a default single-point calibration offset so a raw reading
    of ~78.0°F is corrected to ~71.0°F (offset = -7°F).
  - Provides helpers to change or auto-calibrate the offset at runtime.
  - All math uses float to avoid integer truncation.
*/

class BarometricSensor {
public:
    virtual bool init() = 0;
    virtual float readTemperatureC() = 0;   // calibrated Celsius
    virtual float readPressurePa() = 0;

    // Return calibrated Fahrenheit (uses float math)
    virtual float readTemperatureF() {
        float c = readTemperatureC();
        return (c * 9.0f / 5.0f) + 32.0f;
    }

    // Convert Pascals to inches of mercury (inHg)
    virtual float readPressureInHg() {
        // 1 inHg ≈ 3386.389 Pa
        return readPressurePa() / 3386.389f;
    }

    virtual ~BarometricSensor() {}
};


// BMP085 implementation with calibration support
class BMP085Sensor : public BarometricSensor {
private:
    Adafruit_BMP085 bmp;

    // Calibration applied in Celsius domain:
    // correctedC = rawC * scaleC + offsetC
    float scaleC;
    float offsetC;

public:
    // Default constructor: apply single-point offset so 78°F -> 71°F
    // offsetF = actualF - measuredF = 71 - 78 = -7.0
    // offsetC = offsetF * 5/9 = -7 * 5/9 ≈ -3.8888889 C
    BMP085Sensor() : bmp(), scaleC(1.0f), offsetC(-3.8888889f) {}

    bool init() override {
        if (!bmp.begin()) {
            Serial.println("BMP085 init failed. Check wiring!");
            return false;
        }
        Serial.println("BMP085 initialized.");
        Serial.printf("BMP085 default calibration applied: scaleC=%.6f offsetC=%.6f (F offset ≈ %.3f°F)\n",
                      scaleC, offsetC, offsetC * 9.0f / 5.0f);
        return true;
    }

    // Read raw temperature from sensor, apply calibration (scale + offset)
    float readTemperatureC() override {
        float rawC = bmp.readTemperature(); // Adafruit returns float Celsius
        return (rawC * scaleC) + offsetC;
    }

    float readPressurePa() override {
        return bmp.readPressure();
    }

    // ---------------- Calibration helpers ----------------

    // Set a single-point offset in Fahrenheit (applied in Celsius domain)
    // offsetF = actualF - measuredF
    void setSinglePointOffsetF(float offsetF) {
        offsetC = offsetF * 5.0f / 9.0f;
        Serial.printf("BMP085 single-point offset set: offsetF=%.3f -> offsetC=%.6f\n", offsetF, offsetC);
    }

    // Convenience: set offset by specifying measuredF and actualF
    // measuredF = what the sensor currently reports (uncalibrated)
    // actualF   = true temperature
    void calibrateSinglePointFromMeasuredF(float measuredF, float actualF) {
        float offsetF = actualF - measuredF;
        setSinglePointOffsetF(offsetF);
    }

    // Two-point linear calibration (Fahrenheit inputs)
    void calibrateTwoPointsF(float rawF1, float trueF1, float rawF2, float trueF2) {
        // Convert to Celsius
        float rawC1  = (rawF1 - 32.0f) * 5.0f / 9.0f;
        float trueC1 = (trueF1 - 32.0f) * 5.0f / 9.0f;
        float rawC2  = (rawF2 - 32.0f) * 5.0f / 9.0f;
        float trueC2 = (trueF2 - 32.0f) * 5.0f / 9.0f;
        calibrateTwoPointsC(rawC1, trueC1, rawC2, trueC2);
    }

    // Two-point linear calibration (Celsius inputs)
    void calibrateTwoPointsC(float rawC1, float trueC1, float rawC2, float trueC2) {
        if (fabs(rawC2 - rawC1) < 1e-6f) {
            // Degenerate: fallback to single-point offset
            offsetC = trueC1 - rawC1;
            scaleC = 1.0f;
            Serial.println("BMP085 two-point calibration degenerate; applied single-point offset.");
            return;
        }
        scaleC = (trueC2 - trueC1) / (rawC2 - rawC1);
        offsetC = trueC1 - (scaleC * rawC1);
        Serial.printf("BMP085 two-point calibration applied: scaleC=%.6f, offsetC=%.6f\n", scaleC, offsetC);
    }

    // Directly set scale and offset (Celsius domain)
    void setCalibrationC(float newScaleC, float newOffsetC) {
        scaleC = newScaleC;
        offsetC = newOffsetC;
        Serial.printf("BMP085 calibration set: scaleC=%.6f, offsetC=%.6f\n", scaleC, offsetC);
    }

    // Reset calibration to identity (no correction)
    void resetCalibration() {
        scaleC = 1.0f;
        offsetC = 0.0f;
        Serial.println("BMP085 calibration reset to default (no correction).");
    }

    // Auto-calibrate single-point using current raw reading and a provided true Fahrenheit
    // This reads the current raw sensor value, computes the offset needed to make
    // the sensor match trueF, and stores that offset.
    void autoCalibrateNowUsingTrueF(float trueF) {
        float rawC = bmp.readTemperature();
        float rawF = (rawC * 9.0f / 5.0f) + 32.0f;
        float offsetF = trueF - rawF;
        setSinglePointOffsetF(offsetF);
        Serial.printf("Auto-calibrated: rawF=%.3f trueF=%.3f offsetF=%.3f\n", rawF, trueF, offsetF);
    }

    // Accessors
    float getScaleC() const { return scaleC; }
    float getOffsetC() const { return offsetC; }
    float getOffsetF() const { return offsetC * 9.0f / 5.0f; }
};

#endif // BAROMETRIC_PRESSURE_H
