#ifndef BAROMETRIC_PRESSURE_H
#define BAROMETRIC_PRESSURE_H

#include <Adafruit_BMP085.h>
#include <Wire.h>

// Abstract class for barometric pressure sensor
class BarometricSensor {
public:
    virtual bool init() = 0;
    virtual float readTemperatureC() = 0;
    virtual float readPressurePa() = 0;

    // Default implementation
    virtual float readTemperatureF() {
        return readTemperatureC() * 9.0 / 5.0 + 32.0;
    }

    virtual float readPressureInHg() {
        return readPressurePa() / 3386.39;
    }

    virtual ~BarometricSensor() {}
};

// BMP085 implementation of BarometricSensor
class BMP085Sensor : public BarometricSensor {
private:
    Adafruit_BMP085 bmp;

public:
    bool init() override {
        if (!bmp.begin()) {
            Serial.println("BMP085 init failed. Check wiring!");
            return false;
        }
        Serial.println("BMP085 initialized.");
        return true;
    }

    float readTemperatureC() override {
        return bmp.readTemperature();
    }

    float readPressurePa() override {
        return bmp.readPressure();
    }
};

#endif  // BAROMETRIC_PRESSURE_H