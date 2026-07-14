#ifndef BARO_H
#define BARO_H

/**
 * BMP180 barometric pressure + temperature  (OPTIONAL)
 * ====================================================
 * The BMP180 is register-compatible with the BMP085, so this uses the
 * Adafruit_BMP085 library (install "Adafruit BMP085 Library").
 *
 * Designed to be non-fatal: if the sensor isn't found, begin() returns false
 * and the node keeps running. read() simply reports valid=false.
 *
 * Shares the same I2C bus as the OLED and the ATtiny85 (Wire on 21/22).
 * BMP180 default address is 0x77.
 */

#include <Wire.h>
#include <Adafruit_BMP085.h>

class Barometer {
private:
    Adafruit_BMP085 bmp;
    bool present = false;
public:
    // Wire.begin() must already have been called in main.cpp.
    bool begin() {
        present = bmp.begin();          // probes 0x77
        Serial.print(F("[Baro] BMP180 "));
        Serial.println(present ? F("found") : F("not found (skipping)"));
        return present;
    }

    bool isPresent() const { return present; }

    // Returns false (and leaves outputs untouched) if sensor absent/erroring.
    bool read(float& temperatureC, float& pressurePa) {
        if (!present) return false;
        temperatureC = bmp.readTemperature();      // degC
        pressurePa   = bmp.readPressure();         // Pa
        // crude sanity check
        if (pressurePa < 30000.0f || pressurePa > 120000.0f) return false;
        return true;
    }
};

#endif // BARO_H
