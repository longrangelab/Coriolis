#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

/**
 * Environment.h  --  barometric pressure/temperature (BMP180) + GPS
 * =================================================================
 * Self-contained: includes <Arduino.h> and every dependency it uses, so it
 * compiles on its own as a PlatformIO library header (lib/Environment/).
 *
 * Rule of thumb that bit you: ANY header under lib/ that references Serial,
 * PI, String, millis(), etc. must #include <Arduino.h> itself -- unlike a
 * .ino, library headers don't get the Arduino core for free.
 *
 * Provides:  BarometricSensor / BMP085Sensor   (temp in C+F, pressure Pa+inHg)
 *            GPS / GPSModule                    (coords + Haversine distance)
 */

#include <Arduino.h>                 // <-- the fix: defines Serial, PI, String...
#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <math.h>

// ===========================================================================
// BAROMETRIC PRESSURE / TEMPERATURE  (BMP180, via Adafruit_BMP085 library)
// ===========================================================================
class BarometricSensor {
public:
    virtual bool  init() = 0;
    virtual float readTemperatureC() = 0;      // calibrated Celsius
    virtual float readPressurePa()   = 0;

    virtual float readTemperatureF() {          // imperial
        return (readTemperatureC() * 9.0f / 5.0f) + 32.0f;
    }
    virtual float readPressureInHg() {          // 1 inHg = 3386.389 Pa
        return readPressurePa() / 3386.389f;
    }
    virtual ~BarometricSensor() {}
};

class BMP085Sensor : public BarometricSensor {
private:
    Adafruit_BMP085 bmp;
    float scaleC;
    float offsetC;
public:
    // Default single-point offset: 78F -> 71F  (offsetC = -7F * 5/9)
    // NOTE: verify this against a real thermometer -- it feeds the solver.
    BMP085Sensor() : bmp(), scaleC(1.0f), offsetC(-3.8888889f) {}

    bool init() override {
        if (!bmp.begin()) {
            Serial.println(F("BMP085 init failed. Check wiring!"));
            return false;
        }
        Serial.println(F("BMP085 initialized."));
        return true;
    }

    float readTemperatureC() override {
        return (bmp.readTemperature() * scaleC) + offsetC;
    }
    float readPressurePa() override {
        return bmp.readPressure();
    }

    // ---- calibration helpers ----
    void setSinglePointOffsetF(float offsetF) { offsetC = offsetF * 5.0f / 9.0f; }
    void calibrateFromMeasuredF(float measuredF, float actualF) {
        setSinglePointOffsetF(actualF - measuredF);
    }
    void setCalibrationC(float s, float o) { scaleC = s; offsetC = o; }
    void resetCalibration() { scaleC = 1.0f; offsetC = 0.0f; }
    float getOffsetF() const { return offsetC * 9.0f / 5.0f; }
};

// ===========================================================================
// GPS  (NEO-6M/M8N via TinyGPS++)
// ===========================================================================
class GPS {
public:
    virtual void begin() = 0;
    virtual bool getCoordinates(double& latitude, double& longitude) = 0;
    virtual void displayInfo() = 0;

    // Great-circle distance in YARDS. Returns -1 on invalid (zero) coordinates.
    static double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
        if (lat1 == 0 || lon1 == 0 || lat2 == 0 || lon2 == 0) return -1;
        const double EARTH_R_KM = 6371.0;
        double la1 = lat1 * PI / 180.0, lo1 = lon1 * PI / 180.0;
        double la2 = lat2 * PI / 180.0, lo2 = lon2 * PI / 180.0;
        double dlat = la2 - la1, dlon = lo2 - lo1;
        double a = sin(dlat / 2) * sin(dlat / 2) +
                   cos(la1) * cos(la2) * sin(dlon / 2) * sin(dlon / 2);
        double c = 2 * atan2(sqrt(a), sqrt(1 - a));
        return EARTH_R_KM * c * 1093.6;         // km -> yards
    }
};

class GPSModule : public GPS {
private:
    TinyGPSPlus     gps;
    HardwareSerial& gpsSerial;
    int rxPin, txPin;
public:
    GPSModule(HardwareSerial& serial, int rx, int tx)
        : gpsSerial(serial), rxPin(rx), txPin(tx) {}

    void begin() override {
        gpsSerial.begin(9600, SERIAL_8N1, rxPin, txPin);
    }

    bool getCoordinates(double& latitude, double& longitude) override {
        while (gpsSerial.available() > 0) {
            if (gps.encode(gpsSerial.read())) {
                if (gps.location.isValid()) {
                    latitude  = gps.location.lat();
                    longitude = gps.location.lng();
                }
                return true;
            }
        }
        return false;
    }

    void displayInfo() override {
        Serial.print(F("Location: "));
        if (gps.location.isValid()) {
            Serial.print(gps.location.lat(), 6);
            Serial.print(F(","));
            Serial.print(gps.location.lng(), 6);
        } else {
            Serial.print(F("INVALID"));
        }
        Serial.println();
    }
};

#endif // ENVIRONMENT_H
