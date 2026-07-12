#ifndef LOCATION_H
#define LOCATION_H

/**
 * GPS  -- T-Beam NEO-6M/8M                          *** REFERENCE ***
 * =================================================================
 * You said you already have location.h. THIS IS THE INTERFACE main.cpp expects:
 *
 *     void begin();
 *     void update();                         // call often in loop()
 *     bool getFix(double& lat, double& lon); // true if a valid fix exists
 *     bool hasFix();
 *
 * Reference body uses TinyGPSPlus (install "TinyGPSPlus") on Serial1.
 * T-Beam v1.1 GPS UART: ESP RX = GPIO34, ESP TX = GPIO12, 9600 baud.
 */

#include <Arduino.h>
#include <TinyGPSPlus.h>

#ifndef GPS_RX_PIN
#define GPS_RX_PIN 34      // ESP32 receives GPS TX here
#endif
#ifndef GPS_TX_PIN
#define GPS_TX_PIN 12
#endif
#ifndef GPS_BAUD
#define GPS_BAUD 9600
#endif

class Location {
private:
    TinyGPSPlus gps;
    HardwareSerial& ss = Serial1;
public:
    void begin() {
        ss.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
        Serial.println(F("[GPS] serial started"));
    }

    // Feed bytes to the parser; call as often as possible.
    void update() {
        while (ss.available() > 0) gps.encode(ss.read());
    }

    bool hasFix() {
        return gps.location.isValid() && gps.location.age() < 5000;
    }

    bool getFix(double& lat, double& lon) {
        if (!hasFix()) return false;
        lat = gps.location.lat();
        lon = gps.location.lng();
        return true;
    }
};

#endif // LOCATION_H
