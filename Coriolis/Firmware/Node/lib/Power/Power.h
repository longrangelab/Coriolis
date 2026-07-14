#ifndef POWER_H
#define POWER_H

/**
 * Power / PMU  --  T-Beam v1.2 (AXP2101) and v1.1 (AXP192)
 * =======================================================
 * Auto-detects the chip (AXP2101 first, then AXP192). Enables the battery ADC
 * (the missing step that made v1.2 read 0 V) and the GPS + LoRa rails.
 *
 * Adds gpsPower(on/off): lets a stationary node power the GPS DOWN after it has
 * a fix (GPS draws ~25-40 mA continuously -- a big battery saver for a fixed
 * meter that only needs its position once).
 *
 *   v1.2 (AXP2101): ALDO2 = LoRa, ALDO3 = GPS, DCDC1 = ESP32
 *   v1.1 (AXP192) : LDO2  = LoRa, LDO3  = GPS
 */

#include <Arduino.h>
#include <XPowersLib.h>
#include <Wire.h>

class PowerManagement {
public:
    virtual bool  init() = 0;
    virtual float getBatteryPercentage() = 0;   // 0..100, or -1
    virtual float getBatteryVoltage() = 0;      // volts, or -1
    virtual void  gpsPower(bool on) = 0;
};

class AXPManagement : public PowerManagement {
private:
    XPowersLibInterface *power   = nullptr;
    XPowersAXP2101      *axp2101 = nullptr;
    XPowersAXP192       *axp192  = nullptr;
    uint8_t i2c_sda, i2c_scl;

    static int percentFromVolts(float v) {
        if (v <= 0) return -1;
        int p = (int)((v - 3.30f) / (4.20f - 3.30f) * 100.0f + 0.5f);
        if (p < 0) p = 0; if (p > 100) p = 100;
        return p;
    }

public:
    AXPManagement(uint8_t sda = 21, uint8_t scl = 22) : i2c_sda(sda), i2c_scl(scl) {}

    bool init() override {
        // ---- AXP2101 (v1.2) ----
        axp2101 = new XPowersAXP2101(Wire, i2c_sda, i2c_scl);
        if (axp2101->init()) {
            power = axp2101;
            axp2101->enableBattDetection();
            axp2101->enableBattVoltageMeasure();       // fixes 0 V bug
            axp2101->setALDO2Voltage(3300); axp2101->enableALDO2();   // LoRa
            axp2101->setALDO3Voltage(3300); axp2101->enableALDO3();   // GPS
            Serial.println(F("Tbeam: AXP2101 PMU init (v1.2)"));
            return true;
        }
        delete axp2101; axp2101 = nullptr;

        // ---- AXP192 (v1.1) ----
        axp192 = new XPowersAXP192(Wire, i2c_sda, i2c_scl);
        if (axp192->init()) {
            power = axp192;
            axp192->enableBattDetection();
            axp192->enableBattVoltageMeasure();
            axp192->setLDO2Voltage(3300); axp192->enableLDO2();       // LoRa
            axp192->setLDO3Voltage(3300); axp192->enableLDO3();       // GPS
            Serial.println(F("Tbeam: AXP192 PMU init (v1.1)"));
            return true;
        }
        delete axp192; axp192 = nullptr;

        Serial.println(F("Tbeam: no AXP PMU found"));
        return false;
    }

    // Power the GPS rail on/off (battery saver for stationary nodes).
    void gpsPower(bool on) override {
        if (axp2101) { on ? axp2101->enableALDO3() : axp2101->disableALDO3(); }
        else if (axp192) { on ? axp192->enableLDO3() : axp192->disableLDO3(); }
    }

    float getBatteryVoltage() override {
        if (!power) return -1.0f;
        uint16_t mv = power->getBattVoltage();
        return (mv > 100) ? mv / 1000.0f : -1.0f;
    }

    float getBatteryPercentage() override {
        if (!power) return -1.0f;
        if (axp2101) {
            int p = axp2101->getBatteryPercent();
            if (p >= 0) return (float)p;
        }
        return (float)percentFromVolts(getBatteryVoltage());
    }

    ~AXPManagement() { if (power) delete power; }
};

#endif // POWER_H
