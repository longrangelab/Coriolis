#include <Arduino.h>
#include <XPowersLib.h>
#include <Wire.h>
// Abstract Power Management class
class PowerManagement
{
public:
    virtual bool init() = 0;                  // Initialize the power management module
    virtual float getBatteryPercentage() = 0; // Get battery percentage
    virtual float getBatteryVoltage() = 0;    // Get battery voltage
};
class AXPManagement : public PowerManagement
{
private:
    XPowersLibInterface *power;
    uint8_t i2c_sda;
    uint8_t i2c_scl;

public:
    AXPManagement(uint8_t sda = 21, uint8_t scl = 22) : power(nullptr), i2c_sda(sda), i2c_scl(scl) {}

    bool init() override
    {
        //         // Try initializing AXP2101
        // // #ifndef TTGO_T_BEAM_V1_1
        //         power = new XPowersAXP2101(Wire1, i2c_sda, i2c_scl);
        //         if (power->init())
        //         {
        //             Serial.println("AXP2101 initialized");
        //             return true;
        //         }
        //         delete power;
        // // #endif
        //         // Try initializing AXP192
        //         power = new XPowersAXP192(Wire1, i2c_sda, i2c_scl);
        //         if (power->init())
        //         {
        //             Serial.println("AXP192 initialized");
        //             return true;
        //         }
        //         delete power;

        //         power = nullptr;
        //         Serial.println("Failed to initialize AXP PMU");
        //         return false;
        if (!power)
        {
            power = new XPowersAXP2101(Wire, i2c_sda, i2c_scl);
            if (!power->init())
            {
                Serial.printf("Tbeam:Warning: Failed to find AXP2101 power management\n");
                delete power;
                power = NULL;
            }
            else
            {
                Serial.printf("Tbeam:AXP2101 PMU init succeeded, using AXP2101 PMU\n");
        return true;

            }
        }

        if (!power)
        {
            power = new XPowersAXP192(Wire, i2c_sda, i2c_scl);
            if (!power->init())
            {
                Serial.printf("Tbeam:Warning: Failed to find AXP192 power management\n");
                delete power;
                power = NULL;
            }
            else
            {
                Serial.printf("Tbeam:AXP192 PMU init succeeded, using AXP192 PMU\n");
        return true;
            }
        }
        return false;
    }

    float getBatteryPercentage() override
    {
        if (!power || !power->isBatteryConnect())
        {
            Serial.println("Tbeam:Battery not connected or PMU not initialized.");
            return -1.0;
        }
        return power->getBatteryPercent();
    }

    float getBatteryVoltage() override
    {
        if (!power || !power->isBatteryConnect())
        {
            Serial.println("Tbeam:Battery not connected or PMU not initialized.");
            return -1.0;
        }
        return power->getBattVoltage() / 1000.0; // Convert to volts
    }

    ~AXPManagement()
    {
        if (power)
        {
            delete power;
        }
    }
};