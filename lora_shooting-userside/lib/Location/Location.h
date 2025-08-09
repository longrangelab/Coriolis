#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <cmath> // For trigonometric functions
// #define PI 3.14159265358979323846
#define R 6371.0 // Earth's radius in kilometers
// Abstract GPS class
class GPS
{
public:
  virtual void begin() = 0;                                             // Initialize GPS module
  virtual bool getCoordinates(double &latitude, double &longitude) = 0; // Get current coordinates
  virtual void displayInfo() = 0;                                       // Display
  // Static method to calculate distance between two coordinates
  static double calculateDistance(double lat1, double lon1, double lat2, double lon2)
  {
    // Ensure coordinates are valid (not zero)
    if (lat1 == 0 || lon1 == 0 || lat2 == 0 || lon2 == 0)
    {
      Serial.println("Invalid coordinates: Coordinates must not be zero.");
      return -1; // Return -1 to indicate an invalid result
    }

    // Convert degrees to radians

    double lat1Rad = lat1 * PI / 180.0;
    double lon1Rad = lon1 * PI / 180.0;
    double lat2Rad = lat2 * PI / 180.0;
    double lon2Rad = lon2 * PI / 180.0;

    // Haversine formula
    double dlat = lat2Rad - lat1Rad;
    double dlon = lon2Rad - lon1Rad;
    double a = sin(dlat / 2) * sin(dlat / 2) +
               cos(lat1Rad) * cos(lat2Rad) * sin(dlon / 2) * sin(dlon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));

    return R * c * 1093.6; // Distance in kilometers to Yard
  }
};

// Implementation for M8N or M6N GPS modules
class GPSModule : public GPS
{
private:
  TinyGPSPlus gps;
  HardwareSerial &gpsSerial;
  int rxPin;
  int txPin;

public:
  // Constructor to specify which hardware serial to use
  GPSModule(HardwareSerial &serial, int rx, int tx) : gpsSerial(serial), rxPin(rx), txPin(tx) {}

  void begin() override
  {
    gpsSerial.begin(9600, SERIAL_8N1, rxPin, txPin); // Start GPS serial communication
    // bool result = false;
    // for (int i = 0; i < 5; ++i)
    // {
    //   result = l76kProbe();
    //   if (result)
    //   {
    //     break;
    //   }
    // }
    // if (result)
    //   Serial.println("GPS module initialized.");
    // else
    //   Serial.println("GPS module not initialized");
  }
  bool l76kProbe()
  {
    bool result = false;
    uint32_t startTimeout;
    gpsSerial.write("$PCAS03,0,0,0,0,0,0,0,0,0,0,,,0,0*02\r\n");
    delay(5);
    // Get version information
    startTimeout = millis() + 3000;
    Serial.print("Try to init L76K . Wait stop .");
    // SerialGPS.flush();
    while (gpsSerial.available())
    {
      int c = gpsSerial.read();
      // Serial.write(c);
      // Serial.print(".");
      // Serial.flush();
      // SerialGPS.flush();
      if (millis() > startTimeout)
      {
        Serial.println("Wait L76K stop NMEA timeout!");
        return false;
      }
    };
    Serial.println();
    gpsSerial.flush();
    delay(200);

    gpsSerial.write("$PCAS06,0*1B\r\n");
    startTimeout = millis() + 1000;
    String ver = "";
    while (!gpsSerial.available())
    {
      if (millis() > startTimeout)
      {
        Serial.println("Get L76K timeout!");
        return false;
      }
    }
    gpsSerial.setTimeout(10);
    ver = gpsSerial.readStringUntil('\n');
    if (ver.startsWith("$GPTXT,01,01,02"))
    {
      Serial.println("L76K GNSS init succeeded, using L76K GNSS Module\n");
      result = true;
    }
    delay(500);

    // Initialize the L76K Chip, use GPS + GLONASS
    gpsSerial.write("$PCAS04,5*1C\r\n");
    delay(250);
    // only ask for RMC and GGA
    gpsSerial.write("$PCAS03,1,0,0,0,1,0,0,0,0,0,,,0,0*02\r\n");
    delay(250);
    // Switch to Vehicle Mode, since SoftRF enables Aviation < 2g
    gpsSerial.write("$PCAS11,3*1E\r\n");
    return result;
  }

  bool getCoordinates(double &latitude, double &longitude) override
  {
    while (gpsSerial.available() > 0)
    {
      if (gps.encode(gpsSerial.read()))
      {
        // displayInfo();
        if (gps.location.isValid())
        {
          Serial.print(gps.location.lat(), 6);
          Serial.print(F(","));
          Serial.print(gps.location.lng(), 6);
          latitude = gps.location.lat();
          longitude = gps.location.lng();
        }
        else
        {
          Serial.println(F("INVALID location"));
        }

        return true; // Coordinates successfully retrieved
      }

      // if (gps.location.isUpdated())
      // }
      // {
    }
    return false; // No new data available
  }
  void displayInfo() override
  {
    Serial.print(F("Location: "));
    if (gps.location.isValid())
    {
      Serial.print(gps.location.lat(), 6);
      Serial.print(F(","));
      Serial.print(gps.location.lng(), 6);
    }
    else
    {
      Serial.print(F("INVALID"));
    }

    Serial.print(F("  Date/Time: "));
    if (gps.date.isValid())
    {
      Serial.print(gps.date.month());
      Serial.print(F("/"));
      Serial.print(gps.date.day());
      Serial.print(F("/"));
      Serial.print(gps.date.year());
    }
    else
    {
      Serial.print(F("INVALID"));
    }

    Serial.print(F(" "));
    if (gps.time.isValid())
    {
      if (gps.time.hour() < 10)
        Serial.print(F("0"));
      Serial.print(gps.time.hour());
      Serial.print(F(":"));
      if (gps.time.minute() < 10)
        Serial.print(F("0"));
      Serial.print(gps.time.minute());
      Serial.print(F(":"));
      if (gps.time.second() < 10)
        Serial.print(F("0"));
      Serial.print(gps.time.second());
      Serial.print(F("."));
      if (gps.time.centisecond() < 10)
        Serial.print(F("0"));
      Serial.print(gps.time.centisecond());
    }
    else
    {
      Serial.print(F("INVALID"));
    }

    Serial.println();
  }
};
