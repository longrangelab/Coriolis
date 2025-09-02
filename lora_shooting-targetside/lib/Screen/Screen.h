#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <U8g2lib.h> // For SH1106 support

// Display dimensions
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SSD1306_I2C_ADDRESS 0x3C
// OLED reset pin
#define OLED_RESET -1

// Abstract base class for displays
class Screen
{
public:
  virtual void begin() = 0; // Initialize the display
  virtual void clear() = 0; // Clear the display
  virtual void drawInterface(int batteryPercent, int signalStrength, bool hitFlag, int distance, const char*targetAddress, int windDirection, int windSpeed, int imuSensitivity) = 0; // Loan added temperature and pressure parameters
  virtual void drawHitNotification() = 0;
};

// SSD1306 implementation
class SSD1306Screen : public Screen
{
private:
  Adafruit_SSD1306 display;

public:
  SSD1306Screen() : display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET) {}

  void begin() override
  {
    if (!display.begin(SSD1306_I2C_ADDRESS, 0x3C))
    {
      Serial.println("SSD1306 initialization failed!");
      while (true)
        ; // Halt on error
    }
    Serial.println("Tbeam:SSD1306 init!");
    display.clearDisplay();
    display.setCursor(0, 10);
    display.setTextSize(2);
    display.print("Hello!!");
    display.display();
  }

  void clear() override
  {
    display.clearDisplay();
  }
  void drawHitNotification() override
  {
    clear();
    display.setCursor(0, 10);
    display.setTextSize(4);

    display.print("Hit!!");
    display.display();
  }
  void drawInterface(int batteryPercent, int signalStrength, bool hitFlag, int distance, const char*targetAddress, int windDirection, int windSpeed, int imuSensitivity) override // Loan added temperature and pressure parameters
  {
    clear();

    // Draw the battery icon
    display.drawRect(5, 5, 20, 10, WHITE);                                // Battery outline
    display.fillRect(25, 8, 2, 4, WHITE);                                 // Battery tip
    display.fillRect(7, 7, map(batteryPercent, 0, 100, 0, 15), 6, WHITE); // Battery level based on signal strength

    // Add signal text
    display.setCursor(35, 5);
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.printf("%d dBm", signalStrength);

    // Add Hit flag text
    display.setCursor(5, 20);
    display.setTextSize(1);
    if (hitFlag)
    {
      display.print("Hit!!");
    }

    // Add distance text
    display.setCursor(5, 35);
    display.setTextSize(1);
    display.printf("%d Yards", distance);

    // Add target address text
    display.setCursor(5, 45);
    display.printf("Tar:%s", targetAddress);

    //Loan added.. Temperature and pressure (line 5 of 6)
    // display.setCursor(5, 55);
    // display.printf("%.2fF-%.2finHg", temperatureF, pressureInHg);
    //Loan added.. Temperature and pressure (line 5 of 6)

    
    //Loan Add IMU Sensitivity (last line)
    display.setCursor(75, 5);
    display.setTextSize(1);
    display.printf("IMU:%d", imuSensitivity);
    //Loan Add IMU Sensitivity (last line)

    // Add wind direction text
    display.setTextSize(2);
    display.setCursor(90, 15);
    display.printf("%d", windDirection);
    display.setCursor(54, 20);
    display.setTextSize(1);
    display.printf("Dir:");
// Convert RPM to MPH using your formula
float windSpeedMPH = 0.0384 * windSpeed - 1.613;
if (windSpeedMPH < 0) windSpeedMPH = 0;  // Prevent negative display

// Display MPH value
display.setCursor(54, 35);
display.setTextSize(2);
display.printf("%.1f", windSpeedMPH);  // Show one decimal place
display.setCursor(105, 40);  // Shift further right (near edge of 128px width)
display.setTextSize(1);
display.print("MPH");


   
    // display.setTextSize(2);
    // display.setCursor(60, 20);
    // display.printf("%d", windDirection);
    // if(windMode == 0) {
    //   display.setCursor(70, 45);
    //   display.setTextSize(1);
    //   display.printf("Direction");
    // }
    // if(windMode == 1) {
    //   display.setCursor(65, 45);
    //   display.setTextSize(1);
    //   display.printf("Speed(RPM)");
    // }
    // Display everything
    display.display();
  }
};

// SH1106 implementation
class SH1106Screen : public Screen
{
private:
  U8G2_SH1106_128X64_NONAME_F_HW_I2C display;

public:
  SH1106Screen() : display(U8G2_R0, /* reset=*/U8X8_PIN_NONE)
  {
    Serial.print("SH1106Screen initialized");
  }
  // Constructor with configurable I2C pins
  SH1106Screen(uint8_t sdaPin, uint8_t sclPin) : display(U8G2_R0, /* reset=*/U8X8_PIN_NONE)
  {
    Wire.begin(sdaPin, sclPin); // Initialize I2C with custom pins
    Serial.println("SH1106Screen initialized with custom I2C pins");
  }
  void begin() override
  {
    display.begin();
    display.clearBuffer();
    display.setFont(u8g2_font_logisoso16_tf);

    display.setCursor(0, 10);
    display.print("Starting...");
    display.sendBuffer();
  }

  void clear() override
  {
    display.clearBuffer();
  }
  void drawHitNotification() override
  {
    clear();
    display.setFont(u8g2_font_6x10_tr);
    display.drawStr(0, 15, "Hit!!");
    display.sendBuffer();
  }
  void drawInterface(int batteryPercent, int signalStrength, bool hitFlag, int distance, const char*targetAddress, int windDirection, int windSpeed, int imuSensitivity) override
  {
    clear();

    // Draw the battery icon
    display.drawFrame(5, 5, 20, 10);                              // Battery outline
    display.drawBox(25, 8, 2, 4);                                 // Battery tip
    display.drawBox(7, 7, map(batteryPercent, 0, 100, 0, 15), 6); // Battery level based on signal strength

    // Add signal text
    display.setFont(u8g2_font_6x10_tr);
    display.drawStr(35, 15, ("Signal: " + String(signalStrength) + "dBm").c_str());

    display.setCursor(100, 5);
    //display.setTextSize(1);
    display.printf("IMU:%d", imuSensitivity);

    // Add Hit flag text
    display.drawStr(5, 30, hitFlag ? "Hit!!" : "");

    // Add distance text
    display.drawStr(5, 45, (String(distance) + " Yards").c_str());

    // Add target address text
    display.drawStr(5, 55, ("From: " + String(targetAddress)).c_str());

    //Loan added.. Temperature and pressure
    //display.drawStr(5, 65, (String(temperatureF, 1) + "F -" + String(pressureInHg, 2) + "inHg").c_str());

    display.drawStr(100, 15, ("IMU:" + String(imuSensitivity)).c_str());
    // Add wind direction text
    display.setFont(u8g2_font_logisoso32_tn);
    display.drawStr(80, 20, (String(windDirection)).c_str());
    display.drawStr(60, 20, (String(windDirection)).c_str());

    // Send buffer to the display
    display.sendBuffer();
  }
};

// Factory class to detect and create appropriate screen instance
class ScreenFactory
{
public:
  static Screen *createScreen()
  {
    // Perform I2C scan to detect display type
    Wire.begin(21, 22);
    Wire.beginTransmission(0x3C); // Address for SSD1306/SH1106
    if (Wire.endTransmission() == 0)
    {
      // Default to SSD1306
      return new SSD1306Screen();
    }
    else
    {
      // Fallback to SH1106
      return new SH1106Screen(17, 18);
    }
  }
};
