#ifndef SCREEN_H
#define SCREEN_H

/**
 * Node OLED display (128x64, SSD1306 or SH1106)
 * =============================================
 * Shows: battery, node number, 2 s average wind speed (MPH), wind direction.
 * IMU / hit / distance / target removed (old project).
 *
 * Notes
 *  - Wire.begin() is done ONCE in main.cpp before ScreenFactory::create().
 *    This header no longer calls Wire.begin() (double-init was part of why
 *    the display wasn't coming up).
 *  - FIX: Adafruit SSD1306 begin() takes (vccstate, i2caddr). The old code
 *    passed the address as the first arg, so it never initialized. Fixed.
 *  - Future: node id can be pushed from a phone (BLE) and fed straight into
 *    drawNode(); the display code already takes it as a parameter.
 */

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <U8g2lib.h>

#define SCREEN_WIDTH        128
#define SCREEN_HEIGHT        64
#define OLED_I2C_ADDRESS   0x3C
#define OLED_RESET           -1

// ----------------------------------------------------------------------------
// Abstract display
// ----------------------------------------------------------------------------
class Screen {
public:
    virtual void begin() = 0;
    virtual void clear() = 0;
    // signalDbm: RSSI of last receiver ack / -999 if N/A
    virtual void drawNode(int nodeId, int batteryPercent, int signalDbm,
                          float windSpeedMph, int windDir, bool gpsFix) = 0;
    virtual void drawNodeId(int nodeId) = 0;   // node-ID edit screen
    virtual void powerOn()  = 0;
    virtual void powerOff() = 0;
    virtual bool isOn()     = 0;
};

// ----------------------------------------------------------------------------
// SSD1306
// ----------------------------------------------------------------------------
class SSD1306Screen : public Screen {
private:
    Adafruit_SSD1306 display;
    bool powered;
public:
    SSD1306Screen() : display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET), powered(false) {}

    void begin() override {
        // FIX: first arg is vcc state, second is I2C address.
        if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDRESS)) {
            Serial.println(F("[Screen] SSD1306 init FAILED"));
            return;                       // don't halt the whole node on display fault
        }
        Serial.println(F("[Screen] SSD1306 ready"));
        powered = true;
        windIntro();                  // wind gust -> "CORIOLIS" startup flourish
        display.clearDisplay();
        display.setTextColor(WHITE);
        display.setTextSize(2);
        display.setCursor(0, 10);
        display.print("Wind Node");
        display.display();
    }

    // Wind gust sweeps across, then "CORIOLIS" rides in and settles.
    void windIntro() {
        const int FR = 32;
        const int N  = 10;
        int y0[N], sp[N], x0[N];
        for (int i = 0; i < N; i++) { y0[i] = 4 + i*6; sp[i] = 6 + (i%3); x0[i] = -(i*10); }
        for (int f = 0; f < FR; f++) {
            display.clearDisplay();
            for (int i = 0; i < N; i++) {
                int x = x0[i] + f*sp[i];
                if (x < -8 || x > 136) continue;
                int by  = y0[i] + (x > 0 ? (x*x)/2600 : 0);       // Coriolis curl (down-right)
                int byp = y0[i] + ((x-8) > 0 ? ((x-8)*(x-8))/2600 : 0);
                display.drawLine(x-8, byp, x, by, WHITE);         // motion-blurred streak
                display.drawPixel(x, by, WHITE);
            }
            display.display();
            delay(16);
        }
        // "CORIOLIS" slides in from the right (size-2 = 96 px wide)
        display.setTextColor(WHITE);
        display.setTextSize(2);
        for (int f = 0; f <= 18; f++) {
            int x = 128 - (f * 112) / 18;
            display.clearDisplay();
            display.setCursor(x, 24);
            display.print("CORIOLIS");
            display.display();
            delay(18);
        }
        // underline wipe, then hold
        for (int f = 0; f <= 14; f++) {
            display.clearDisplay();
            display.setCursor(16, 24);
            display.print("CORIOLIS");
            display.drawLine(16, 44, 16 + (f*96)/14, 44, WHITE);
            display.display();
            delay(16);
        }
        delay(450);
    }

    void clear() override { if (powered) display.clearDisplay(); }

    void drawNode(int nodeId, int batteryPercent, int signalDbm,
                  float windSpeedMph, int windDir, bool gpsFix) override {
        if (!powered) return;
        display.clearDisplay();
        display.setTextColor(WHITE);

        // --- top bar: battery + node + signal ---
        display.drawRect(5, 5, 20, 10, WHITE);                                 // outline
        display.fillRect(25, 8, 2, 4, WHITE);                                  // tip
        display.fillRect(7, 7, map(batteryPercent, 0, 100, 0, 15), 6, WHITE);  // level

        display.setTextSize(1);
        display.setCursor(35, 5);
        display.printf("%d%%", batteryPercent);

        display.setCursor(75, 5);            // where "IMU:" used to live
        display.printf("Node:%d", nodeId);

        // --- wind direction (big), right side ---
        display.setCursor(54, 20);
        display.setTextSize(1);
        display.print("Dir:");
        display.setTextSize(2);
        display.setCursor(90, 15);
        display.printf("%d", windDir);

        // --- 2 s avg wind speed (big), left side ---
        display.setTextSize(2);
        display.setCursor(54, 35);
        display.printf("%.1f", windSpeedMph);
        display.setTextSize(1);
        display.setCursor(90, 40);
        display.print("MPH");

        // --- footer: signal + gps ---
        display.setTextSize(1);
        display.setCursor(5, 55);
        if (signalDbm > -999) display.printf("%ddBm", signalDbm);
        else                  display.print("---dBm");
        display.setCursor(95, 55);
        display.print(gpsFix ? "GPS" : "no fix");

        display.display();
    }

    void drawNodeId(int nodeId) override {
        if (!powered) return;
        display.clearDisplay(); display.setTextColor(WHITE);
        display.setTextSize(1); display.setCursor(0, 2);  display.print("SET NODE ID");
        display.setTextSize(4); display.setCursor(48, 22); display.printf("%d", nodeId);
        display.setTextSize(1); display.setCursor(0, 56);
        display.print("short:+1  long:save");
        display.display();
    }

    void powerOn() override {
        if (powered) return;
        display.ssd1306_command(SSD1306_DISPLAYON);
        powered = true;
    }
    void powerOff() override {
        if (!powered) return;
        display.clearDisplay(); display.display();
        display.ssd1306_command(SSD1306_DISPLAYOFF);
        powered = false;
    }
    bool isOn() override { return powered; }
};

// ----------------------------------------------------------------------------
// SH1106 (U8g2)
// ----------------------------------------------------------------------------
class SH1106Screen : public Screen {
private:
    U8G2_SH1106_128X64_NONAME_F_HW_I2C display;
    bool powered;
public:
    SH1106Screen() : display(U8G2_R0, U8X8_PIN_NONE), powered(false) {}

    void begin() override {
        display.begin();
        powered = true;
        windIntro();                  // wind gust -> "CORIOLIS" startup flourish
        display.clearBuffer();
        display.setFont(u8g2_font_logisoso16_tf);
        display.setCursor(0, 18);
        display.print("Wind Node");
        display.sendBuffer();
        Serial.println(F("[Screen] SH1106 ready"));
    }

    // Wind gust sweeps across, then "CORIOLIS" rides in and settles.
    void windIntro() {
        const int FR = 32;
        const int N  = 10;
        int y0[N], sp[N], x0[N];
        for (int i = 0; i < N; i++) { y0[i] = 6 + i*6; sp[i] = 6 + (i%3); x0[i] = -(i*10); }
        for (int f = 0; f < FR; f++) {
            display.clearBuffer();
            for (int i = 0; i < N; i++) {
                int x = x0[i] + f*sp[i];
                if (x < -8 || x > 136) continue;
                int by  = y0[i] + (x > 0 ? (x*x)/2600 : 0);
                int byp = y0[i] + ((x-8) > 0 ? ((x-8)*(x-8))/2600 : 0);
                display.drawLine(x-8, byp, x, by);
                display.drawPixel(x, by);
            }
            display.sendBuffer();
            delay(16);
        }
        // "CORIOLIS" slides in from the right (9x15 = 72 px wide -> center x=28)
        display.setFont(u8g2_font_9x15B_tr);
        for (int f = 0; f <= 18; f++) {
            int x = 128 - (f * 100) / 18;
            display.clearBuffer();
            display.setCursor(x, 38);
            display.print("CORIOLIS");
            display.sendBuffer();
            delay(18);
        }
        for (int f = 0; f <= 14; f++) {
            display.clearBuffer();
            display.setCursor(28, 38);
            display.print("CORIOLIS");
            display.drawLine(28, 43, 28 + (f*72)/14, 43);
            display.sendBuffer();
            delay(16);
        }
        delay(450);
    }

    void clear() override { if (powered) display.clearBuffer(); }

    void drawNode(int nodeId, int batteryPercent, int signalDbm,
                  float windSpeedMph, int windDir, bool gpsFix) override {
        if (!powered) return;
        display.clearBuffer();

        // top bar
        display.drawFrame(5, 5, 20, 10);
        display.drawBox(25, 8, 2, 4);
        display.drawBox(7, 7, map(batteryPercent, 0, 100, 0, 15), 6);

        display.setFont(u8g2_font_6x10_tr);
        display.setCursor(35, 14);  display.printf("%d%%", batteryPercent);
        display.setCursor(75, 14);  display.printf("Node:%d", nodeId);

        // direction
        display.setCursor(54, 30);  display.print("Dir:");
        display.setFont(u8g2_font_logisoso16_tf);
        display.setCursor(86, 34);  display.printf("%d", windDir);

        // 2 s avg speed
        display.setFont(u8g2_font_logisoso16_tf);
        display.setCursor(2, 52);   display.printf("%.1f", windSpeedMph);
        display.setFont(u8g2_font_6x10_tr);
        display.setCursor(86, 52);  display.print("MPH");

        // footer
        display.setCursor(2, 62);
        if (signalDbm > -999) display.printf("%ddBm", signalDbm);
        else                  display.print("---dBm");
        display.setCursor(95, 62);  display.print(gpsFix ? "GPS" : "----");

        display.sendBuffer();
    }

    void drawNodeId(int nodeId) override {
        if (!powered) return;
        display.clearBuffer();
        display.setFont(u8g2_font_6x10_tr);
        display.setCursor(0, 10); display.print("SET NODE ID");
        display.setFont(u8g2_font_logisoso32_tn);
        display.setCursor(50, 46); display.printf("%d", nodeId);
        display.setFont(u8g2_font_6x10_tr);
        display.setCursor(0, 62); display.print("short:+1  long:save");
        display.sendBuffer();
    }

    void powerOn()  override { if (!powered) { display.setPowerSave(0); powered = true; } }
    void powerOff() override { if (powered)  { display.clearBuffer(); display.sendBuffer();
                                               display.setPowerSave(1); powered = false; } }
    bool isOn() override { return powered; }
};

// ----------------------------------------------------------------------------
// Factory: probe 0x3C, default SSD1306, fall back to SH1106
// Assumes Wire.begin() already called in main.cpp.
// ----------------------------------------------------------------------------
class ScreenFactory {
public:
    static Screen* create() {
        Wire.beginTransmission(OLED_I2C_ADDRESS);
        if (Wire.endTransmission() == 0) {
            Serial.println(F("[Screen] 0x3C present -> SSD1306"));
            return new SSD1306Screen();
        }
        Serial.println(F("[Screen] 0x3C absent  -> SH1106 fallback"));
        return new SH1106Screen();
    }
};

#endif // SCREEN_H
