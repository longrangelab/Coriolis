#ifndef SCREEN_H
#define SCREEN_H

/**
 * Receiver display (128x64, SSD1306 or SH1106)  --  imperial units
 * ================================================================
 * GENERAL : avg wind speed/dir + avg temperature (F) + avg pressure (inHg).
 * NODE    : per-node wind, temp (F), pressure (inHg), battery, distance, age.
 */

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <U8g2lib.h>

#define SCREEN_WIDTH      128
#define SCREEN_HEIGHT      64
#define OLED_I2C_ADDRESS 0x3C
#define OLED_RESET         -1

static const char* cardinal(int deg) {
    static const char* n[] = {"N","NE","E","SE","S","SW","W","NW"};
    int i = ((deg + 22) / 45) % 8; if (i < 0) i += 8; return n[i];
}

class Screen {
public:
    virtual void begin() = 0;
    virtual void clear() = 0;
    virtual void drawGeneral(int count, float avgSpeed, int avgDir,
                             float avgTempF, float avgPresInHg,
                             int rxBatt, float rxBattV, bool rxGps) = 0;
    virtual void drawNode(int nodeId, float speed, float speedSd, int dir, int dirSd,
                          float tempF, float presInHg, int battPct, float battV,
                          long distYards, float rssi, int ageSec) = 0;
};

// ----------------------------------------------------------------------------
// SSD1306
// ----------------------------------------------------------------------------
class SSD1306Screen : public Screen {
private:
    Adafruit_SSD1306 d;
public:
    SSD1306Screen() : d(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET) {}

    void begin() override {
        if (!d.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDRESS)) {
            Serial.println(F("[Screen] SSD1306 init FAILED")); return;
        }
        Serial.println(F("[Screen] SSD1306 ready"));
        windIntro();                  // wind gust -> "CORIOLIS" startup flourish
        d.clearDisplay(); d.setTextColor(WHITE);
        d.setTextSize(2); d.setCursor(0, 8);  d.print("Wind Rx");
        d.setTextSize(1); d.setCursor(0, 40); d.print("listening...");
        d.display();
    }

    // Wind gust sweeps across, then "CORIOLIS" rides in and settles.
    void windIntro() {
        const int FR = 32;
        const int N  = 10;
        int y0[N], sp[N], x0[N];
        for (int i = 0; i < N; i++) { y0[i] = 4 + i*6; sp[i] = 6 + (i%3); x0[i] = -(i*10); }
        for (int f = 0; f < FR; f++) {
            d.clearDisplay();
            for (int i = 0; i < N; i++) {
                int x = x0[i] + f*sp[i];
                if (x < -8 || x > 136) continue;
                int by  = y0[i] + (x > 0 ? (x*x)/2600 : 0);       // Coriolis curl (down-right)
                int byp = y0[i] + ((x-8) > 0 ? ((x-8)*(x-8))/2600 : 0);
                d.drawLine(x-8, byp, x, by, WHITE);
                d.drawPixel(x, by, WHITE);
            }
            d.display();
            delay(16);
        }
        d.setTextColor(WHITE);
        d.setTextSize(2);
        for (int f = 0; f <= 18; f++) {
            int x = 128 - (f * 112) / 18;
            d.clearDisplay();
            d.setCursor(x, 24);
            d.print("CORIOLIS");
            d.display();
            delay(18);
        }
        for (int f = 0; f <= 14; f++) {
            d.clearDisplay();
            d.setCursor(16, 24);
            d.print("CORIOLIS");
            d.drawLine(16, 44, 16 + (f*96)/14, 44, WHITE);
            d.display();
            delay(16);
        }
        delay(450);
    }
    void clear() override { d.clearDisplay(); }

    void drawGeneral(int count, float avgSpeed, int avgDir, float avgTempF,
                     float avgPresInHg, int rxBatt, float rxBattV, bool rxGps) override {
        d.clearDisplay(); d.setTextColor(WHITE);
        d.setTextSize(1); d.setCursor(0, 0); d.printf("ALL NODES (%d)", count);
        if (count <= 0) {
            d.setTextSize(2); d.setCursor(0, 26); d.print("no nodes");
        } else {
            d.setTextSize(2); d.setCursor(0, 12); d.printf("%.1f", avgSpeed);
            d.setTextSize(1); d.setCursor(60, 12); d.print("MPH");
            d.setCursor(60, 22); d.printf("%d%s", avgDir, cardinal(avgDir));
            d.setCursor(0, 34);  d.printf("Air %.0fF  %.2finHg", avgTempF, avgPresInHg);
        }
        d.setCursor(0, 48); d.printf("Rx %d%%  %.2fV", rxBatt, rxBattV);
        d.setCursor(0, 57); d.printf("Base GPS: %s", rxGps ? "ok" : "-- (no fix)");
        d.display();
    }

    void drawNode(int nodeId, float speed, float speedSd, int dir, int dirSd,
                  float tempF, float presInHg, int battPct, float battV,
                  long distYards, float rssi, int ageSec) override {
        d.clearDisplay(); d.setTextColor(WHITE);
        d.setTextSize(1);
        d.setCursor(0, 0);  d.printf("Node %d", nodeId);
        d.setCursor(56, 0); d.printf("%ddBm", (int)rssi);
        d.setCursor(104,0); d.printf("%ds", ageSec);

        d.setTextSize(2); d.setCursor(0, 11); d.printf("%.1f", speed);
        d.setTextSize(1); d.setCursor(60, 11); d.printf("MPH s%.1f", speedSd);

        d.setCursor(0, 30); d.printf("Dir %d%s sd%d", dir, cardinal(dir), dirSd);
        d.setCursor(0, 39); d.printf("%.0fF  %.2finHg", tempF, presInHg);
        d.setCursor(0, 48);
        if (battV > 0) d.printf("Bat %.2fV %d%%", battV, battPct);
        else           d.print("Bat --");
        d.setCursor(0, 57);
        if (distYards >= 0)       d.printf("Dist %ld yd", distYards);
        else if (distYards == -1) d.print("Dist -- rx no GPS");
        else                      d.print("Dist -- node noGPS");
        d.display();
    }
};

// ----------------------------------------------------------------------------
// SH1106 (U8g2)
// ----------------------------------------------------------------------------
class SH1106Screen : public Screen {
private:
    U8G2_SH1106_128X64_NONAME_F_HW_I2C d;
public:
    SH1106Screen() : d(U8G2_R0, U8X8_PIN_NONE) {}

    void begin() override {
        d.begin();
        windIntro();                  // wind gust -> "CORIOLIS" startup flourish
        d.clearBuffer();
        d.setFont(u8g2_font_logisoso16_tf); d.setCursor(0, 18); d.print("Wind Rx");
        d.sendBuffer(); Serial.println(F("[Screen] SH1106 ready"));
    }

    // Wind gust sweeps across, then "CORIOLIS" rides in and settles.
    void windIntro() {
        const int FR = 32;
        const int N  = 10;
        int y0[N], sp[N], x0[N];
        for (int i = 0; i < N; i++) { y0[i] = 6 + i*6; sp[i] = 6 + (i%3); x0[i] = -(i*10); }
        for (int f = 0; f < FR; f++) {
            d.clearBuffer();
            for (int i = 0; i < N; i++) {
                int x = x0[i] + f*sp[i];
                if (x < -8 || x > 136) continue;
                int by  = y0[i] + (x > 0 ? (x*x)/2600 : 0);
                int byp = y0[i] + ((x-8) > 0 ? ((x-8)*(x-8))/2600 : 0);
                d.drawLine(x-8, byp, x, by);
                d.drawPixel(x, by);
            }
            d.sendBuffer();
            delay(16);
        }
        d.setFont(u8g2_font_9x15B_tr);
        for (int f = 0; f <= 18; f++) {
            int x = 128 - (f * 100) / 18;
            d.clearBuffer();
            d.setCursor(x, 38);
            d.print("CORIOLIS");
            d.sendBuffer();
            delay(18);
        }
        for (int f = 0; f <= 14; f++) {
            d.clearBuffer();
            d.setCursor(28, 38);
            d.print("CORIOLIS");
            d.drawLine(28, 43, 28 + (f*72)/14, 43);
            d.sendBuffer();
            delay(16);
        }
        delay(450);
    }
    void clear() override { d.clearBuffer(); }

    void drawGeneral(int count, float avgSpeed, int avgDir, float avgTempF,
                     float avgPresInHg, int rxBatt, float rxBattV, bool rxGps) override {
        d.clearBuffer(); d.setFont(u8g2_font_6x10_tr);
        d.setCursor(0, 9); d.printf("ALL NODES (%d)", count);
        if (count <= 0) {
            d.setFont(u8g2_font_logisoso16_tf); d.setCursor(0, 40); d.print("no nodes");
        } else {
            d.setFont(u8g2_font_logisoso16_tf); d.setCursor(0, 30); d.printf("%.1f", avgSpeed);
            d.setFont(u8g2_font_6x10_tr);
            d.setCursor(64, 22); d.print("MPH");
            d.setCursor(64, 32); d.printf("%d%s", avgDir, cardinal(avgDir));
            d.setCursor(0, 44);  d.printf("Air %.0fF %.2finHg", avgTempF, avgPresInHg);
        }
        d.setCursor(0, 55); d.printf("Rx %d%% %.2fV", rxBatt, rxBattV);
        d.setCursor(0, 64); d.printf("Base GPS:%s", rxGps ? "ok" : "--");
        d.sendBuffer();
    }

    void drawNode(int nodeId, float speed, float speedSd, int dir, int dirSd,
                  float tempF, float presInHg, int battPct, float battV,
                  long distYards, float rssi, int ageSec) override {
        d.clearBuffer(); d.setFont(u8g2_font_6x10_tr);
        d.setCursor(0, 9);   d.printf("Node %d", nodeId);
        d.setCursor(56, 9);  d.printf("%ddBm", (int)rssi);
        d.setCursor(104, 9); d.printf("%ds", ageSec);

        d.setFont(u8g2_font_logisoso16_tf); d.setCursor(0, 30); d.printf("%.1f", speed);
        d.setFont(u8g2_font_6x10_tr); d.setCursor(64, 24); d.printf("MPH s%.1f", speedSd);

        d.setCursor(0, 40); d.printf("Dir %d%s sd%d", dir, cardinal(dir), dirSd);
        d.setCursor(0, 50); d.printf("%.0fF %.2finHg", tempF, presInHg);
        d.setCursor(0, 60);
        if (battV > 0) d.printf("%.2fV %d%%", battV, battPct);
        else           d.print("Bat --");
        d.setCursor(66, 60);
        if (distYards >= 0)       d.printf("%ldyd", distYards);
        else if (distYards == -1) d.print("rx noGPS");
        else                      d.print("nd noGPS");
        d.sendBuffer();
    }
};

class ScreenFactory {
public:
    static Screen* create() {
        Wire.beginTransmission(OLED_I2C_ADDRESS);
        if (Wire.endTransmission() == 0) { Serial.println(F("[Screen] SSD1306")); return new SSD1306Screen(); }
        Serial.println(F("[Screen] SH1106 fallback")); return new SH1106Screen();
    }
};

#endif // SCREEN_H
