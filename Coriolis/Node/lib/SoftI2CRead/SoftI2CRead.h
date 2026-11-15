#ifndef SOFT_I2C_READ_H
#define SOFT_I2C_READ_H

/**
 * Bit-banged I2C read with a hard time limit, for the ATtiny wind sensor
 * ======================================================================
 * Why: on this Arduino core (2.0.x, ESP-IDF 4.4 legacy I2C driver) a
 * transaction that never raises a "done" interrupt is only abandoned after the
 * driver's fixed 1000 ms "alive" check, whatever Wire.setTimeOut() says. Bench
 * 2026-11-15: every failed ATtiny read ("Error 263") froze the node's loop for
 * 1006-1007 ms, which made it transmit up to 1 s past its TDMA slot.
 *
 * This reads the ATtiny with the I2C peripheral released (Wire.end() before,
 * Wire.begin() after) and gives up within SI2C_STRETCH_US of any clock stretch
 * that doesn't end, so a bad read costs a few ms instead of a second. It
 * supports clock stretching (the ATtiny's USI holds SCL low after each byte
 * until its interrupt runs) and ends a failed read by clocking the bus free.
 *
 * Plain C++ with the pin access in macros: the host test (tests/firmware)
 * drives it against a simulated ATtiny.
 */

#include <stdint.h>
#include <stddef.h>

#ifndef SI2C_HOST_TEST
#include <Arduino.h>
#include "driver/gpio.h"
#define SI2C_SET(pin, lvl)  gpio_set_level((gpio_num_t)(pin), (lvl))   // 1 = release (open drain)
#define SI2C_GET(pin)       gpio_get_level((gpio_num_t)(pin))
#define SI2C_DELAY_US(us)   delayMicroseconds(us)
#define SI2C_MICROS()       ((uint32_t)micros())
// Take both pins as open-drain GPIOs with pull-ups, released (high). This
// also routes them back to plain GPIO output in the pin matrix after Wire.end().
static inline void si2cTakePins(int sda, int scl) {
    gpio_set_level((gpio_num_t)sda, 1);
    gpio_set_level((gpio_num_t)scl, 1);
    gpio_config_t c = {};
    c.pin_bit_mask = (1ULL << sda) | (1ULL << scl);
    c.mode = GPIO_MODE_INPUT_OUTPUT_OD;
    c.pull_up_en = GPIO_PULLUP_ENABLE;
    c.pull_down_en = GPIO_PULLDOWN_DISABLE;
    c.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&c);
}
#endif

#ifndef SI2C_HALF_US
#define SI2C_HALF_US     5       // half bit time: ~100 kHz
#endif
#ifndef SI2C_STRETCH_US
#define SI2C_STRETCH_US  3000    // longest clock stretch we wait out
#endif

enum SoftI2CResult : uint8_t {
    SI2C_OK = 0,
    SI2C_BUS_BUSY,       // SDA or SCL low before START (and couldn't be freed)
    SI2C_NACK,           // nobody answered the address
    SI2C_STRETCH,        // SCL held low longer than SI2C_STRETCH_US
};

class SoftI2CRead {
    int sda, scl;

    bool sclHigh() {                       // release SCL, wait out any stretch
        SI2C_SET(scl, 1);
        uint32_t t0 = SI2C_MICROS();
        while (!SI2C_GET(scl)) {
            if ((uint32_t)(SI2C_MICROS() - t0) > SI2C_STRETCH_US) return false;
            SI2C_DELAY_US(1);
        }
        return true;
    }
    void sclLow() { SI2C_SET(scl, 0); }

    // One clock with SDA set by us (bit) or released (bit = 1); returns SDA as read.
    bool clock(int bit, int& in) {
        SI2C_SET(sda, bit ? 1 : 0);
        SI2C_DELAY_US(SI2C_HALF_US);
        if (!sclHigh()) return false;
        SI2C_DELAY_US(SI2C_HALF_US);
        in = SI2C_GET(sda);
        sclLow();
        return true;
    }

public:
    SoftI2CRead(int sdaPin, int sclPin) : sda(sdaPin), scl(sclPin) {}

    // Free a slave left mid-transfer: clock (up to 9 times, SDA released)
    // until SDA reads high while SCL is high, then send START + STOP right
    // there, without moving SCL, which resets every slave's state machine.
    // (A STOP after one more SCL low could be masked by the slave driving its
    // next bit.) Leaves both lines released.
    void clearBus() {
        SI2C_SET(sda, 1);
        for (int i = 0; i < 10; i++) {
            if (!sclHigh()) break;
            SI2C_DELAY_US(SI2C_HALF_US);
            if (SI2C_GET(sda)) {
                SI2C_SET(sda, 0); SI2C_DELAY_US(SI2C_HALF_US);   // START
                SI2C_SET(sda, 1); SI2C_DELAY_US(SI2C_HALF_US);   // STOP
                return;
            }
            sclLow(); SI2C_DELAY_US(SI2C_HALF_US);
        }
        SI2C_SET(scl, 1);
    }

    void stop() {
        sclLow();
        SI2C_SET(sda, 0);
        SI2C_DELAY_US(SI2C_HALF_US);
        if (!sclHigh()) { SI2C_SET(sda, 1); return; }
        SI2C_DELAY_US(SI2C_HALF_US);
        SI2C_SET(sda, 1);
        SI2C_DELAY_US(SI2C_HALF_US);
    }

    // SCL is being held low: nothing can be clocked, so just let go of both
    // lines and return at once (one SI2C_STRETCH_US wait in total).
    SoftI2CResult giveUp() {
        SI2C_SET(sda, 1);
        SI2C_SET(scl, 1);
        return SI2C_STRETCH;
    }

    // Read n bytes from 7-bit address addr. Pins must already be taken.
    SoftI2CResult read(uint8_t addr, uint8_t* buf, size_t n) {
        SI2C_SET(sda, 1);
        if (!sclHigh() || !SI2C_GET(sda)) {
            clearBus();
            if (!SI2C_GET(scl) || !SI2C_GET(sda)) return SI2C_BUS_BUSY;
        }
        // START: SDA falls while SCL is high
        SI2C_SET(sda, 0);
        SI2C_DELAY_US(SI2C_HALF_US);
        sclLow();

        uint8_t a = (uint8_t)((addr << 1) | 1);
        int in = 1;
        for (int b = 7; b >= 0; b--)
            if (!clock((a >> b) & 1, in)) return giveUp();
        if (!clock(1, in)) return giveUp();     // ACK slot
        if (in) { stop(); return SI2C_NACK; }

        for (size_t i = 0; i < n; i++) {
            uint8_t v = 0;
            for (int b = 0; b < 8; b++) {
                if (!clock(1, in)) return giveUp();
                v = (uint8_t)((v << 1) | (in ? 1 : 0));
            }
            buf[i] = v;
            // ACK every byte but the last; NACK the last so the slave lets go
            if (!clock(i + 1 < n ? 0 : 1, in)) return giveUp();
        }
        stop();
        return SI2C_OK;
    }
};

#endif // SOFT_I2C_READ_H
