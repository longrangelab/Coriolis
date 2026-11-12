#pragma once
/**
 * AttinyFlasher -- update the ATtiny85 wind sensor firmware over I2C.
 * =====================================================================
 * The ATtiny85 carries the twiboot I2C bootloader (Coriolis/attiny85_bootloader)
 * in its top 1 KB. This file is the master side of that bootloader's protocol:
 *
 *   1. ask the running sketch (0x42) to reboot into the bootloader (0x43),
 *   2. check the chip,
 *   3. write the new image page by page and read every page back,
 *   4. start the new sketch.
 *
 * Interrupted updates are safe. Page 0 (the reset vector) is first written
 * pointing back at the bootloader and only gets its real contents after every
 * other page has been written and verified. Until then a power cycle leaves
 * the ATtiny sitting in the bootloader at 0x43, so the update can simply be
 * run again. No ISP programmer needed.
 *
 * The core (AttinyFlasher<Bus>, parseIntelHexLine) is plain C++ so it is
 * tested off-target against a simulated ATtiny85. The Arduino parts (WireBus,
 * attinySerialUpdate) are at the bottom.
 *
 * Bus must provide:
 *   bool write(uint8_t addr, const uint8_t* d, uint16_t n);   // START..STOP, true if all ACKed
 *   bool writeRead(uint8_t addr, const uint8_t* w, uint16_t wn,
 *                  uint8_t* r, uint16_t rn);                   // repeated START between
 *   void delayMs(uint32_t ms);
 */

#include <stdint.h>
#include <string.h>

namespace attiny {

constexpr uint8_t  APP_ADDR      = 0x42;    // the wind sensor sketch
constexpr uint8_t  BOOT_ADDR     = 0x43;    // twiboot (Coriolis build)
constexpr uint16_t PAGE_SIZE     = 64;      // ATtiny85 flash page
constexpr uint16_t BOOT_START    = 0x1C00;  // bytes; app may use 0..BOOT_START-1
constexpr uint8_t  EE_RDY_VECTOR = 6;       // vector twiboot borrows for the app entry

// twiboot protocol bytes
constexpr uint8_t CMD_WAIT       = 0x00;    // abort the 1 s boot timeout
constexpr uint8_t CMD_SWITCH_APP = 0x01;
constexpr uint8_t BOOTTYPE_BOOTLOADER  = 0x00;
constexpr uint8_t BOOTTYPE_APPLICATION = 0x80;
constexpr uint8_t CMD_ACCESS_MEM = 0x02;
constexpr uint8_t MEMTYPE_CHIPINFO = 0x00;
constexpr uint8_t MEMTYPE_FLASH    = 0x01;

enum Result : uint8_t {
    OK = 0,
    NO_BOOTLOADER,      // nothing answered at 0x43 (bootloader not installed?)
    WRONG_CHIP,         // signature or page size is not an ATtiny85 + twiboot
    IMAGE_EMPTY,
    IMAGE_TOO_BIG,      // sketch overlaps the bootloader (> 7168 bytes)
    IMAGE_USES_EE_RDY,  // sketch has an EE_RDY interrupt, which twiboot needs
    WRITE_FAILED,
    VERIFY_FAILED,
};

inline const char* resultText(Result r) {
    switch (r) {
        case OK:                return "ok";
        case NO_BOOTLOADER:     return "no bootloader answered at 0x43";
        case WRONG_CHIP:        return "chip is not an ATtiny85 with the Coriolis twiboot";
        case IMAGE_EMPTY:       return "image is empty";
        case IMAGE_TOO_BIG:     return "image is larger than 7168 bytes";
        case IMAGE_USES_EE_RDY: return "image uses the EE_RDY interrupt";
        case WRITE_FAILED:      return "page write was not acknowledged";
        case VERIFY_FAILED:     return "read-back did not match";
    }
    return "?";
}

// RJMP opcode at word address `from` that lands on byte address `toByte`.
inline uint16_t rjmpOpcode(uint16_t fromWord, uint16_t toByte) {
    return 0xC000 | ((uint16_t)((toByte / 2) - fromWord - 1) & 0x0FFF);
}

// Byte address an RJMP at word `fromWord` jumps to (8 KB wrap-around).
inline uint16_t rjmpTarget(uint16_t fromWord, uint16_t opcode) {
    int16_t k = opcode & 0x0FFF;
    if (k & 0x0800) k -= 0x1000;
    return (uint16_t)(((fromWord + 1 + k) & 0x0FFF) * 2);
}

template <class Bus>
class Flasher {
public:
    explicit Flasher(Bus& bus) : bus_(bus) {}

    // Get the ATtiny into its bootloader and stop the 1 s boot timeout.
    // Works whether the sketch is running, the bootloader is already active,
    // or a previous update was interrupted. Returns true when 0x43 answers.
    bool enterBootloader(uint32_t waitMs = 3000) {
        if (holdBootloader()) return true;
        const uint8_t req[2] = { CMD_SWITCH_APP, BOOTTYPE_BOOTLOADER };
        bus_.write(APP_ADDR, req, 2);           // sketch watchdog-resets itself
        for (uint32_t t = 0; t < waitMs; t += 5) {
            bus_.delayMs(5);
            if (holdBootloader()) return true;
        }
        return false;
    }

    // 8 bytes: signature[3], page size, app flash size (hi, lo), eeprom size (hi, lo)
    bool readChipInfo(uint8_t info[8]) {
        const uint8_t cmd[4] = { CMD_ACCESS_MEM, MEMTYPE_CHIPINFO, 0, 0 };
        return bus_.writeRead(BOOT_ADDR, cmd, 4, info, 8);
    }

    // Write `len` bytes of sketch (from address 0) and verify. The bootloader
    // must already be active (enterBootloader). On failure the ATtiny stays in
    // the bootloader and program() can simply be called again.
    Result program(const uint8_t* image, uint16_t len) {
        if (len == 0) return IMAGE_EMPTY;

        uint8_t info[8];
        if (!readChipInfo(info)) return NO_BOOTLOADER;
        if (info[0] != 0x1E || info[1] != 0x93 || info[2] != 0x0B || info[3] != PAGE_SIZE)
            return WRONG_CHIP;
        uint16_t appSize = ((uint16_t)info[4] << 8) | info[5];
        if (appSize != BOOT_START) return WRONG_CHIP;
        if (len > appSize) return IMAGE_TOO_BIG;
        if (usesEeRdyVector(image, len)) return IMAGE_USES_EE_RDY;

        const uint16_t pages = (len + PAGE_SIZE - 1) / PAGE_SIZE;
        uint8_t page[PAGE_SIZE];

        // 1) Page 0 with its reset vector pointing back at the bootloader.
        //    twiboot copies that jump into the app-entry vector, so until step 4
        //    "start the app" re-enters the bootloader instead of a half-written
        //    sketch.
        loadPage(image, len, 0, page);
        const uint16_t toBoot = rjmpOpcode(0, BOOT_START);
        page[0] = toBoot & 0xFF;
        page[1] = toBoot >> 8;
        if (!writePage(0, page)) return WRITE_FAILED;

        // 2) Every other page.
        for (uint16_t p = 1; p < pages; p++) {
            loadPage(image, len, p, page);
            if (!writePage(p, page)) return WRITE_FAILED;
        }

        // 3) Read them all back (one retry per page).
        for (uint16_t p = 1; p < pages; p++) {
            loadPage(image, len, p, page);
            if (verifyPage(p, page)) continue;
            if (!writePage(p, page) || !verifyPage(p, page)) return VERIFY_FAILED;
        }

        // 4) The real page 0. twiboot keeps the sketch's own reset/EE_RDY
        //    words in RAM and returns them on read-back, so it verifies
        //    against the unmodified image.
        loadPage(image, len, 0, page);
        if (!writePage(0, page)) return WRITE_FAILED;
        if (!verifyPage(0, page)) {
            if (!writePage(0, page) || !verifyPage(0, page)) return VERIFY_FAILED;
        }
        return OK;
    }

    // Compare the ATtiny's flash with `image` without writing anything.
    // Bootloader must be active (enterBootloader).
    bool matches(const uint8_t* image, uint16_t len) {
        uint8_t page[PAGE_SIZE];
        const uint16_t pages = (len + PAGE_SIZE - 1) / PAGE_SIZE;
        for (uint16_t p = 0; p < pages; p++) {
            loadPage(image, len, p, page);
            if (p == 0) {
                // After a reset twiboot returns the vectors as stored, i.e.
                // patched: reset -> bootloader, EE_RDY -> sketch start.
                // Rebuild that view of the image and accept either.
                if (verifyPage(0, page)) continue;
                uint16_t sketchStart = rjmpTarget(0, page[0] | (page[1] << 8));
                uint16_t rst = rjmpOpcode(0, BOOT_START);
                uint16_t app = rjmpOpcode(EE_RDY_VECTOR, sketchStart);
                page[0] = rst & 0xFF; page[1] = rst >> 8;
                page[2 * EE_RDY_VECTOR] = app & 0xFF; page[2 * EE_RDY_VECTOR + 1] = app >> 8;
            }
            if (!verifyPage(p, page)) return false;
        }
        return true;
    }

    // Leave the bootloader and run the sketch (it takes ~2 s to answer at 0x42).
    bool startApp() {
        const uint8_t cmd[2] = { CMD_SWITCH_APP, BOOTTYPE_APPLICATION };
        return bus_.write(BOOT_ADDR, cmd, 2);
    }

    // The whole job: enter, program, start. Starts the sketch only on success,
    // so a failed update stays in the bootloader and can be retried.
    Result update(const uint8_t* image, uint16_t len) {
        if (!enterBootloader()) return NO_BOOTLOADER;
        Result r = program(image, len);
        if (r == OK) startApp();
        return r;
    }

    // twiboot overwrites the EE_RDY vector with the jump to the sketch. Avr-gcc
    // points every unused vector at __bad_interrupt, so EE_RDY must jump to the
    // same place as most of the other vectors.
    static bool usesEeRdyVector(const uint8_t* image, uint16_t len) {
        const uint8_t VECTORS = 15;
        if (len < VECTORS * 2) return false;
        uint16_t target[VECTORS];
        for (uint8_t v = 0; v < VECTORS; v++)
            target[v] = rjmpTarget(v, image[2 * v] | (image[2 * v + 1] << 8));
        uint8_t same = 0;
        for (uint8_t v = 1; v < VECTORS; v++)
            if (v != EE_RDY_VECTOR && target[v] == target[EE_RDY_VECTOR]) same++;
        return same == 0;
    }

private:
    Bus& bus_;

    bool holdBootloader() {
        const uint8_t wait = CMD_WAIT;
        return bus_.write(BOOT_ADDR, &wait, 1);
    }

    static void loadPage(const uint8_t* image, uint16_t len, uint16_t p, uint8_t* page) {
        uint16_t base = p * PAGE_SIZE;
        for (uint16_t i = 0; i < PAGE_SIZE; i++)
            page[i] = (base + i < len) ? image[base + i] : 0xFF;
    }

    // twiboot stretches SCL for ~9 ms while it erases and writes the page,
    // inside the ESP32's 13 ms hardware limit.
    bool writePage(uint16_t p, const uint8_t* page) {
        uint8_t msg[4 + PAGE_SIZE];
        uint16_t addr = p * PAGE_SIZE;
        msg[0] = CMD_ACCESS_MEM; msg[1] = MEMTYPE_FLASH;
        msg[2] = addr >> 8;      msg[3] = addr & 0xFF;
        memcpy(msg + 4, page, PAGE_SIZE);
        for (uint8_t attempt = 0; attempt < 3; attempt++) {
            if (bus_.write(BOOT_ADDR, msg, sizeof(msg))) return true;
            bus_.delayMs(20);
        }
        return false;
    }

    bool verifyPage(uint16_t p, const uint8_t* page) {
        uint16_t addr = p * PAGE_SIZE;
        const uint8_t cmd[4] = { CMD_ACCESS_MEM, MEMTYPE_FLASH, (uint8_t)(addr >> 8), (uint8_t)(addr & 0xFF) };
        uint8_t got[PAGE_SIZE];
        if (!bus_.writeRead(BOOT_ADDR, cmd, 4, got, PAGE_SIZE)) return false;
        return memcmp(got, page, PAGE_SIZE) == 0;
    }
};

// ---------------------------------------------------------------------------
// Intel HEX (what Arduino IDE "Export Compiled Binary" writes)
// ---------------------------------------------------------------------------
enum HexLine : uint8_t { HEX_DATA, HEX_EOF, HEX_SKIP, HEX_BAD, HEX_RANGE };

// Parse one line into image[0..cap). Tracks the highest byte written in *used.
inline HexLine parseIntelHexLine(const char* s, uint8_t* image, uint16_t cap, uint16_t* used) {
    auto nib = [](char c) -> int {
        if (c >= '0' && c <= '9') return c - '0';
        if (c >= 'A' && c <= 'F') return c - 'A' + 10;
        if (c >= 'a' && c <= 'f') return c - 'a' + 10;
        return -1;
    };
    while (*s == ' ' || *s == '\t') s++;
    if (*s == 0 || *s == '\r' || *s == '\n') return HEX_SKIP;
    if (*s != ':') return HEX_BAD;
    s++;
    uint8_t b[5 + 255];
    uint16_t n = 0;
    while (nib(s[0]) >= 0 && nib(s[1]) >= 0 && n < sizeof(b)) {
        b[n++] = (uint8_t)((nib(s[0]) << 4) | nib(s[1]));
        s += 2;
    }
    if (n < 5 || n != (uint16_t)b[0] + 5) return HEX_BAD;
    uint8_t sum = 0;
    for (uint16_t i = 0; i < n; i++) sum += b[i];
    if (sum != 0) return HEX_BAD;

    uint8_t  count = b[0];
    uint16_t addr  = ((uint16_t)b[1] << 8) | b[2];
    switch (b[3]) {
        case 0x00:
            if ((uint32_t)addr + count > cap) return HEX_RANGE;
            memcpy(image + addr, b + 4, count);
            if (addr + count > *used) *used = addr + count;
            return HEX_DATA;
        case 0x01:
            return HEX_EOF;
        case 0x02: case 0x04:           // extended address: must stay 0 on a tiny
            return (b[4] == 0 && b[5] == 0) ? HEX_SKIP : HEX_RANGE;
        default:                        // 03/05 start address: not needed
            return HEX_SKIP;
    }
}

} // namespace attiny

// ===========================================================================
// ESP32 / Arduino side
// ===========================================================================
#ifdef ARDUINO
#include <Arduino.h>
#include <Wire.h>

namespace attiny {

struct WireBus {
    bool write(uint8_t addr, const uint8_t* d, uint16_t n) {
        Wire.beginTransmission(addr);
        Wire.write(d, n);
        return Wire.endTransmission() == 0;
    }
    bool writeRead(uint8_t addr, const uint8_t* w, uint16_t wn, uint8_t* r, uint16_t rn) {
        Wire.beginTransmission(addr);
        Wire.write(w, wn);
        if (Wire.endTransmission(false) != 0) return false;
        if (Wire.requestFrom(addr, (uint8_t)rn) != rn) return false;
        for (uint16_t i = 0; i < rn; i++) r[i] = Wire.read();
        return true;
    }
    void delayMs(uint32_t ms) { delay(ms); }
};

/**
 * USB-serial update, for the bench. Call from loop() with each complete line
 * the node receives on Serial. Returns true if the line was ours.
 *
 *   1. Arduino IDE: Sketch > Export Compiled Binary (ATtiny sketch).
 *   2. Send the line  attiny-flash  then the .hex file, e.g. with
 *      tools/attiny_i2c_flash.py, or paste it into a serial terminal.
 *   3. The node answers  [attiny] OK ...  or  [attiny] FAIL ...
 */
inline bool attinySerialUpdate(const String& line, Stream& io, uint32_t idleTimeoutMs = 10000) {
    if (line != "attiny-flash") return false;

    static uint8_t image[BOOT_START];
    memset(image, 0xFF, sizeof(image));
    uint16_t used = 0;
    io.println(F("[attiny] ready: send the Intel HEX file"));

    String rec;
    uint32_t last = millis();
    bool done = false;
    while (!done) {
        if (millis() - last > idleTimeoutMs) {
            io.println(F("[attiny] FAIL timed out waiting for HEX (nothing written)"));
            return true;
        }
        while (io.available()) {
            char c = (char)io.read();
            last = millis();
            if (c != '\n' && c != '\r') { if (rec.length() < 600) rec += c; continue; }
            if (rec.length() == 0) continue;
            HexLine h = parseIntelHexLine(rec.c_str(), image, sizeof(image), &used);
            rec = "";
            if (h == HEX_BAD)   { io.println(F("[attiny] FAIL bad HEX line (nothing written)")); return true; }
            if (h == HEX_RANGE) { io.println(F("[attiny] FAIL sketch is larger than 7168 bytes (nothing written)")); return true; }
            if (h == HEX_EOF)   { done = true; break; }
        }
        delay(1);
    }

    io.printf("[attiny] got %u bytes, entering bootloader...\n", used);
    WireBus bus;
    Wire.setTimeOut(200);                       // page writes stretch SCL ~9 ms
    Flasher<WireBus> f(bus);
    uint32_t t0 = millis();
    Result r = f.update(image, used);
    Wire.setTimeOut(50);
    if (r == OK)
        io.printf("[attiny] OK %u bytes written and verified in %lu ms, sketch starting\n",
                  used, (unsigned long)(millis() - t0));
    else
        io.printf("[attiny] FAIL %s%s\n", resultText(r),
                  r == NO_BOOTLOADER ? "" : " (ATtiny left in bootloader, safe to retry)");
    return true;
}

} // namespace attiny
#endif // ARDUINO
