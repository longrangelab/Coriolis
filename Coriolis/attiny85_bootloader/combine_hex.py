#!/usr/bin/env python3
"""
Combine the bootloader and a sketch into one .hex for the one-time ISP step,
so a chip comes off the programmer with both and starts the sketch on its own.

    python combine_hex.py twiboot.hex attiny85_wind_sensor.ino.hex combined.hex

Writes the vector table exactly as twiboot itself would after an I2C upload:
reset -> bootloader (0x1C00), EE_RDY -> the sketch's start.
"""
import sys

BOOT_START = 0x1C00
EE_RDY = 6


def read_hex(path):
    mem = {}
    with open(path) as f:
        for raw in f:
            line = raw.strip()
            if not line:
                continue
            d = bytes.fromhex(line[1:])
            if sum(d) & 0xFF or len(d) != d[0] + 5:
                sys.exit(f"{path}: bad record {line}")
            addr, kind = (d[1] << 8) | d[2], d[3]
            if kind == 0:
                for i, b in enumerate(d[4:-1]):
                    mem[addr + i] = b
            elif kind == 1:
                break
            elif kind in (2, 4) and (d[4] or d[5]):
                sys.exit(f"{path}: data above 64 KB")
    return mem


def rjmp(from_word, to_byte):
    return 0xC000 | (((to_byte // 2) - from_word - 1) & 0x0FFF)


def rjmp_target(from_word, op):
    k = op & 0x0FFF
    if k & 0x800:
        k -= 0x1000
    return ((from_word + 1 + k) & 0x0FFF) * 2


def write_hex(mem, path):
    with open(path, "w") as f:
        addrs = sorted(mem)
        i = 0
        while i < len(addrs):
            start = addrs[i]
            chunk = [mem[start]]
            i += 1
            while i < len(addrs) and addrs[i] == start + len(chunk) and len(chunk) < 16:
                chunk.append(mem[addrs[i]])
                i += 1
            rec = [len(chunk), start >> 8, start & 0xFF, 0] + chunk
            rec.append((-sum(rec)) & 0xFF)
            f.write(":" + "".join(f"{b:02X}" for b in rec) + "\n")
        f.write(":00000001FF\n")


def main():
    if len(sys.argv) != 4:
        sys.exit(__doc__)
    boot, app = read_hex(sys.argv[1]), read_hex(sys.argv[2])
    if min(boot) < BOOT_START:
        sys.exit("bootloader hex does not start at 0x1C00")
    if max(app) >= BOOT_START:
        sys.exit(f"sketch is {max(app) + 1} bytes; the limit is {BOOT_START}")

    word = lambda m, w: m.get(2 * w, 0xFF) | (m.get(2 * w + 1, 0xFF) << 8)
    start = rjmp_target(0, word(app, 0))
    ee = rjmp_target(EE_RDY, word(app, EE_RDY))
    others = [rjmp_target(v, word(app, v)) for v in range(1, 15) if v != EE_RDY]
    if ee not in others:
        sys.exit("sketch uses the EE_RDY interrupt, which the bootloader needs")

    mem = dict(app)
    for w, op in ((0, rjmp(0, BOOT_START)), (EE_RDY, rjmp(EE_RDY, start))):
        mem[2 * w], mem[2 * w + 1] = op & 0xFF, op >> 8
    mem.update(boot)
    write_hex(mem, sys.argv[3])
    print(f"{sys.argv[3]}: sketch {max(app) + 1} bytes (starts at 0x{start:04X}) + bootloader at 0x{BOOT_START:04X}")


if __name__ == "__main__":
    main()
