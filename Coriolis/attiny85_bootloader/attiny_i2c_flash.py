#!/usr/bin/env python3
"""
Update the ATtiny85 wind sensor over I2C, through the wind meter node's USB port.

    python attiny_i2c_flash.py COM6 attiny85_wind_sensor.ino.hex

The .hex comes from Arduino IDE: Sketch > Export Compiled Binary (ATtiny85,
8 MHz internal). The node must run firmware with the "attiny-flash" command,
and the ATtiny must have the I2C bootloader (see README.md in this folder).

Needs pyserial:  python -m pip install pyserial
(PlatformIO's own Python already has it.)
"""
import argparse
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial is missing: python -m pip install pyserial")

APP_LIMIT = 0x1C00  # bytes below the bootloader


def check_hex(path):
    """Return (lines, size) after checking checksums and the size limit."""
    lines, top = [], 0
    with open(path, "r") as f:
        for n, raw in enumerate(f, 1):
            line = raw.strip()
            if not line:
                continue
            if not line.startswith(":"):
                sys.exit(f"{path}:{n}: not an Intel HEX line")
            data = bytes.fromhex(line[1:])
            if len(data) < 5 or len(data) != data[0] + 5 or sum(data) & 0xFF:
                sys.exit(f"{path}:{n}: bad record or checksum")
            if data[3] == 0x00:
                top = max(top, ((data[1] << 8) | data[2]) + data[0])
            lines.append(line)
            if data[3] == 0x01:
                break
    if top > APP_LIMIT:
        sys.exit(f"sketch is {top} bytes; with the bootloader the limit is {APP_LIMIT}")
    return lines, top


def wait_for(port, needles, timeout, echo=True):
    """Read lines until one contains any of `needles`; return that line."""
    end = time.time() + timeout
    while time.time() < end:
        line = port.readline().decode("utf-8", "replace").strip()
        if not line:
            continue
        if echo and line.startswith("[attiny]"):
            print("  node:", line)
        for n in needles:
            if n in line:
                return line
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port", help="node serial port, e.g. COM6 or /dev/ttyACM0")
    ap.add_argument("hexfile", help="ATtiny sketch .hex")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    lines, size = check_hex(args.hexfile)
    print(f"{args.hexfile}: {size} bytes ({APP_LIMIT - size} bytes free)")

    port = serial.Serial()
    port.port, port.baudrate, port.timeout = args.port, args.baud, 0.2
    port.dtr = False            # don't reset the T-Beam when the port opens
    port.rts = False
    port.open()
    time.sleep(0.5)
    port.reset_input_buffer()

    # The node may be booting (some USB adapters reset it anyway), so ask again
    # until it answers.
    ready = None
    for _ in range(8):
        port.write(b"\nattiny-flash\n")
        ready = wait_for(port, ["[attiny] ready"], 2.0)
        if ready:
            break
    if not ready:
        sys.exit("node did not answer 'attiny-flash' (is the new node firmware on it?)")

    for line in lines:
        port.write((line + "\n").encode("ascii"))
        time.sleep(0.002)
    port.flush()

    result = wait_for(port, ["[attiny] OK", "[attiny] FAIL"], 60.0)
    port.close()
    if result is None:
        sys.exit("no result from the node within 60 s")
    sys.exit(0 if "[attiny] OK" in result else 1)


if __name__ == "__main__":
    main()
