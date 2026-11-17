#!/bin/bash
# Build attiny85_wind_sensor.ino for the real chip with avr-gcc, outside the
# Arduino IDE, the same way the release .hex files are built.
#
#   build_sketch.sh <ATTinyCore dir> <TinyWireS dir> <out dir> <meter defines...>
#   e.g. build_sketch.sh ATTinyCore TinyWireS out/sparkfun -DWIND_METER_SPARKFUN
#
# ATTinyCore: github.com/SpenceKonde/ATTinyCore (release builds: 9e70249)
# TinyWireS : github.com/nadavmatalon/TinyWireS (release builds: 9c08201)
# Needs gcc-avr, avr-libc, binutils-avr. Writes <out>/sketch.hex and prints
# the flash use; fails if the sketch would overlap the I2C bootloader.
set -e
CORE=$(cd "$1" && pwd); TWS=$(cd "$2" && pwd); OUT=$3; shift 3
INO="$(cd "$(dirname "$0")/.." && pwd)/attiny85_wind_sensor.ino"
mkdir -p "$OUT"; OUT=$(cd "$OUT" && pwd); rm -f "$OUT"/*.o "$OUT"/*.a
CF="-mmcu=attiny85 -DF_CPU=8000000UL -DCLOCK_SOURCE=0 -DARDUINO=10819 -DARDUINO_AVR_ATTINYX5 -DARDUINO_ARCH_AVR -Os -flto -ffunction-sections -fdata-sections -w -I$CORE/avr/cores/tiny -I$CORE/avr/variants/tinyx5 -I$TWS -I$TWS/utility"
for f in "$CORE"/avr/cores/tiny/*.c;   do avr-gcc $CF -std=gnu11 -fno-fat-lto-objects -c "$f" -o "$OUT/$(basename "$f").o"; done
for f in "$CORE"/avr/cores/tiny/*.cpp; do avr-g++ $CF -std=gnu++11 -fno-exceptions -fno-threadsafe-statics -fno-fat-lto-objects -c "$f" -o "$OUT/$(basename "$f").o"; done
for f in "$CORE"/avr/cores/tiny/*.S;   do avr-gcc $CF -x assembler-with-cpp -c "$f" -o "$OUT/$(basename "$f").o"; done
avr-gcc-ar rcs "$OUT/core.a" "$OUT"/*.o; rm "$OUT"/*.o
avr-g++ $CF -std=gnu++11 -fno-exceptions -fno-threadsafe-statics -c "$TWS/TinyWireS.cpp" -o "$OUT/tws.o"
avr-gcc $CF -std=gnu11 -c "$TWS/utility/USI_TWI_Slave.c" -o "$OUT/usi.o"
( echo '#include <Arduino.h>'; cat "$INO" ) > "$OUT/sketch.cpp"
avr-g++ $CF -std=gnu++11 -fno-exceptions -fno-threadsafe-statics "$@" -c "$OUT/sketch.cpp" -o "$OUT/sketch.o"
avr-gcc $CF -Wl,--gc-sections -fuse-linker-plugin "$OUT/sketch.o" "$OUT/tws.o" "$OUT/usi.o" "$OUT/core.a" -lm -o "$OUT/sketch.elf"
avr-objcopy -O ihex -R .eeprom "$OUT/sketch.elf" "$OUT/sketch.hex"
SIZE=$(avr-size -A "$OUT/sketch.elf" | awk '$1==".text"||$1==".data"{s+=$2} END{print s}')
echo "flash used: $SIZE of 7168 bytes ($*)"
[ "$SIZE" -le 7168 ] || { echo "too big: the I2C bootloader starts at 0x1C00"; exit 1; }
