# ATtiny85 I2C bootloader

Lets the wind meter node update the ATtiny85 wind sensor firmware over the I2C
wires it already uses (SDA/SCL on PB0/PB2). After a one-time setup with your
ISP programmer, the ATtiny never needs the programmer again.

It is [twiboot](https://github.com/orempel/twiboot) (Olaf Rempel, GPLv2) at
upstream commit `559a403`, with Coriolis changes marked `Coriolis:` in
`twiboot.c`.

## How it works

```
 PC  --USB serial-->  T-Beam node  --I2C-->  ATtiny85
 attiny_i2c_flash.py  "attiny-flash"         sketch at 0x42  (normal running)
                      Node/lib/AttinyFlasher bootloader at 0x43 (updating)
```

1. The node writes `{0x01, 0x00}` to the sketch at 0x42. The sketch resets
   itself with the watchdog.
2. After any reset the bootloader runs first and answers at **0x43**. With no
   I2C command for 1 s it starts the sketch. (At power-up the LED on PB1
   flickers for that 1 s.)
3. The node writes the new sketch page by page (64 bytes), reads every page
   back, and then tells the bootloader to start the sketch.

An interrupted update is safe. The reset vector is written last, so until the
whole image is verified the chip stays in the bootloader (even across power
cycles) and the update can simply be run again.

Memory: the bootloader takes the top 1 KB (0x1C00-0x1FFF, 988 bytes used).
**Sketches must be 7168 bytes or less.** The current sketch on `main` is about
5.1 KB; the PR #17 version is 6.7 KB (SparkFun) / 6.9 KB (Inspeed).
The sketch must not use the `EE_RDY` interrupt (it does not); the flasher
refuses an image that does.

## One-time setup per ATtiny (ISP programmer)

Fuses: `lfuse 0xE2` (8 MHz internal, as now), `hfuse 0xDD` (2.7 V brown-out,
protects flash writes), `efuse 0xFE` (self-programming on, required).

Easiest way to get the right avrdude command: in Arduino IDE turn on
*Preferences > Show verbose output during upload*, do one normal "Upload Using
Programmer" of the sketch, and copy the avrdude command it prints. Keep the
`avrdude`, `-C`, `-p`, `-c`, `-P`, `-b` parts and replace the `-U ...` part:

```
avrdude -C <avrdude.conf> -p attiny85 -c <programmer> [-P <port> -b <baud>] -e ^
  -U lfuse:w:0xe2:m -U hfuse:w:0xdd:m -U efuse:w:0xfe:m ^
  -U flash:w:twiboot.hex:i
```

(`^` continues a line in Windows cmd; use `\` on Linux/macOS, or put it on one
line.) `twiboot.hex` is prebuilt in this folder; `make` rebuilds it
(avr-gcc). With an "Arduino as ISP" programmer, `make install fuses PORT=COMx` (the programmer's port)
does the same.

The chip now sits in the bootloader with no sketch. Put it on the daughterboard
and load the sketch over I2C as below.

To program the sketch in the same ISP step instead, combine the two files and
write `combined.hex` in place of `twiboot.hex` above:

```
python combine_hex.py twiboot.hex attiny85_wind_sensor.ino.hex combined.hex
```

**Do not** use "Upload Using Programmer" or "Burn Bootloader" on this chip
again: both erase the bootloader or turn self-programming off. If that happens,
repeat the setup.

## Updating the sketch (every time after that)

1. Node firmware with the `attiny-flash` command (this branch) on the T-Beam.
2. Arduino IDE, ATtiny sketch: *Sketch > Export Compiled Binary*. Use the
   `.hex` **without** `with_bootloader` in its name.
3. With the node on USB (close the serial monitor first):

   ```
   python attiny_i2c_flash.py COMx attiny85_wind_sensor.ino.hex
   ```

   `COMx` is the wind meter node's port (Device Manager > Ports); the
   base station and other USB devices have their own, so check which is which.

   Needs pyserial (`python -m pip install pyserial`). It prints
   `[attiny] OK ... written and verified` in about 3 s.

Without Python: open a serial terminal at 115200, type `attiny-flash`, then
paste the `.hex` file contents.

If it fails, the node prints the reason. `no bootloader answered at 0x43` means
the chip has no bootloader, or its sketch is older than the enter-bootloader
command: do the one-time setup. Any other failure leaves the ATtiny in the
bootloader: just run the command again.

## Changes from upstream twiboot

Found by running it on the simulator below; the first four would have stopped
it working on this board.

| Change | Why |
|---|---|
| Reset vector jumps to 0x1C00 | Upstream computed the RJMP from the byte address and jumped to 0x1800. That only worked while the sketch was under 6 KB (the gap is erased flash, which slides into the bootloader). The PR #17 sketch is larger. |
| Application start calls word address 6 | Upstream called the byte address (12) of the EE_RDY vector as a word address, landing on the WDT vector, so the sketch never started. |
| ACK the last byte of a page / of "start application" (USI only) | Upstream NAKed them, so the ESP32 reported every page write as failed. |
| SCL never driven push-pull | Upstream set SCL as an output before enabling the USI and left it driven high after exit until the sketch's I2C start (~1.7 s), fighting the node's I2C. |
| I2C address 0x43 | 0x29 is the optional TSL2591 light sensor on the node's bus. |
| LED on PB1 only | Upstream drives PB4/PB5; PB4 is the anemometer input. |
| Watchdog cleared at start | Needed so the sketch can enter the bootloader with a watchdog reset. |

## Test (no hardware needed)

`test/sim_test.cpp` runs this bootloader and two builds of the real sketch on
simavr's ATtiny85 (with a USI two-wire and SPM model added) and drives them
with `Node/lib/AttinyFlasher` through a bit-level 100 kHz I2C master that
enforces the ESP32's 13 ms clock-stretch limit:

```
sudo apt install gcc-avr avr-libc libsimavr-dev libelf-dev
cd test && make test APP_V1=a.hex APP_V2=b.hex
```

An optional fourth argument, a `combine_hex.py` image of APP_V2, also checks
the ISP-combined path. It covers: fresh chip, first load, power-up hand-off, update of a running
sketch, an update cut off after 40 pages followed by power loss (chip stays in
the bootloader, retry works), oversized and EE_RDY images, the bootloader
ignoring other addresses, no bus contention, longest clock stretch (9.1 ms).
