# ATtiny85 first-time setup (Arduino UNO as programmer)

Every new ATtiny85 needs this setup **once**, before it goes into a wind meter.
It takes about 15 minutes the first time and 2 minutes for each chip after that.

This one step does three things:

1. Sets the chip's **fuses**: 8 MHz clock, brown-out protection, and permission
   for the chip to rewrite its own flash.
2. Installs the small **I2C bootloader**. This lets the wind meter node update
   the chip later over the wires it already uses.
3. Installs the **wind sensor firmware** for your meter type.

After that, the chip lives in the meter for good. Every later update goes
through the node over USB (see [Updating later](#8-updating-later-no-unplugging-needed)),
so you won't need the UNO again for that chip.

> **Already have a meter running V1.1.0 or older?** Its ATtiny has no I2C
> bootloader yet. Pull the chip once, do this setup, and put it back.

## Contents

1. [What you need](#1-what-you-need)
2. [Turn the UNO into a programmer](#2-turn-the-uno-into-a-programmer-arduinoisp)
3. [Wire the ATtiny to the UNO](#3-wire-the-attiny85-to-the-uno)
4. [Find avrdude](#4-find-avrdude)
5. [Download the firmware for your meter](#5-download-the-firmware-for-your-wind-meter)
6. [Program the chip](#6-program-the-chip-one-command)
7. [Move the chip into the meter and check it](#7-move-the-chip-into-the-wind-meter-and-check-it)
8. [Updating later](#8-updating-later-no-unplugging-needed)
9. [Never do this afterwards](#9-never-do-this-after-setup)
10. [Troubleshooting](#10-troubleshooting)

---

## 1. What you need

| Item | Notes |
|---|---|
| Arduino UNO (or clone) and its USB cable | Any UNO works, including CH340 clones. A Nano also works with the same pins. |
| ATtiny85, DIP-8 package | ATtiny85-20PU or ATtiny85V-10PU. A new blank chip is fine. |
| 10 µF electrolytic capacitor | Anything from 10 µF to 22 µF, 10 V or higher. |
| Breadboard and 6 jumper wires | Or an "ATtiny programming shield" for the UNO, which has the socket, wiring and capacitor built in. |
| A PC with the [Arduino IDE](https://www.arduino.cc/en/software) installed | Windows, macOS or Linux. It provides the ArduinoISP sketch and the `avrdude` program. |
| The V1.2.0 (or newer) release files | From the [Releases page](https://github.com/longrangelab/Coriolis/releases). See step 5. |

You do **not** need ATTinyCore or any extra board package for this. You only
need it if you want to build your own changes to the sketch.

---

## 2. Turn the UNO into a programmer (ArduinoISP)

Do this with **nothing** plugged into the UNO's headers yet, especially not the
capacitor. The capacitor stops the UNO from accepting new sketches.

1. Plug the UNO into your PC by USB.
2. Open the Arduino IDE.
3. **Tools > Board > Arduino AVR Boards > Arduino Uno.**
4. **Tools > Port:** pick the UNO's port. On Windows it looks like `COM9`. On
   macOS it looks like `/dev/cu.usbmodem…`, and on Linux like `/dev/ttyACM0`.
   Write the port down, because you need it in step 6.
   > On Windows, if you're not sure which port is the UNO, unplug it, look at
   > the list, plug it back in, and see which port appears. If you also have a
   > Coriolis node or base station plugged in, they have their own ports. Don't
   > pick those.
5. **File > Examples > 11.ArduinoISP > ArduinoISP.** (In Arduino IDE 2 it's under
   *File > Examples > Built-in examples > 11.ArduinoISP*.)
6. Click **Upload** (the right-arrow button). Wait for "Done uploading".

The UNO is now a programmer. It keeps this sketch until you upload something
else, so you only do this step once, not once per chip.

---

## 3. Wire the ATtiny85 to the UNO

**Unplug the UNO's USB cable before you wire anything.**

### Find pin 1

Look at the chip from the top (the side with the writing), legs pointing down
into the table. One short end has a **notch** (a half-moon cut-out), and many
chips also have a small **dot** in one corner. Turn the chip so the notch is at
the top. Pin 1 is then the **top-left** leg, the one by the dot. Pins count
down the left side (1, 2, 3, 4) and back up the other side (5, 6, 7, 8).

```
                 notch
             ┌─────U─────┐
   pin 1  ●──┤ 1       8 ├──  pin 8
   pin 2   ──┤ 2       7 ├──  pin 7
   pin 3   ──┤ 3       6 ├──  pin 6
   pin 4   ──┤ 4       5 ├──  pin 5
             └───────────┘
            (top view, legs down)
```

### Connections

| ATtiny85 pin | Name | Connect to UNO |
|---|---|---|
| 1 | RESET | **D10** |
| 4 | GND | **GND** |
| 5 | PB0 (MOSI) | **D11** |
| 6 | PB1 (MISO) | **D12** |
| 7 | PB2 (SCK) | **D13** |
| 8 | VCC | **5V** |
| 2, 3 | | nothing |

Then the capacitor, on the **UNO** (not the ATtiny):

| Capacitor leg | Connect to UNO |
|---|---|
| Long leg (+) | **RESET** |
| Short leg (−), the side with the stripe | **GND** |

```
   UNO                          ATtiny85
  ┌──────┐                    ┌────U────┐
  │  D10 ├────────────────────┤1       8├──────────┐
  │      │                    │2       7├───────┐  │
  │      │                    │3       6├────┐  │  │
  │  GND ├────────────────────┤4       5├─┐  │  │  │
  │      │                    └─────────┘ │  │  │  │
  │  D11 ├────────────────────────────────┘  │  │  │
  │  D12 ├───────────────────────────────────┘  │  │
  │  D13 ├──────────────────────────────────────┘  │
  │   5V ├─────────────────────────────────────────┘
  │      │
  │RESET ├──┐
  │      │ ─┴─  (+)
  │      │ ─┬─  (−) stripe      10 µF capacitor
  │  GND ├──┘                   (the UNO has several GND pins; any one works)
  └──────┘
```

Why the capacitor? Opening the UNO's port normally resets the UNO. The capacitor
stops that reset, so the UNO stays in programmer mode while avrdude talks to it.

Double-check every wire, especially 5V and GND. Then plug the UNO's USB back in.

---

## 4. Find avrdude

`avrdude` is the program that writes to the chip. The Arduino IDE already
installed it.

### Windows

Open **PowerShell** (Start menu, type "PowerShell") and run:

```powershell
Get-ChildItem "$env:LOCALAPPDATA\Arduino15\packages\arduino\tools\avrdude" -Recurse -Include avrdude.exe,avrdude.conf | Select-Object FullName
```

You get two paths, something like:

```
C:\Users\you\AppData\Local\Arduino15\packages\arduino\tools\avrdude\6.3.0-arduino17\bin\avrdude.exe
C:\Users\you\AppData\Local\Arduino15\packages\arduino\tools\avrdude\6.3.0-arduino17\etc\avrdude.conf
```

The version folder (`6.3.0-arduino17` here) may differ. That's fine. If several
versions are listed, use the newest, and take both files from the same version
folder.

If nothing is listed, open the Arduino IDE once, let it finish installing
"Arduino AVR Boards", and run the command again. Or download avrdude on its own
from [github.com/avrdudes/avrdude/releases](https://github.com/avrdudes/avrdude/releases)
(the Windows zip has `avrdude.exe` and `avrdude.conf` side by side).

### macOS / Linux

The easiest way is to install avrdude itself:

```bash
brew install avrdude          # macOS (Homebrew)
sudo apt install avrdude      # Debian / Ubuntu / Raspberry Pi OS
```

Then you can leave out `-C <avrdude.conf>` in the commands below.

### Check that the UNO can see the chip

This only reads from the chip. It writes nothing. In PowerShell, replace the two
paths with yours and `COM9` with your UNO's port:

```powershell
& "C:\...\bin\avrdude.exe" -C "C:\...\etc\avrdude.conf" -p attiny85 -c stk500v1 -P COM9 -b 19200
```

(The `&` at the start is PowerShell's way to run a program from a quoted path.
In the old Command Prompt, leave it out.)

You want to see the chip's signature:

```
avrdude: Device signature = 0x1e930b          (avrdude 6.x)
Device signature = 1E 93 0B (ATtiny85)         (avrdude 7.x and 8.x)
```

If you see that, the wiring is right. If not, see [Troubleshooting](#10-troubleshooting).

---

## 5. Download the firmware for your wind meter

On the [Releases page](https://github.com/longrangelab/Coriolis/releases), open
**V1.2.0** (or newer) and download the `isp_with_bootloader` file for your
meter. The calibration is built into the file, so **pick the one that matches
your hardware**:

| Your wind meter | File to download | LED code |
|---|---|---|
| Inspeed WS2, plastic (polycarbonate) cups | `attiny85_inspeed_ws2_plastic_1.2.0_isp_with_bootloader.hex` | 3 blinks |
| Inspeed WS2, metal cups | `attiny85_inspeed_ws2_metal_1.2.0_isp_with_bootloader.hex` | 3 blinks |
| Inspeed Classic (1.25" body), plastic cups | `attiny85_inspeed_classic_plastic_1.2.0_isp_with_bootloader.hex` | 3 blinks |
| SparkFun weather meter kit | `attiny85_sparkfun_1.2.0_isp_with_bootloader.hex` | 2 blinks |

Also download the matching `_i2c_update.hex` for your meter. You don't need it
today, but it's the file you'll use for later updates (step 8).

Put the `.hex` file in a folder you can find, for example
`Documents\Coriolis\attiny`. The examples below use the Inspeed WS2 plastic file.

> Using a newer release? Replace `1.2.0` in the file names with that version.

---

## 6. Program the chip (one command)

In PowerShell, go to the folder with the `.hex` file:

```powershell
cd "$env:USERPROFILE\Documents\Coriolis\attiny"
```

Then run this as **one line**, with your avrdude paths, your UNO's port and your
meter's file:

```powershell
& "C:\...\bin\avrdude.exe" -C "C:\...\etc\avrdude.conf" -p attiny85 -c stk500v1 -P COM9 -b 19200 -e -U lfuse:w:0xe2:m -U hfuse:w:0xdd:m -U efuse:w:0xfe:m -U flash:w:attiny85_inspeed_ws2_plastic_1.2.0_isp_with_bootloader.hex:i
```

On macOS / Linux:

```bash
avrdude -p attiny85 -c stk500v1 -P /dev/ttyACM0 -b 19200 -e -U lfuse:w:0xe2:m -U hfuse:w:0xdd:m -U efuse:w:0xfe:m -U flash:w:attiny85_inspeed_ws2_plastic_1.2.0_isp_with_bootloader.hex:i
```

What each part does:

| Part | Meaning |
|---|---|
| `-p attiny85` | The chip type. |
| `-c stk500v1 -P COM9 -b 19200` | Talk to the ArduinoISP sketch on the UNO's port at 19200 baud. |
| `-e` | Erase the chip first. |
| `-U lfuse:w:0xe2:m` | Low fuse **0xE2**: 8 MHz internal clock. A new chip runs at 1 MHz until this is set. |
| `-U hfuse:w:0xdd:m` | High fuse **0xDD**: brown-out reset at 2.7 V. This protects flash writes when the battery sags. |
| `-U efuse:w:0xfe:m` | Extended fuse **0xFE**: lets the bootloader write flash. Without it, I2C updates can't work. |
| `-U flash:w:<file>:i` | Write the bootloader and wind firmware (Intel HEX). |

It takes about 20 to 30 seconds. avrdude checks every write by reading it back.
A good run ends like this (the wording varies a little between versions):

```
avrdude: 1 bytes of lfuse verified
avrdude: 1 bytes of hfuse verified
avrdude: 1 bytes of efuse verified
avrdude: 7930 bytes of flash verified

avrdude done.  Thank you.
```

The flash byte count depends on your meter and avrdude version: about 7,900
for Inspeed and 7,700 for SparkFun. Older avrdude versions also count the empty
gap below the bootloader and show about 8,150. Any of these is fine.

Anything that says `mismatch`, `error` or `not in sync` means it did not work.
See [Troubleshooting](#10-troubleshooting). It is always safe to run the same
command again.

### Optional: a second check

This reads the fuses and compares the whole flash against the file again:

```powershell
& "C:\...\bin\avrdude.exe" -C "C:\...\etc\avrdude.conf" -p attiny85 -c stk500v1 -P COM9 -b 19200 -U lfuse:r:-:h -U hfuse:r:-:h -U efuse:r:-:h -U flash:v:attiny85_inspeed_ws2_plastic_1.2.0_isp_with_bootloader.hex:i
```

You should see `0xe2`, `0xdd`, `0xfe` and `flash verified`.

### Programming more chips

Unplug the UNO's USB, swap in the next chip (pin 1 the same way), plug it back
in, and run the same command. If the next chip goes into a different type of
meter, change the file name.

Label each programmed chip (a dot of paint or a bit of tape with "WS2-P", "SF"
and so on), because they all look the same.

---

## 7. Move the chip into the wind meter and check it

### Moving the chip

1. **Unplug the UNO's USB first,** so the chip has no power when you pull it.
2. **Turn the wind meter node off.** Switch the T-Beam off and unplug its USB
   cable. Never put a chip into a powered socket.
3. Lift the ATtiny out evenly. Use a chip puller, or work a small flat
   screwdriver under each end in turn, a little at a time, so the legs don't
   bend.
4. Line up **pin 1** with the notch on the daughterboard's 8-pin socket (and its
   silkscreen outline). A chip in backwards can be damaged when powered.
5. Check that all 8 legs go into the socket and none fold under the chip. Press
   it in evenly.

### First power-up: watch the yellow LED

Turn the node on and watch the ATtiny's yellow LED on the daughterboard:

| Time | LED | Meaning |
|---|---|---|
| First ~1 s | Fast flicker | The I2C bootloader is waiting for an update. With no update, it starts the wind firmware. |
| Then | 1 blink | The wind firmware started. |
| Then, after a short pause | **3 blinks** (Inspeed) or **2 blinks** (SparkFun) | Which calibration is installed. Check it matches your meter. |
| From then on | Slow on/off, about every 2 s | Running normally. |

If the blink count is wrong, you programmed the wrong file. Put the chip back
on the UNO and repeat step 6 with the right one.

### Check the wind readings

1. Give the cups a spin. The node's screen should show a wind speed within a
   few seconds. It goes back to 0 about 10 s after the cups stop.
2. Turn the vane. The direction should follow while the cups are turning. The
   direction only updates while the cups turn, so a still vane on still cups
   holds its last value.

Optional deeper check: open a serial monitor on the node's port at **115200**
baud and restart the node. Look for:

```
[ATtiny] found
```

`[ATtiny] NOT found` means the node can't see the chip over I2C. Check
orientation and seating first.

---

## 8. Updating later (no unplugging needed)

From now on, the chip stays in the meter. To install a newer wind firmware:

1. The node must run Coriolis **1.2.0 or newer** (it adds the `attiny-flash`
   command). Flash `windlab_node_<version>_merged.bin` with
   [espflash.app](https://espflash.app/) as the release notes describe.
2. Install Python 3 and pyserial once: `python -m pip install pyserial`.
3. Get `attiny_i2c_flash.py` from this folder (`Coriolis/attiny85_bootloader/`)
   and the release's `attiny85_<meter>_<version>_i2c_update.hex` for your meter.
   Use the `_i2c_update` file here, **not** the `isp_with_bootloader` one.
4. Plug the node in by USB, close any serial monitor that has its port open,
   and run this, with `COM12` replaced by your node's port (not the UNO's):

   ```
   python attiny_i2c_flash.py COM12 attiny85_inspeed_ws2_plastic_1.2.0_i2c_update.hex
   ```

5. After about 12 s it prints:

   ```
     node: [attiny] OK 6942 bytes written and verified in 2700 ms, sketch starting
   ```

   The LED shows the same start-up pattern as in step 7.

If an update is interrupted, even by a power cut, the chip stays safely in its
bootloader (the LED keeps flickering) and you just run the command again.
More detail is in [README.md](README.md#updating-the-sketch-every-time-after-that).

---

## 9. Never do this after setup

Once a chip is set up, **don't use these Arduino IDE menu items on it:**

- **Tools > Burn Bootloader.** It rewrites the fuses for the Arduino board
  package. That turns off self-programming (efuse 0xFE becomes 0xFF), so the
  I2C bootloader can no longer write flash.
- **Sketch > Upload Using Programmer.** It erases the whole chip, including the
  I2C bootloader.

Either one means the node can no longer update the chip. Nothing is damaged.
Just repeat steps 3 to 6 to set it up again.

To change the firmware, use the I2C update in step 8. To program a chip with
your own build over the UNO, see "To program the sketch in the same ISP step"
in [README.md](README.md#one-time-setup-per-attiny-isp-programmer), which shows
how to combine your sketch with the bootloader.

---

## 10. Troubleshooting

| What you see | Likely cause and fix |
|---|---|
| Upload of ArduinoISP to the UNO fails | The 10 µF capacitor is already in. Pull it out, upload, then put it back. Also check that the board is "Arduino Uno" and the port is right. |
| `stk500_getsync(): not in sync` or `programmer is not responding` | Wrong port, the capacitor is missing or backwards, or the UNO is not running ArduinoISP (repeat step 2). On Windows, close the Arduino IDE's Serial Monitor and anything else using that port. |
| `Device signature = 0x000000` or `0xffffff`, or "Yikes! Invalid device signature" | avrdude reaches the UNO but not the ATtiny. Check the 6 wires, especially pin 1 to D10, 5V and GND, and that the chip is the right way round and fully seated. Don't add `-F` to force past this. |
| Signature is right but writing fails with `verification error` or `mismatch` | Loose breadboard contact. Press the chip and wires in firmly and run the command again. |
| A chip that was used in an older project gives a wrong signature | Its fuses may be set for an external crystal. It needs a high-voltage programmer or a crystal to recover. Using a new chip is easier. |
| `can't open device "COM9"` / `Access is denied` | Another program has the port open (Serial Monitor, Cura, another avrdude). Close it, or unplug and replug the UNO. |
| PowerShell says `Unexpected token '-C'` | You left out the `&` at the start of the line. |
| PowerShell: `avrdude.exe is not recognized` | Use the full quoted path from step 4, with `&` in front. |
| No LED activity at all in the meter | The chip is backwards or not seated, or the node isn't powered. |
| LED keeps flickering fast and never blinks the meter code | The chip has the bootloader but no wind firmware. That happens if you wrote the bootloader-only file, or after an interrupted I2C update. Run the I2C update (step 8) or repeat step 6 with the `isp_with_bootloader` file. |
| `[attiny] FAIL no bootloader answered at 0x43` during an I2C update | The chip never had this setup, or someone used Upload Using Programmer / Burn Bootloader on it. Do this setup. |
| Wind speed reads 0 while the cups spin | Check the anemometer plug is in the anemometer jack. Make sure the chip runs 1.2.0 or newer: 1.2.0 fixed a bug where chattering reed switches read as no wind. |

Still stuck? Ask on the Coriolis Discord (link in the [main README](../../README.md)),
and include the full avrdude output.
