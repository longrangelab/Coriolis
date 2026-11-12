# Coriolis ATtiny85 I2C bootloader (twiboot, see README.md)
#
#   make            build twiboot.hex
#   make install    flash the bootloader with an ISP programmer (one time)
#   make fuses      set the fuses (one time)
#
# PROG is the avrdude programmer. The default is an Arduino running the
# "ArduinoISP" sketch on serial port PORT (required, e.g. PORT=COM7). Override on the command line, e.g.
#   make install PROG="-c usbasp"
#   make install PROG="-c stk500v1 -b 19200 -P COMx"

MCU              = attiny85
BOOTLOADER_START = 0x1C00
PORT            ?= $(error set PORT to the programmer's serial port, e.g. make install PORT=COM7)
PROG            ?= -c stk500v1 -b 19200 -P $(PORT)

# Fuses:  L 0xE2  8 MHz internal RC, no clock divide (same as the sketch)
#         H 0xDD  SPI programming on, 2.7 V brown-out (protects flash writes)
#         E 0xFE  SELFPRGEN: lets the bootloader write flash
FUSES = -U lfuse:w:0xe2:m -U hfuse:w:0xdd:m -U efuse:w:0xfe:m

CC      = avr-gcc
OBJCOPY = avr-objcopy
SIZE    = avr-size

CFLAGS  = -Os -mmcu=$(MCU) -Wall -fdata-sections -ffunction-sections
CFLAGS += -DBOOTLOADER_START=$(BOOTLOADER_START) -DUSE_CLOCKSTRETCH=1 -DVIRTUAL_BOOT_SECTION=1
LDFLAGS = -Wl,--relax,--gc-sections,--section-start=.text=$(BOOTLOADER_START) -nostartfiles

all: twiboot.hex

twiboot.elf: twiboot.c
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $<
	$(SIZE) $@

twiboot.hex: twiboot.elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@

install: twiboot.hex
	avrdude -p t85 $(PROG) -U flash:w:twiboot.hex:i

fuses:
	avrdude -p t85 $(PROG) $(FUSES)

clean:
	rm -f twiboot.elf

.PHONY: all install fuses clean
