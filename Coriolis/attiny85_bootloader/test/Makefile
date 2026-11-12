# Off-target test of the I2C update path (bootloader + sketch + node flasher).
# Needs: g++, simavr 1.6 headers/library (apt install libsimavr-dev libelf-dev).
#
#   make test APP_V1=sketch_a.hex APP_V2=sketch_b.hex
#
# APP_V1/APP_V2 are two different builds of attiny85_wind_sensor.ino
# (Arduino IDE: Sketch > Export Compiled Binary), e.g. FORCE_SENSOR 1 and 2.

CXXFLAGS = -O2 -std=c++17 -Wall -I/usr/include/simavr
LDLIBS   = -lsimavr -lelf

sim_test: sim_test.cpp ../../Node/lib/AttinyFlasher/AttinyFlasher.h
	$(CXX) $(CXXFLAGS) -o $@ sim_test.cpp $(LDLIBS)

test: sim_test
	$(MAKE) -C .. twiboot.hex
	./sim_test ../twiboot.hex $(APP_V1) $(APP_V2)

clean:
	rm -f sim_test

.PHONY: test clean
