# Off-target test of the wind sketch's sleep/wake power saving.
# Needs: g++, simavr 1.6 headers/library (apt install libsimavr-dev libelf-dev).
#
#   make test HEX=sketch.hex [MPH_PER_HZ=1.492]
#
# HEX is a build of ../attiny85_wind_sensor.ino (Arduino IDE: Sketch > Export
# Compiled Binary). MPH_PER_HZ: 1.492 SparkFun, 2.7 Inspeed WS2 plastic,
# 2.45 WS2 metal, 2.85 Classic plastic.

CXXFLAGS = -O2 -std=c++17 -Wall -I/usr/include/simavr
LDLIBS   = -lsimavr -lelf
MPH_PER_HZ ?= 1.492

sim_power: sim_power.cpp
	$(CXX) $(CXXFLAGS) -o $@ sim_power.cpp $(LDLIBS)

test: sim_power
	./sim_power $(HEX) $(MPH_PER_HZ)

clean:
	rm -f sim_power

.PHONY: test clean
