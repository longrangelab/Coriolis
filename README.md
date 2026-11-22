# Coriolis
Coriolis is an open-source,long-range, digital platform that is capable of hosting a variety of sensors built for long-range precision shooters. 

Discord link for build questions or to chat about ideas: https://discord.gg/T3Q2JN2y2S


This video briefly describes Coriolis and what it can do: https://www.youtube.com/watch?v=9miKXuf_NCE&t=154s 



## Opening Wind Lab on a phone or tablet

The base station runs its own Wi-Fi network and serves the Wind Lab page (live wind, wind hold, solver, log). Nothing to install:

1. Power on the base station.
2. On your phone or tablet, join the Wi-Fi network **`WindStation`**. The password is **`windwind12`**.
3. Open a browser and go to **http://192.168.4.1** (type the `http://`; there is no `https` version).

The network has no internet, so your phone may warn about that. Choose to stay connected. The page keeps working with no signal anywhere else, and your rifles and loads are saved on the base station so every phone that joins sees them.

## Flashing the firmware

- **T-Beam node and base station:** download the `.bin` files from [Releases](https://github.com/longrangelab/Coriolis/releases) and write them with [espflash.app](https://espflash.app/). The release notes have the steps.
- **ATtiny85 wind sensor chip:** each new chip needs a one-time setup with an Arduino UNO. Follow the step-by-step [ATtiny85 first-time setup guide](Coriolis/attiny85_bootloader/FIRST_TIME_SETUP.md). After that, updates go over I2C through the node.
