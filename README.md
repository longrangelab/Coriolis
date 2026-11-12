# Coriolis
Coriolis is an open-source,long-range, digital platform that is capable of hosting a variety of sensors built for long-range precision shooters. 

Discord link for build questions or to chat about ideas: https://discord.gg/T3Q2JN2y2S


This video briefly describes Coriolis and what it can do: https://www.youtube.com/watch?v=9miKXuf_NCE&t=154s 



## Flashing the firmware

- **T-Beam node and base station:** download the `.bin` files from [Releases](https://github.com/longrangelab/Coriolis/releases) and write them with [espflash.app](https://espflash.app/). The release notes have the steps.
- **ATtiny85 wind sensor chip:** each new chip needs a one-time setup with an Arduino UNO. Follow the step-by-step [ATtiny85 first-time setup guide](Coriolis/attiny85_bootloader/FIRST_TIME_SETUP.md). After that, updates go over I2C through the node.
