# Coriolis
Coriolis is an open-source, hit indicator, GPS range finder, and fully automated windage adjustment system for rifle scopes. It integrates downrange wind meters, a ballistic solver, a motorized scope dial to apply precise, and real-time wind corrections for long-range precision shooting.

<hr style="display: inline-block; width: 100%; border: 1px dotted #ff00cc;">

<p align="center" style="margin-top: -2em;">
  <a href="https://discord.gg/G65yBWfZ">
    <picture>
      <source height="24px" media="(prefers-color-scheme: dark)" srcset="/assets/icons/Discord.png" />
      <img height="24px" src="/assets/icons/Discord.png" />
    </picture>
  </a>&nbsp;&nbsp;&nbsp;
  <a href="https://www.youtube.com/@Longrangelab">
    <picture>
      <source height="24px" media="(prefers-color-scheme: dark)" srcset="/assets/icons/YouTube.png" />
      <img height="24px" src="/assets/icons/YouTube.png" />
    </picture>
  </a>
</p>

## YouTube video
[Here](https://www.youtube.com/watch?v=kEiZPHotwlk) is a YouTube video that explains the project.

## Images
### Base station:
  <picture>
    <img height=640 src="/assets/images/Base_Station.jpg" />
  </picture>

### FAWD (Fully Autonomous Wind Dial):
  <picture>
    <img height=640 src="/assets/images/fawd.jpg" />
  </picture>

### Target unit connect to weather meter:
  <picture>
    <img height=640 src="/assets/images/target_unit_connected_to_weather_meter.jpg" />
  </picture>

## How It Works
Wind Data Collection – Multiple long-range wind meters send live telemetry to a central base station.

Ballistic Processing – A microcontroller processes the data through a ballistic solver, applying weighted influence based on meter placement (e.g., near max ordnance vs. in valleys).

Automatic Dialing – The system transmits the calculated correction via [Bluetooth](https://en.wikipedia.org/wiki/Bluetooth) to the scope-mounted motor, which physically turns the windage dial every 1–3 seconds as new data arrives.

Real-Time Interface (Planned) – A live dashboard will display wind readings, averages, and weighting adjustments for experimentation and tuning.

## Current Build
Mounting – 3D-printed PLA bracket ([Arken Optics LH4 6-24](https://opticsforce.com/products/arken-optics-lh4-6-24x50-ffp-capped-tool-less-turrets-illuminated-vhr-vpr-30mm-tube), discontinued).

Motion – Stepper motor + 5:1 gearbox for torque, driven by a [DRV8825](https://www.pololu.com/product/2133) driver.

Power – RC car battery.

Control – [Arduino](https://www.arduino.cc/) microcontroller running simple but precise early-stage firmware.

## Applications
Testing and refining the Corololis wind meter system.

Pushing rimfire and .22 centerfire cartridges to extreme distances.

Experimental research in wind reading placement, height, and configuration.

## Project Goals
1. Achieve consistent 1,000-yard accuracy across all wind conditions.
2. Shrink target size as wind-reading precision improves.
3. Make all parts lists, code, diagrams, and STL files freely available for DIY builders.

## Contribution
Looking for collaborators with skills in electronics, firmware, PCB design, 3D modeling, and software.

All designs will remain open source; pre-built units may also be available for plug-and-play use.

## Usage

### For Target-Device(lora_shooting-targetside)
1. Uncomment line 11  #define IS_TARGET if this line is being commented
2. Upload to device ,check serial log

### For User-Device(lora_shooting-userside)
1. Comment line 11  #define IS_TARGET if this line is being uncommented
2. Upload to device ,check serial log

If there is anything unusual, please let me know and I will correct it for you immediately.

