# Urashima Effect Device

![Urashima Effect Device](https://github.com/kennel-org/urashima-effect-device/raw/master/images/device_photo.jpg)

## Overview

The Urashima Effect Device is a portable gadget designed to demonstrate time dilation effects based on Einstein's Theory of Special Relativity. It's named after the Japanese folktale "Urashima Taro," in which the protagonist spends what feels like a few days at the Dragon Palace, only to return home and find that hundreds of years have passed in the real world.

This device uses an M5Stack AtomS3R with an AtomicBase GPS unit to simulate relativistic time dilation in real-time based on your movement speed.

## Features

- **Real-time relativistic time dilation calculation**: Computes time dilation based on movement speed using Einstein's Special Relativity equations
- **GPS speed measurement**: Accurate velocity tracking with a high-precision GPS module
- **Accelerometer backup**: Estimates speed using the built-in accelerometer when GPS signals are weak
- **SD card logging**: Records movement data and time dilation effects to CSV files
- **Compact design**: Displays essential information on the M5Stack AtomS3R's small screen
- **Battery-powered**: Built-in battery for outdoor use

## Hardware Requirements

- M5Stack AtomS3R
- M5Stack AtomicBase GPS
- microSD card (optional, for data logging)

## Software Requirements

- Arduino IDE
- Required libraries:
  - M5Unified
  - TinyGPS++
  - SPI
  - SD
  - FS

## Installation

1. Clone or download this repository
2. Open `Urashima_Effect_Device_Arduino.ino` in the Arduino IDE
3. Select the M5Stack AtomS3R board
4. Compile and upload to your device

## Usage

1. Power on the device to see the startup screen
2. Move to an outdoor location or near a window to acquire GPS signals
3. Once GPS signals are acquired, the current speed and time dilation effects will be displayed
4. Touch the reset button at the bottom of the screen to reset the elapsed time measurement
5. Short press the physical button to toggle GPS raw data display mode
6. Long press the physical button to calibrate the accelerometer with GPS data

## How It Works

This device calculates time dilation based on Einstein's Theory of Special Relativity. Since the actual speed of light (approximately 300,000 km/s) would make time dilation effects imperceptible at everyday speeds, this device uses a virtual "modified light speed" (0.3 km/s) to exaggerate the effect.

This allows you to experience significant time dilation effects even when walking or driving. For example, moving at 60 km/h will show a noticeable slowing of time compared to someone who is stationary.

## Technical Details

- GPS update rate: 1Hz
- Accelerometer sampling rate: As fast as possible (device-dependent)
- Time dilation calculation: Lorentz factor γ = 1/√(1-(v²/c²))
- Data logging interval: 1 second

## License

MIT License

## Contributing

Pull requests and feature suggestions are welcome. For major changes, please open an issue first to discuss what you would like to change.

## Acknowledgments

This project would not have been possible without the support of the M5Stack community and the open-source hardware/software ecosystem.
