# Urashima Effect Device

![Urashima Effect Device](https://github.com/kennel-org/urashima-effect-device/raw/master/images/device_photo.jpg)

## Overview

The Urashima Effect Device is a portable gadget designed to demonstrate time dilation effects based on Einstein's Theory of Special Relativity. It's named after the Japanese folktale "Urashima Taro," in which the protagonist spends what feels like a few days at the Dragon Palace, only to return home and find that hundreds of years have passed in the real world.

This device uses an M5Stack AtomS3R with an AtomicBase GPS unit to simulate relativistic time dilation in real-time based on your movement speed.

## The Urashima Taro Legend

The name "Urashima Effect" comes from the famous Japanese folktale of Urashima Taro, which dates back over 1,500 years. In the story, a young fisherman named Urashima Taro rescues a turtle, which turns out to be a princess from the Dragon Palace (Ryūgū-jō) beneath the sea. As a reward, he is invited to the underwater palace where he enjoys a few days of hospitality.

When Urashima decides to return home, he is given a mysterious box (tamatebako) with instructions never to open it. Upon returning to his village, he discovers that 300 years have passed in what felt like only a few days to him. In his shock and confusion, he opens the forbidden box, which releases a cloud of white smoke that instantly transforms him into an old man – his true age catching up with him.

This ancient tale presents a fascinating parallel to Einstein's relativistic time dilation, where time can pass at different rates for observers in different reference frames. Just as Urashima experienced time at a different rate in the Dragon Palace, our device demonstrates how movement affects the passage of time according to the Theory of Special Relativity.

## Features

- **Real-time relativistic time dilation calculation**: Computes time dilation based on movement speed using Einstein's Special Relativity equations
- **GPS speed measurement**: Accurate velocity tracking with a high-precision GPS module
- **GPS-IMU sensor fusion**: Combines GPS and IMU data for improved speed accuracy, with fallback to IMU-only when GPS is unavailable
- **Accelerometer backup**: Estimates speed using the built-in accelerometer when GPS signals are weak
- **Time and distance reset**: Long-press the button to reset elapsed time and distance measurements
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
4. Short press the physical button to cycle through display modes:
   - Main display (speed, time dilation)
   - GPS raw data display
   - IMU raw data display
5. Long press the physical button (2 seconds) to enter reset confirmation mode:
   - Press the button again to reset elapsed time, relativistic time, and total distance
   - Wait 5 seconds to cancel the reset operation
   - Confirmation and cancellation messages will be displayed

## How It Works

This device calculates time dilation based on Einstein's Theory of Special Relativity. Since the actual speed of light (approximately 300,000 km/s) would make time dilation effects imperceptible at everyday speeds, this device uses a virtual "modified light speed" (0.03 km/s) to exaggerate the effect.

This allows you to experience significant time dilation effects even when walking or driving. For example, moving at 60 km/h will show a noticeable slowing of time compared to someone who is stationary.

The time dilation is calculated using the Lorentz factor:

γ = 1/√(1-(v²/c²))

Where:
- γ (gamma) is the time dilation factor
- v is your current speed
- c is the modified speed of light (0.03 km/s or 108 km/h)

## Time Dilation Visualization

The following graph illustrates the relationship between speed and time dilation in the Urashima Effect Device:

![Time Dilation Graph](images/lorentz_effect_plot.png)

As shown in the graph, time dilation effects become increasingly pronounced as your speed approaches the virtual light speed of 108 km/h:

- At 30 km/h: Time passes at approximately 0.96× normal rate (γ ≈ 1.04)
- At 60 km/h: Time passes at approximately 0.83× normal rate (γ ≈ 1.20)
- At 90 km/h: Time passes at approximately 0.55× normal rate (γ ≈ 1.81)
- At 100 km/h: Time passes at approximately 0.38× normal rate (γ ≈ 2.65)
- At 107.5 km/h: Time passes at approximately 0.10× normal rate (γ ≈ 10.40)

When γ = 2, time for a moving observer passes at half the rate of a stationary observer. This occurs at approximately 93.5 km/h with our virtual light speed setting.

## Technical Details

- GPS update rate: 1Hz
- Accelerometer sampling rate: As fast as possible (device-dependent)
- GPS-IMU fusion weight: 0.2 (20% GPS, 80% IMU when both are available)
- GPS validity timeout: 5 seconds (falls back to IMU-only after this period)
- Time dilation calculation: Lorentz factor γ = 1/√(1-(v²/c²))
- Data logging interval: 1 second
- Display modes: Main, GPS Raw, IMU Raw (cycle with short button press)

## License

MIT License

## Contributing

Pull requests and feature suggestions are welcome. For major changes, please open an issue first to discuss what you would like to change.

## Acknowledgments

This project would not have been possible without the support of the M5Stack community and the open-source hardware/software ecosystem.
