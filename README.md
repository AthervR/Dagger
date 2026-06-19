# Dagger — GPS-Guided Model Rocket Flight Computer

**Dagger** is a custom flight computer built around the **Teensy 4.1** microcontroller for a GPS-guided high-powered model rocket. The goal is active trajectory correction using real-time GPS positioning and inertial navigation, with onboard data logging for post-flight analysis.

## Hardware

| Component | Interface | Role |
|-----------|-----------|------|
| Teensy 4.1 | — | Main flight computer (600 MHz ARM Cortex-M7) |
| ICM-20948 9-DOF IMU | SPI0, CS pin 10 | Accelerometer (±16g), gyroscope (±2000 dps), magnetometer |
| BMP390 Barometer | SPI0, CS pin 9 | Pressure-based altitude, ±0.03 hPa resolution |
| SSD1306 0.91" OLED | I2C0 (pins 18/19) | Real-time status display on the ground |
| u-blox NEO-M9N GPS | UART1 (pins 0/1) | High-sensitivity L1 GNSS, 5 Hz position updates |
| MicroSD (SDIO) | Built-in Teensy 4.1 slot | Flight log — IMU, baro, GPS at full data rates |

## GPS Guided Mode

The primary mission of Dagger is **GPS-in-the-loop guidance**. After motor burnout, the flight computer compares the rocket's real-time GPS position against a pre-loaded target trajectory. Corrections are computed and fed to control surfaces (fins or thrust vectoring, TBD) to minimize lateral deviation from the intended flight path.

Key guidance elements:
- **NEO-M9N** provides 5 Hz position/velocity fixes with 1 PPS timing pulse for EKF time synchronization
- **ICM-20948** runs at ~225 Hz to bridge GPS update gaps with dead-reckoning via the onboard EKF
- **BMP390** provides barometric altitude at 25 Hz for vertical channel augmentation
- Magnetometer cross-checks heading independent of GPS course-over-ground

## Firmware Architecture

The MK2 firmware (`src/Dagger_1_MK2_Code.cpp`) is a bring-up / sensor validation sketch. All sensor inits are non-fatal — the flight computer boots and logs status for any subset of connected hardware, making incremental protoboard assembly straightforward.

```
Loop at ~1 kHz:
  ├── IMU poll @ 200 Hz (getAGMT — accel + gyro + mag in one SPI burst)
  ├── Baro poll @ 25 Hz (forced-mode, ~20 ms conversion)
  ├── GPS @ 5 Hz via pvt_callback (UBX NAV-PVT packet)
  ├── Serial monitor @ 2 Hz (all sensor data or OFFLINE notices)
  └── OLED update @ 2 Hz (4-row 128×32 status display)
```

OLED layout (128×32, 4 rows):
```
I:OK B:OK G:OK S:OK      ← component live/offline status
 0.0  0.0  1.0g           ← IMU acceleration in g
 142m 21.3C               ← baro altitude and temperature
GPS 3D 8sv                ← GPS fix type and satellite count
```

## Build

Requires [PlatformIO](https://platformio.org/) with the Teensy platform.

```ini
platform = teensy
board = teensy41
framework = arduino
```

**Dependencies** (installed automatically by PlatformIO):
- `sparkfun/SparkFun 9DoF IMU Breakout - ICM 20948`
- `adafruit/Adafruit BMP3XX Library`
- `adafruit/Adafruit SSD1306` + GFX + BusIO
- `sparkfun/SparkFun u-blox GNSS Arduino Library`

**Upload:** Use the PlatformIO **Upload** button (→ in the VS Code status bar), not the VS Code Run button. Teensy uses the HalfKay USB bootloader — press the physical button on the board when prompted.

## Status

Currently in hardware bring-up. Wiring components to protoboard one by one. Guidance algorithm and EKF implementation are next after full sensor validation.
