# NanoGlide — Gesture-Controlled Interaction Module

NanoGlide is a wearable hand control unit that detects yaw, pitch, and roll using an MPU6050 accelerometer/gyroscope and wirelessly transmits real-time control commands via Bluetooth to paired receiver modules.

---

## Overview

An Arduino Nano reads 3D orientation from an MPU6050 and translates it into directional control signals (`F`, `B`, `R`, `L`, `S`) transmitted over Bluetooth (HC-05).

**Applications:**
- Assistive technology (wheelchair direction control)
- Home automation (door access)
- Robotics and interactive systems

---

## Hardware

**Components:**
- Arduino Nano (or compatible)
- MPU6050 accelerometer + gyroscope
- HC-05 Bluetooth module
- LED indicator (pin 13)
- Optional: 3D-printed wearable housing

**Wiring:**

| MPU6050 | Arduino |
|---------|---------|
| SDA     | A4      |
| SCL     | A5      |
| VCC     | 5V      |
| GND     | GND     |
| INT     | Pin 2   |

| HC-05 | Arduino                          |
|-------|----------------------------------|
| RX    | Pin 11 (voltage divider if needed) |
| TX    | Pin 10                           |
| VCC   | 5V                               |
| GND   | GND                              |

---

## Software

Arduino IDE with the following libraries:
- `Wire.h`
- `I2Cdev.h`
- `MPU6050_6Axis_MotionApps20.h` (Jeff Rowberg)
- `SoftwareSerial.h`

---

## Usage

1. Upload firmware to the Arduino
2. Power on and pair HC-05 with the receiver
3. Hand orientation maps to: `F` Forward · `B` Backward · `R` Right · `L` Left · `S` Stop
4. Adjust sensitivity thresholds in firmware as needed

---

## Notes

- Ensure correct voltage levels on HC-05 RX/TX lines
- Secure all connections to avoid signal loss

---

## Demo

https://drive.google.com/file/d/1emj3J04ihiRBoaoa_5fszwuSHFGRbFfa/view
