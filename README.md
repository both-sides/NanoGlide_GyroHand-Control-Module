# 🖐️ NanoGlide – Gesture-Controlled Interaction Module

NanoGlide is a **wearable hand control unit** that detects **yaw, pitch, and roll** orientation using an **MPU6050** accelerometer/gyroscope, and wirelessly transmits real-time control commands via Bluetooth to receiver modules (e.g., door module, wheelchair control).

## 🚀 Overview
The project uses an **Arduino Nano** connected to an **MPU6050** motion sensor. The sensor measures 3D orientation, and the firmware translates this into control signals (`F`, `B`, `R`, `L`, `S`) sent over Bluetooth (HC-05) to paired devices.

Applications include:
- **Assistive technology** (wheelchair direction control)
- **Home automation** (door access)
- **Robotics & interactive systems**

## 🔧 Components
- Arduino Nano (or compatible)
- MPU6050 accelerometer + gyroscope
- HC-05 Bluetooth module
- LED indicator (on pin 13)
- Optional: 3D-printed wearable housing

## 🔌 Hardware Connections
**MPU6050 → Arduino**
- SDA → A4
- SCL → A5
- VCC → 5V
- GND → GND
- INT → Pin 2

**HC-05 → Arduino**
- RX → Pin 11 (through voltage divider if needed)
- TX → Pin 10
- VCC → 5V
- GND → GND

## 💻 Software Requirements
- Arduino IDE
- Libraries:
  - `Wire.h`
  - `I2Cdev.h`
  - `MPU6050_6Axis_MotionApps20.h`
  - `SoftwareSerial.h`

## ▶️ Usage
1. Upload firmware to Arduino.
2. Power on the module and pair the HC-05 with the receiver.
3. Move your hand — NanoGlide will send:
   - `F` = Forward  
   - `B` = Backward  
   - `R` = Right  
   - `L` = Left  
   - `S` = Stop  
4. Adjust sensitivity and thresholds in the code for your needs.

## ⚠️ Notes
- Ensure proper voltage levels for HC-05 RX/TX.
- Secure connections to avoid data loss.
- The MPU6050 library by **Jeff Rowberg** is used for sensor interfacing.

## 📸 Demo
*https://drive.google.com/file/d/1emj3J04ihiRBoaoa_5fszwuSHFGRbFfa/view*
