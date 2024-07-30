# GyroHand-Control-Module
Hand microcontroller unit; yaw, pitch and roll axis orientation module for controlling select receiver modules (door module and a wheelchair module) with MPU6050 Bluetooth Control to detect orientation.


The project utilizes an Arduino board interfaced with an MPU6050 accelerometer and gyroscope sensor. The MPU6050 sensor provides orientation data which is used to generate control commands sent via Bluetooth to another device.

**Components>** 
Arduino board (compatible with the MPU6050 and SoftwareSerial library)
MPU6050 sensor module
HC-05 Bluetooth module

**Hardware Connection>**
Arduino Connections:
MPU6050 SDA to Arduino A4 pin
MPU6050 SCL to Arduino A5 pin
MPU6050 VCC to Arduino 5V
MPU6050 GND to Arduino GND
HC-05 RX to Arduino pin 11 (TX via voltage divider if needed)
HC-05 TX to Arduino pin 10 (RX via voltage divider if needed)

**Additional Hardware>**
LED connected to pin 13 (for activity indication)
Interrupt pin of the MPU6050 connected to pin 2

**Software Reqs>**
Arduino IDE
Libraries:
Wire.h
I2Cdev.h
MPU6050_6Axis_MotionApps20.h
SoftwareSerial.h

**usage>**
The MPU6050 should detect orientation and send corresponding control commands ('F', 'B', 'R', 'L', 'S') over Bluetooth to the receiver module

**Notes>**
Adjust sensitivity and command thresholds in the code as needed for your specific application.
Ensure proper voltage levels and connections to prevent hardware damage.

**Credits>**
This project utilizes the MPU6050 library by Jeff Rowberg for interfacing with the sensor.
