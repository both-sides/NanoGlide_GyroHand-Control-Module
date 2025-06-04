#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <SoftwareSerial.h>

SoftwareSerial BTSerial(10, 11); // CONNECT BT RX PIN TO ARDUINO 11 PIN | CONNECT BT TX PIN TO ARDUINO 10 PIN

constexpr uint8_t INTERRUPT_PIN = 2; // Interrupt on D2
constexpr uint8_t LED_PIN = 13;

MPU6050 mpu;
bool blinkState = false;

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];

enum GestureState
{
  GESTURE_STOP = 'S',
  GESTURE_FORWARD = 'F',
  GESTURE_BACKWARD = 'B',
  GESTURE_LEFT = 'L',
  GESTURE_RIGHT = 'R'
};

constexpr float RAD_TO_DEG = 180.0 / M_PI;
constexpr int16_t PITCH_YAW_THRESHOLD = 100;
constexpr int16_t ROLL_MIN = 45;
constexpr int16_t ROLL_MAX = 55;
constexpr int16_t PITCH_MIN = 145;
constexpr int16_t PITCH_MAX = 155;

volatile bool mpuInterrupt = false;

void dmpDataReady()
{
  mpuInterrupt = true;
}

constexpr int mapGestureValue(float value, int fromLow, int fromHigh, int toLow, int toHigh)
{
  return static_cast<int>((value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow);
}

void sendGestureCommand(GestureState gesture)
{
  BTSerial.write(static_cast<uint8_t>(gesture));
}

void setup()
{
  Fastwire::setup(400, true);

  Serial.begin(38400);
  BTSerial.begin(38400);

  while (!Serial)
    ;

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(126);
  mpu.setYGyroOffset(57);
  mpu.setZGyroOffset(-69);
  mpu.setZAccelOffset(1869);

  if (devStatus == 0)
  {
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else
  {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  pinMode(LED_PIN, OUTPUT);
}

void loop()
{
  if (!dmpReady)
    return;

  while (!mpuInterrupt && fifoCount < packetSize)
  {
  }

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
  }
  else if (mpuIntStatus & 0x02)
  {
    while (fifoCount < packetSize)
      fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    float yaw = ypr[0] * RAD_TO_DEG;
    float pitch = ypr[1] * RAD_TO_DEG;
    float roll = ypr[2] * RAD_TO_DEG;

    int mappedRoll = mapGestureValue(roll, -PITCH_YAW_THRESHOLD, PITCH_YAW_THRESHOLD, 0, 100);
    int mappedPitch = mapGestureValue(pitch, -PITCH_YAW_THRESHOLD, PITCH_YAW_THRESHOLD, 100, 200);

    Serial.print(mappedRoll);
    Serial.print("\t");
    Serial.println(mappedPitch);

    if (mappedRoll >= ROLL_MIN && mappedRoll <= ROLL_MAX && mappedPitch >= PITCH_MIN && mappedPitch <= PITCH_MAX)
    {
      sendGestureCommand(GESTURE_STOP);
    }
    else if (mappedRoll > ROLL_MAX)
    {
      sendGestureCommand(GESTURE_RIGHT);
    }
    else if (mappedRoll < ROLL_MIN)
    {
      sendGestureCommand(GESTURE_LEFT);
    }
    else if (mappedPitch > PITCH_MAX)
    {
      sendGestureCommand(GESTURE_BACKWARD);
    }
    else if (mappedPitch < PITCH_MIN)
    {
      sendGestureCommand(GESTURE_FORWARD);
    }

    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }
}
