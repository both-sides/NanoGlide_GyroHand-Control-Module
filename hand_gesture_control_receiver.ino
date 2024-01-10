#include <SoftwareSerial.h>
SoftwareSerial BTSerial(10, 11); // CONNECT BT RX PIN TO ARDUINO 11 PIN | CONNECT BT TX PIN TO ARDUINO 10 PIN | BTSerial (RX, TX) of the Arduino

char tiltDirection;
int motorInput1 = 2;
int motorInput2 = 3;
int motorInput3 = 4;
int motorInput4 = 5;

void setup() {

  pinMode(motorInput1, OUTPUT);
  pinMode(motorInput2, OUTPUT);
  pinMode(motorInput3, OUTPUT);
  pinMode(motorInput4, OUTPUT);

  digitalWrite(motorInput1, LOW);
  digitalWrite(motorInput2, LOW);
  digitalWrite(motorInput3, LOW);
  digitalWrite(motorInput4, LOW);

  Serial.begin(38400);      // Serial communication is activated at 38400 baud/s.
  BTSerial.begin(38400);    // HC-05 default speed in AT command more
}

void loop() {
  if (BTSerial.available()) {   // Witing for data incoming from the other XBee module
    tiltDirection = BTSerial.read();
    if(tiltDirection == 'F'){
      Serial.println("Backward");
      backward();
    }else if(tiltDirection == 'B'){
      Serial.println("Forward");
      forward();
    }else if(tiltDirection == 'R'){
      Serial.println("Left");
      left();
    }else if(tiltDirection == 'L'){
      Serial.println("Right");
      right();
    }else if(tiltDirection == 'S'){
      Serial.println("Stop");
      stopCar();
    }
  }
}

void forward()
{
  digitalWrite(motorInput1, LOW);
  digitalWrite(motorInput2, HIGH);
  digitalWrite(motorInput3, LOW);
  digitalWrite(motorInput4, HIGH);
}
void backward()
{
  digitalWrite(motorInput1, HIGH);
  digitalWrite(motorInput2, LOW);
  digitalWrite(motorInput3, HIGH);
  digitalWrite(motorInput4, LOW);
}

void right()
{
  digitalWrite(motorInput1, LOW);
  digitalWrite(motorInput2, HIGH);
  digitalWrite(motorInput3, LOW);
  digitalWrite(motorInput4, LOW);
}

void left()
{
  digitalWrite(motorInput1, LOW);
  digitalWrite(motorInput2, LOW);
  digitalWrite(motorInput3, LOW);
  digitalWrite(motorInput4, HIGH);
}

void stopCar() {
  digitalWrite(motorInput1, LOW);
  digitalWrite(motorInput2, LOW);
  digitalWrite(motorInput3, LOW);
  digitalWrite(motorInput4, LOW);
}
