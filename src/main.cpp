#include <Arduino.h>
#include <SoftwareSerial.h>
#include "L298N.h"

SoftwareSerial bt(2, 3); // RX, TX

// L298N motor driver
// L298N(IN1, IN2, ENA, IN3, IN4, ENB)
// Right motor (OUT1): IN1, IN2, ENA
// Left motor (OUT2): IN3, IN4, ENB 
L298N motor(4, 5, 6, 7, 8, 9);


void setup() {
  Serial.begin(9600);
  bt.begin(9600);
  
  motor.begin();
  motor.setSpeed(200);  // Set speed 0-255
}

void controlUsingSerial() {
  // if (Serial.available()) {
  //   char cmd = Serial.read();
  //   Serial.println(cmd);

  //   if (cmd == 'F')
  //     motor.forward();
  //   if (cmd == 'B')
  //     motor.backward();
  //   if (cmd == 'L')
  //     motor.turnLeft();
  //   if (cmd == 'R')
  //     motor.turnRight();
  //   if (cmd == 'S')
  //     motor.stop();
  // }
}

void loop() {
  controlUsingSerial();
  int humidity = random(0, 100);
  bt.print("Humidity: ");
  bt.print(humidity);
  bt.println("%");
  bt.print("\n");
  // Serial.println("Waiting for command...");
  bt.println("Waiting for command...");
  if (bt.available()) {
    char cmd = bt.read();
    Serial.println(cmd);

    if (cmd == 'F')
      motor.forward();
    if (cmd == 'B')
      motor.backward();
    if (cmd == 'L')
      motor.turnLeft();
    if (cmd == 'R')
      motor.turnRight();
    if (cmd == 'S')
      motor.stop();
  }
  delay(100);
}

