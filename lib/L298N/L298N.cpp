#include "L298N.h"

L298N::L298N(uint8_t in1, uint8_t in2, uint8_t enA, uint8_t in3, uint8_t in4, uint8_t enB) {
    this->in1 = in1;  // Right motor
    this->in2 = in2;
    this->enA = enA;
    
    this->in3 = in3;  // Left motor
    this->in4 = in4;
    this->enB = enB;
    
    this->currentSpeed = 200;  // Default speed
}

void L298N::begin() {
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(enA, OUTPUT);
    
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);
    pinMode(enB, OUTPUT);
    
    // Set default speed
    analogWrite(enA, currentSpeed);
    analogWrite(enB, currentSpeed);
    
    // Stop motors initially
    stop();
}

void L298N::setSpeed(uint8_t speed) {
    currentSpeed = speed;
    analogWrite(enA, speed);
    analogWrite(enB, speed);
}

void L298N::forward() {
    rightMotorForward();
    leftMotorForward();
}

void L298N::backward() {
    rightMotorBackward();
    leftMotorBackward();
}

void L298N::turnLeft() {
    rightMotorForward();
    leftMotorBackward();
}

void L298N::turnRight() {
    rightMotorBackward();
    leftMotorForward();
}

void L298N::stop() {
    rightMotorStop();
    leftMotorStop();
}

// Right motor control (OUT1)
void L298N::rightMotorForward() {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
}

void L298N::rightMotorBackward() {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
}

void L298N::rightMotorStop() {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
}

// Left motor control (OUT2)
void L298N::leftMotorForward() {
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
}

void L298N::leftMotorBackward() {
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
}

void L298N::leftMotorStop() {
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
}

// Individual motor speed control
void L298N::setRightMotorSpeed(uint8_t speed) {
    analogWrite(enA, speed);
}

void L298N::setLeftMotorSpeed(uint8_t speed) {
    analogWrite(enB, speed);
}
