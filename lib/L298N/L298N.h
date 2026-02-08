#ifndef L298N_H
#define L298N_H

#include <Arduino.h>

class L298N {
private:
    // Motor A (Right side - OUT1)
    uint8_t in1;
    uint8_t in2;
    uint8_t enA;
    
    // Motor B (Left side - OUT2)
    uint8_t in3;
    uint8_t in4;
    uint8_t enB;
    
    uint8_t currentSpeed;

public:
    // Constructor
    L298N(uint8_t in1, uint8_t in2, uint8_t enA, uint8_t in3, uint8_t in4, uint8_t enB);
    
    // Initialization
    void begin();
    
    // Speed control (0-255)
    void setSpeed(uint8_t speed);
    
    // Movement functions
    void forward();
    void backward();
    void turnLeft();
    void turnRight();
    void stop();
    
    // Individual motor control
    void rightMotorForward();
    void rightMotorBackward();
    void rightMotorStop();
    
    void leftMotorForward();
    void leftMotorBackward();
    void leftMotorStop();
    
    // Individual motor speed control
    void setRightMotorSpeed(uint8_t speed);
    void setLeftMotorSpeed(uint8_t speed);
};

#endif
