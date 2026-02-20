#include "pins.h"   


class TouchSensor {
public:
  TouchSensor(int pin) : _pin(pin) {
    Serial.println("Initializing Touch Sensor...");
    Serial.print("Pin: ");
    Serial.println(_pin);
    pinMode(_pin, INPUT);
  }
  void read() {
    unsigned long currentMs = millis();
    if (currentMs - _lastReadTime < _readInterval) {
      return; // Skip reading if within the interval
    }
    int value = analogRead(_pin);
    Serial.print("Touch Sensor Value: ");
    Serial.println(value);
    _lastReadTime = currentMs;
    _adc_value = value;
  }
  bool isTouched() {
    bool touch = _adc_value < _threshold;
    _adc_value = 0;
  }

private:
int _pin;
// Only Read after 10 seconds to allow sensor to stabilize

unsigned long _lastReadTime = 0;
// 10 Sec Timeout
unsigned long _readInterval = 10000;
int _adc_value;
int _threshold = 500; 
};

