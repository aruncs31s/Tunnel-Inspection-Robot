#include <Arduino.h>
#include <SoftwareSerial.h>
#include "L298N.h"
#include "touch_sensor.cpp"
#include <DHT.h>
#include "pins.h"

#define DHTPIN 8     // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11   // DHT 11
#define BAUD_RATE 9600

SoftwareSerial bt(2, 3); // RX, TX

// L298N motor driver
// L298N(IN1, IN2, ENA, IN3, IN4, ENB)
// Right motor (OUT1): IN1, IN2, ENA
// Left motor (OUT2): IN3, IN4, ENB 

// L298N motor(4, 5, 8, 6, 7,9);
L298N motor(IN1, IN2, ENA, IN3, IN4, ENB);

TouchSensor ts(TOUCH_SENSOR_PIN);

DHT dht;

// MQ6 Sensor variables
float Referance_V = 5000.0; /* Arduino Uno Reference Voltage in mV (5V) */
float RL = 1.0;             /* In Module RL value is 1k Ohm */
float Ro = 10.0;            /* The Ro value is 10k Ohm */
float mVolt = 0.0;
const float Ro_clean_air_factor = 10.0;
bool mq6_calibrated = false;

void send_readings();
void calibrateMQ6();
float getMQ6Voltage();
float calculateRs(float Vo);
unsigned int calculateLPG_PPM(float RsRo_ratio);
float get_mq7_readings() ;
void setup_mq7();
void setup() {
  Serial.begin(BAUD_RATE);

  bt.begin(9600);
  setup_mq7();
  dht.setup(DHT_PIN);
  motor.begin();
  
  motor.setSpeed(200);  // Set speed 0-255
  
  // MQ6 Sensor setup
  pinMode(MQ6_PIN, INPUT);
  bt.println("MQ6 Sensor warming up... Please wait 30 seconds");
  Serial.println("MQ6 Sensor warming up... Please wait 30 seconds");
  delay(1000);  // 30 second warmup
  bt.println("MQ6 Warmup Complete. Calibrating...");
  Serial.println("MQ6 Warmup Complete. Calibrating...");
  
  // Calibrate MQ6 sensor
  calibrateMQ6();
  mq6_calibrated = true;
  bt.println("MQ6 Calibration Complete!");
  Serial.println("MQ6 Calibration Complete!");
}

unsigned long lastSendTime = 0;
const unsigned long sendInterval = 5000; // Send readings every 5 seconds
void loop() {
  ts.read();
  if (ts.isTouched()) {
    Serial.println("Touch detected! Stopping motors.");
    motor.stop();
    delay(1000); // Debounce delay
  }

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
  send_readings();
}

void send_readings() {
  if (millis() - lastSendTime < sendInterval) {
    return; // Skip sending if within the interval
  }
  int touch_value = ts.isTouched() ? 1 : 0;
  bt.print("Touch: ");
  bt.println(touch_value);

  float h = dht.getHumidity();
  float t = dht.getTemperature();
  bt.print("Temp: ");
  bt.print(t);
  bt.print(" C, Humidity: ");
  bt.print(h);
  bt.println(" %");
  
  // MQ6 Sensor readings
  if (mq6_calibrated) {
    float mq6_voltage = getMQ6Voltage();
    float Rs = calculateRs(mq6_voltage);
    float ratio_RsRo = Rs / Ro;
    unsigned int lpg_ppm = calculateLPG_PPM(ratio_RsRo);
    bt.print("MQ6 Voltage: ");
    bt.print(mq6_voltage);
    bt.println(" mV");
    
    bt.print("MQ6 Rs: ");
    bt.println(Rs);
    
    bt.print("MQ6 Rs/Ro: ");
    bt.println(ratio_RsRo);
    
    bt.print("LPG PPM: ");
    bt.println(lpg_ppm);
  } else {
    bt.println("MQ6 Sensor not calibrated yet");
  }
  // MQ7 Sensor reading
  float mq7_voltage = get_mq7_readings();
  bt.print("MQ7 Voltage: ");
  bt.print(mq7_voltage);
  bt.println(" V");
  lastSendTime = millis();
}

// MQ6 Sensor Functions
void calibrateMQ6() {
  mVolt = 0.0;
  for(int i = 0; i < 30; i++) {
    mVolt += getMQ6Voltage();
    delay(100);
  }
  mVolt = mVolt / 30.0;  // Average voltage for 30 samples
  
  Serial.print("Calibration Voltage: ");
  Serial.print(mVolt);
  Serial.println(" mV");
  
  float Rs_cal = calculateRs(mVolt);
  Serial.print("Calibration Rs: ");
  Serial.println(Rs_cal);
  
  Ro = Rs_cal / Ro_clean_air_factor;
  Serial.print("Ro (calibrated): ");
  Serial.println(Ro);
  
  bt.print("Calibration Voltage: ");
  bt.print(mVolt);
  bt.println(" mV");
  bt.print("Ro (calibrated): ");
  bt.println(Ro);
}

float getMQ6Voltage() {
  // Arduino Uno has 10-bit ADC (0-1023), reference voltage 5V
  int adc_value = analogRead(MQ6_PIN);
  float voltage = (adc_value * Referance_V) / 1024.0;
  delay(1);
  return voltage;
}

float calculateRs(float Vo) {
  // Rs = (Vc - Vo) * (RL / Vo)
  float Rs = (Referance_V - Vo) * (RL / Vo);
  return Rs;
}

unsigned int calculateLPG_PPM(float RsRo_ratio) {
  // LPG ppm = [(Rs/Ro)/18.446]^(1/-0.421)
  float ppm = pow((RsRo_ratio / 18.446), (1 / -0.421));
  return (unsigned int)ppm;
  
  // MQ6 Sensor readings
  if (mq6_calibrated) {
    float mq6_voltage = getMQ6Voltage();
    float Rs = calculateRs(mq6_voltage);
    float ratio_RsRo = Rs / Ro;
    unsigned int lpg_ppm = calculateLPG_PPM(ratio_RsRo);
    
    bt.print("MQ6 Voltage: ");
    bt.print(mq6_voltage);
    bt.println(" mV");
    
    bt.print("MQ6 Rs: ");
    bt.println(Rs);
    
    bt.print("MQ6 Rs/Ro: ");
    bt.println(ratio_RsRo);
    
    bt.print("LPG PPM: ");
    bt.println(lpg_ppm);
  } else {
    bt.println("MQ6 Sensor not calibrated yet");
  }
}




void setup_mq7(){
  pinMode(MQ7_PIN, INPUT);
}

float get_mq7_readings() {
  int analogValue = analogRead(MQ7_PIN);
  float voltage = analogValue * (5.0 / 1023.0);

  Serial.print("MQ7 A3 Raw: ");
  Serial.print(analogValue);
  Serial.print(" | Voltage: ");
  Serial.println(voltage);
  delay(1000);
  return voltage;
}