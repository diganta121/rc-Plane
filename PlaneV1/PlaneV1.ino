#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <MPU9250.h>
#include <ESP32Servo.h>

#define CE_PIN 7
#define CSN_PIN 8
RF24 radio(CE_PIN, CSN_PIN);
const uint64_t address = 0xFCDBA0000;

// Motor & Servo Pins
#define MOTOR_C 12
#define MOTOR_L1 13
#define MOTOR_L2 14
#define MOTOR_R1 15
#define MOTOR_R2 16
#define SERVO_AIL 17
#define SERVO_ELEV 18
#define SERVO_RUD 19

// Configuration
#define DIFFERENTIAL_GAIN 0.5 // Adjust for turn responsiveness
#define SIGNAL_TIMEOUT 1000
MPU9250 mpu;
Servo aileron, elevator, rudder;

struct Packet {
  int16_t throttle;
  int16_t yaw;
  int16_t pitch;
  int16_t roll;
  bool armed;
  bool selfLevel;
  bool calibrate;
};

Packet rxData;
unsigned long lastRecv = 0;
bool prevArmed = false;

// Motor PWM Channels
const int motorChannels[5] = {0,1,2,3,4}; // C, L1, L2, R1, R2

void setup() {
  Serial.begin(115200);
  
  // Initialize NRF24
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MAX);
  radio.startListening();

  // Configure motors
  for(int i=0; i<5; i++) {
    ledcSetup(motorChannels[i], 1000, 12);
    ledcAttachPin(MOTOR_C + i, motorChannels[i]);
  }

  // Attach servos
  aileron.attach(SERVO_AIL);
  elevator.attach(SERVO_ELEV);
  rudder.attach(SERVO_RUD);

  // Initialize MPU9250
  if(!mpu.begin()) {
    Serial.println("MPU9250 not found!");
    while(1);
  }
  mpu.setAccelerometerRange(MPU9250_RANGE_4_G);
  mpu.setGyroRange(MPU9250_RANGE_500_DEG);
}

void setMotors(bool armed, int16_t throttle, int16_t yaw) {
  static int16_t center, left, right;
  
  if(armed) {
    center = constrain(throttle, 0, 4095);
    
    // Differential thrust calculation
    int16_t diff = yaw * DIFFERENTIAL_GAIN;
    left = constrain(throttle + diff, 0, 4095);
    right = constrain(throttle - diff, 0, 4095);
  } else {
    center = left = right = 0;
  }

  ledcWrite(motorChannels[0], center);  // Center motor
  ledcWrite(motorChannels[1], left);    // L1
  ledcWrite(motorChannels[2], left);    // L2
  ledcWrite(motorChannels[3], right);   // R1
  ledcWrite(motorChannels[4], right);   // R2
}

void loop() {
  if(radio.available()) {
    radio.read(&rxData, sizeof(rxData));
    lastRecv = millis();
    
    // Auto-disarm check
    if(!prevArmed && rxData.armed) Serial.println("ARMED!");
    prevArmed = rxData.armed;
  }

  // Failsafe disarm
  if(millis() - lastRecv > SIGNAL_TIMEOUT) {
    rxData.armed = false;
    setMotors(false, 0, 0);
  }

  // Control surfaces
  if(rxData.armed) {
    setMotors(true, rxData.throttle, rxData.yaw);
    aileron.write(map(rxData.roll, -2048, 2048, 0, 180));
    elevator.write(map(rxData.pitch, -2048, 2048, 0, 180));
    rudder.write(map(rxData.yaw, -2048, 2048, 0, 180));
    
    // Self-level mode (basic implementation)
    if(rxData.selfLevel) {
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);
      
      // Add PID stabilization here
    }
  }
}