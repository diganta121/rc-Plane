#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h> // Install correct MPU9250 library

// NRF24L01 Setup
RF24 radio(7, 8); // CE=GPIO7, CSN=GPIO8 (match transmitter)
const uint64_t address = 0xTRANS0000;

// Motor and Servo Pins
#define MOTOR_PIN 5
#define SERVO_PITCH_PIN 6
#define SERVO_ROLL_PIN 7

// PWM Configuration
#define PWM_FREQ 50
#define PWM_RES 12

// IMU Setup
Adafruit_MPU9250 mpu;
sensors_event_t accel, gyro, temp;

// Data Structure
struct Packet {
  uint16_t throttle;
  uint16_t yaw;
  uint16_t pitch;
  uint16_t roll;
  bool switchState;
};

Packet rxData;

void setup() {
  Serial.begin(115200);
  
  // Initialize NRF24
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MAX);
  radio.startListening();

  // Configure PWM
  ledcSetup(0, PWM_FREQ, PWM_RES); // Channel 0: Motor
  ledcAttachPin(MOTOR_PIN, 0);
  ledcSetup(1, PWM_FREQ, PWM_RES); // Channel 1: Pitch servo
  ledcAttachPin(SERVO_PITCH_PIN, 1);
  ledcSetup(2, PWM_FREQ, PWM_RES); // Channel 2: Roll servo
  ledcAttachPin(SERVO_ROLL_PIN, 2);

  // Initialize MPU9250
  if (!mpu.begin()) {
    Serial.println("MPU9250 not found!");
    while(1);
  }
  mpu.setAccelerometerRange(MPU9250_RANGE_4_G);
  mpu.setGyroRange(MPU9250_RANGE_500_DEG);
}

void loop() {
  if (radio.available()) {
    radio.read(&rxData, sizeof(rxData));

    // Convert throttle to PWM (1000-2000μs range)
    uint32_t motorPWM = map(rxData.throttle, 0, 4095, 205, 410); // 12-bit to PWM duty
    ledcWrite(0, motorPWM);

    // Convert servo values (-45° to +45°)
    uint32_t pitchPWM = map(rxData.pitch, 0, 4095, 205, 410);
    uint32_t rollPWM = map(rxData.roll, 0, 4095, 205, 410);
    ledcWrite(1, pitchPWM);
    ledcWrite(2, rollPWM);

    // Read IMU data (for future stabilization)
    mpu.getEvent(&accel, &gyro, &temp);
  }
}