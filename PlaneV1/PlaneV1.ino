#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <MPU9250.h>
#include <ESP32Servo.h>

// NRF24L01 Pins
#define CE_PIN 2
#define CSN_PIN 3
RF24 radio(CE_PIN, CSN_PIN);
const uint64_t address = 0xFCDBA0000;

// Motor & Servo Pins
#define MOTOR_PWM 5
#define SERVO_AIL 8
#define SERVO_ELEV 10

// PID Gains (TUNE THESE)
#define KP_ROLL 2.0  
#define KI_ROLL 0.02 
#define KD_ROLL 0.5  

#define KP_PITCH 2.5  
#define KI_PITCH 0.03 
#define KD_PITCH 0.6  

// Complementary Filter Factor (TUNE THIS)
#define FILTER_ALPHA 0.98  

#define SIGNAL_TIMEOUT 1000

MPU9250 mpu;
Servo aileron, elevator;

struct Packet {
  int16_t throttle;
  int16_t yaw;
  int16_t pitch;
  int16_t roll;
  bool armed;
  bool selfLevel;
};

Packet rxData;
unsigned long lastRecv = 0;
bool prevArmed = false;

// Motor PWM Channel
const int motorChannel = 0;

// PID Variables
float rollError, pitchError;
float prevRollError, prevPitchError;
float integralRoll, integralPitch;
unsigned long lastPIDTime = 0;

// Angle Estimation
float rollAngle = 0, pitchAngle = 0;

void setup() {
  Serial.begin(115200);
  
  // Initialize NRF24
  radio.begin();
  radio.setPALevel(RF24_PA_MAX);
  radio.openReadingPipe(0, address);
  radio.startListening();

  // Configure motor PWM
  ledcSetup(motorChannel, 1000, 12);
  ledcAttachPin(MOTOR_PWM, motorChannel);

  // Attach servos
  aileron.attach(SERVO_AIL);
  elevator.attach(SERVO_ELEV);

  // Initialize MPU9250
  Wire.begin(6, 7); // I2C SDA = GPIO6, SCL = GPIO7
  if (!mpu.begin()) {
    Serial.println("MPU9250 not found!");
    while (1);
  }
  mpu.setAccelerometerRange(MPU9250_RANGE_4_G);
  mpu.setGyroRange(MPU9250_RANGE_500_DEG);

  lastPIDTime = millis(); // Initialize PID timer
}

void setMotor(bool armed, int16_t throttle) {
  int16_t power = armed ? constrain(throttle, 0, 4095) : 0;
  ledcWrite(motorChannel, power);
}

void updateAngles() {
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastPIDTime) / 1000.0; // Convert to seconds
  lastPIDTime = currentTime;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Get accelerometer-based angles
  float accelRoll = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
  float accelPitch = atan2(-a.acceleration.x, a.acceleration.z) * 180.0 / PI;

  // Get gyro-based change in angles
  float gyroRollRate = g.gyro.x * 180.0 / PI;
  float gyroPitchRate = g.gyro.y * 180.0 / PI;

  // Complementary Filter: Blend gyro & accel data
  rollAngle = FILTER_ALPHA * (rollAngle + gyroRollRate * deltaTime) + (1 - FILTER_ALPHA) * accelRoll;
  pitchAngle = FILTER_ALPHA * (pitchAngle + gyroPitchRate * deltaTime) + (1 - FILTER_ALPHA) * accelPitch;
}

void selfLevelControl() {
  updateAngles(); // Get filtered angles

  // Target angles (default 0°, but can be controlled by user input)
  float targetRoll = map(rxData.roll, -2048, 2048, -30, 30);  // Max ±30°
  float targetPitch = map(rxData.pitch, -2048, 2048, -30, 30); // Max ±30°

  // Compute errors
  rollError = targetRoll - rollAngle;
  pitchError = targetPitch - pitchAngle;

  // Time-based integration
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastPIDTime) / 1000.0;
  lastPIDTime = currentTime;

  integralRoll += rollError * deltaTime;
  integralPitch += pitchError * deltaTime;

  float derivativeRoll = (rollError - prevRollError) / deltaTime;
  float derivativePitch = (pitchError - prevPitchError) / deltaTime;

  float rollCorrection = KP_ROLL * rollError + KI_ROLL * integralRoll + KD_ROLL * derivativeRoll;
  float pitchCorrection = KP_PITCH * pitchError + KI_PITCH * integralPitch + KD_PITCH * derivativePitch;

  prevRollError = rollError;
  prevPitchError = pitchError;

  // Apply correction to servos
  int aileronPos = constrain(90 + rollCorrection, 0, 180);
  int elevatorPos = constrain(90 + pitchCorrection, 0, 180);

  aileron.write(aileronPos);
  elevator.write(elevatorPos);
}

void loop() {
  if (radio.available()) {
    radio.read(&rxData, sizeof(rxData));
    lastRecv = millis();
    
    // Auto-disarm check
    if (!prevArmed && rxData.armed) Serial.println("ARMED!");
    prevArmed = rxData.armed;
  }

  // Failsafe disarm
  if (millis() - lastRecv > SIGNAL_TIMEOUT) {
    rxData.armed = false;
    setMotor(false, 0);
  }

  // Control surfaces
  if (rxData.armed) {
    setMotor(true, rxData.throttle);

    if (rxData.selfLevel) {
      selfLevelControl();
    } else {
      // Manual control
      aileron.write(map(rxData.roll, -2048, 2048, 0, 180));
      elevator.write(map(rxData.pitch, -2048, 2048, 0, 180));
    }
  }
}
