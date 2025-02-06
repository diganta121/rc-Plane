// code for plane esp32 c3
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <MPU9250_WE.h>
#include <ESP32Servo.h>
#include <Arduino.h>
#include <driver/ledc.h> // LEDC functions

// NRF24L01 Pins
#define CE_PIN 2
#define CSN_PIN 3
RF24 radio(CE_PIN, CSN_PIN);
const uint64_t address = 0xFCDBA0690;

// Motor/Servo Pins
#define MOTOR_PWM 5
#define SERVO_AIL 8
#define SERVO_ELEV 10

// PID Gains
#define KP_ROLL 2.0  
#define KI_ROLL 0.02 
#define KD_ROLL 0.5  

#define KP_PITCH 2.5  
#define KI_PITCH 0.03 
#define KD_PITCH 0.6  

// Complementary Filter
#define FILTER_ALPHA 0.98  
#define SIGNAL_TIMEOUT 1000

// LEDC Configuration
#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_CHANNEL    LEDC_CHANNEL_0
#define LEDC_RESOLUTION LEDC_TIMER_12_BIT
#define LEDC_FREQ       1000

MPU9250_WE mpu = MPU9250_WE(0x68);
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

// PID Variables
float rollError, pitchError;
float prevRollError, prevPitchError;
float integralRoll, integralPitch;
unsigned long lastPIDTime = 0;

// Angle Estimation
float rollAngle = 0, pitchAngle = 0;

void setupLEDC() {
  // Configure LEDC timer
  ledc_timer_config_t timer_conf = {
    .speed_mode = LEDC_MODE,
    .duty_resolution = LEDC_RESOLUTION,
    .timer_num = LEDC_TIMER,
    .freq_hz = LEDC_FREQ,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&timer_conf);

  // Configure LEDC channel
  ledc_channel_config_t channel_conf = {
    .gpio_num = MOTOR_PWM,
    .speed_mode = LEDC_MODE,
    .channel = LEDC_CHANNEL,
    .timer_sel = LEDC_TIMER,
    .duty = 0,
    .hpoint = 0
  };
  ledc_channel_config(&channel_conf);
}

void setMotor(bool armed, int16_t throttle) {
  int16_t power = armed ? constrain(throttle, 0, 4095) : 0;
  ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, power);
  ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

void setup() {
  Serial.begin(115200);
  
  // Initialize NRF24
  radio.begin();
  radio.setPALevel(RF24_PA_MAX);
  radio.openReadingPipe(0, address);
  radio.startListening();

  // Configure motor PWM
  setupLEDC();

  // Attach servos
  aileron.attach(SERVO_AIL);
  elevator.attach(SERVO_ELEV);

  // Initialize MPU9250
  Wire.begin(6, 7); // SDA=GPIO6, SCL=GPIO7
  if(!mpu.init()){
    Serial.println("MPU9250 not found!");
    while(1);
  }
  mpu.setSampleRateDivider(5);

  mpu.setAccRange(MPU9250_ACC_RANGE_2G);
  mpu.enableAccDLPF(true);
  mpu.setAccDLPF(MPU9250_DLPF_1);

  mpu.setGyrRange(MPU9250_GYRO_RANGE_500);
  mpu.enableGyrDLPF();
  mpu.setGyrDLPF(MPU9250_DLPF_1);
  lastPIDTime = millis();
}

void updateAngles() {
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastPIDTime) / 1000.0;
  lastPIDTime = currentTime;

  // in g
  xyzFloat acc = mpu.getGValues();
  xyzFloat gyr = mpu.getGyrValues();

  // Accelerometer angles
  float accelRoll = atan2(acc.y, acc.z) * 180.0 / PI;
  float accelPitch = atan2(-acc.x, acc.z) * 180.0 / PI;

  // Gyro rates (already in deg/s)
  float gyroRollRate = gyr.x;
  float gyroPitchRate = gyr.y;

  // Complementary filter
  rollAngle = FILTER_ALPHA * (rollAngle + gyroRollRate * deltaTime) + 
             (1 - FILTER_ALPHA) * accelRoll;
  pitchAngle = FILTER_ALPHA * (pitchAngle + gyroPitchRate * deltaTime) + 
              (1 - FILTER_ALPHA) * accelPitch;
}

// Rest of the code remains the same as previous version...
// [Keep the selfLevelControl() and loop() functions unchanged]

void selfLevelControl() {
  updateAngles();

  // Convert transmitter inputs to target angles
  float targetRoll = map(rxData.roll, -2048, 2048, -30, 30);
  float targetPitch = map(rxData.pitch, -2048, 2048, -30, 30);

  // PID calculations
  float deltaTime = (millis() - lastPIDTime) / 1000.0;
  lastPIDTime = millis();

  rollError = targetRoll - rollAngle;
  pitchError = targetPitch - pitchAngle;

  integralRoll += rollError * deltaTime;
  integralPitch += pitchError * deltaTime;

  float derivativeRoll = (rollError - prevRollError) / deltaTime;
  float derivativePitch = (pitchError - prevPitchError) / deltaTime;

  float rollCorrection = KP_ROLL * rollError + 
                        KI_ROLL * integralRoll + 
                        KD_ROLL * derivativeRoll;
  
  float pitchCorrection = KP_PITCH * pitchError + 
                         KI_PITCH * integralPitch + 
                         KD_PITCH * derivativePitch;

  prevRollError = rollError;
  prevPitchError = pitchError;

  // Apply corrections
  aileron.write(constrain(90 + rollCorrection, 0, 180));
  elevator.write(constrain(90 + pitchCorrection, 0, 180));
}

void loop() {
  if(radio.available()){
    radio.read(&rxData, sizeof(rxData));
    lastRecv = millis();
    
    if(!prevArmed && rxData.armed){
      Serial.println("ARMED!");
      // Reset PID integrals when arming
      integralRoll = 0;
      integralPitch = 0;
    }
    prevArmed = rxData.armed;
  }

  // Failsafe
  if(millis() - lastRecv > SIGNAL_TIMEOUT){
    rxData.armed = false;
    setMotor(false, 0);
  }

  if(rxData.armed){
    setMotor(true, rxData.throttle);
  }
  if(rxData.selfLevel){
    selfLevelControl();
  }
  else{
    // Manual overwrite
    aileron.write(map(rxData.roll, -2048, 2048, 0, 180));
    elevator.write(map(rxData.pitch, -2048, 2048, 0, 180));
  }
  
}