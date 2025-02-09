#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <MPU9250_WE.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <driver/ledc.h>

#define MOTOR_PWM 4
#define SERVO_AIL 8
#define SERVO_ELEV 10

#define KP_ROLL 2.0  
#define KI_ROLL 0.02 
#define KD_ROLL 0.5  

#define KP_PITCH 2.5  
#define KI_PITCH 0.03 
#define KD_PITCH 0.6  

#define FILTER_ALPHA 0.98  
#define SIGNAL_TIMEOUT 1000

#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_CHANNEL    LEDC_CHANNEL_0
#define LEDC_RESOLUTION LEDC_TIMER_12_BIT
#define LEDC_FREQ       1000

uint8_t newMACAddress[] = { 0x00, 0x1A, 0x2B, 0x3C, 0x4D, 0x5E };
#define MPU9250_ADDR 0x68
MPU9250_WE MPU9250 = MPU9250_WE(MPU9250_ADDR);
Servo aileron, elevator;

#pragma pack(push, 1)
struct Packet {
  int16_t throttle;
  int16_t roll;
  int16_t pitch;
  bool armed;
  bool selfLevel;
};
#pragma pack(pop)

Packet rxData = {0 ,0 ,0, true, true};
unsigned long lastRecv = 0;
bool prevArmed = false;

float rollError, pitchError;
float prevRollError, prevPitchError;
float integralRoll, integralPitch;
unsigned long lastPIDTime = 0;

float rollAngle = 0, pitchAngle = 0;

void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  if (len == sizeof(Packet)) {
    memcpy(&rxData, incomingData, sizeof(rxData));
    lastRecv = millis();
    Serial.println("Received data: " + String(rxData.roll) + ", " + String(rxData.pitch) + ", " + String(rxData.throttle));
    
    if (!prevArmed && rxData.armed) {
      Serial.println("ARMED!");
      integralRoll = 0;
      integralPitch = 0;
    }
    prevArmed = rxData.armed;
  }
}

void setupLEDC() {
  ledc_timer_config_t timer_conf = {
    .speed_mode = LEDC_MODE,
    .duty_resolution = LEDC_RESOLUTION,
    .timer_num = LEDC_TIMER,
    .freq_hz = LEDC_FREQ,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&timer_conf);

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
  int16_t power = armed ? constrain(throttle, 0, 255) : 0;
  ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, power);
  ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

  Serial.println("Motor: " + String(power));
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
   if(!MPU9250.init()){
    Serial.println("MPU9250 does not respond");
  }
  else{
    Serial.println("MPU9250 is connected");
  }
  WiFi.mode(WIFI_STA);
  if (esp_wifi_set_mac(WIFI_IF_STA, newMACAddress) != ESP_OK) {
    Serial.println("Failed to set MAC address");
  } else {
    Serial.println("MAC address changed successfully");
  }
  delay(100);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    while(1);
  }
  esp_now_register_recv_cb(OnDataRecv);
  setupLEDC();
  aileron.attach(SERVO_AIL);
  elevator.attach(SERVO_ELEV);
 
  lastPIDTime = millis();
}

void updateAngles() {
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastPIDTime) / 1000.0;
  lastPIDTime = currentTime;
  xyzFloat acc = MPU9250.getGValues();
  xyzFloat gyr = MPU9250.getGyrValues();
  float accelRoll = atan2(acc.y, acc.z) * 180.0 / PI;
  float accelPitch = atan2(-acc.x, acc.z) * 180.0 / PI;
  float gyroRollRate = gyr.x;
  float gyroPitchRate = gyr.y;
  rollAngle = FILTER_ALPHA * (rollAngle + gyroRollRate * deltaTime) + (1 - FILTER_ALPHA) * accelRoll;
  pitchAngle = FILTER_ALPHA * (pitchAngle + gyroPitchRate * deltaTime) + (1 - FILTER_ALPHA) * accelPitch;
  Serial.println("Angles: " + String(rollAngle) + ", " + String(pitchAngle));
}

void selfLevelControl() {
  updateAngles();
  float targetRoll = map(rxData.roll, -255, 255, -30, 30);
  float targetPitch = map(rxData.pitch, -255, 255, -30, 30);
  float deltaTime = (millis() - lastPIDTime) / 1000.0;
  lastPIDTime = millis();
  rollError = targetRoll - rollAngle;
  pitchError = targetPitch - pitchAngle;
  integralRoll += rollError * deltaTime;
  integralPitch += pitchError * deltaTime;
  float derivativeRoll = (rollError - prevRollError) / deltaTime;
  float derivativePitch = (pitchError - prevPitchError) / deltaTime;
  float rollCorrection = KP_ROLL * rollError + KI_ROLL * integralRoll + KD_ROLL * derivativeRoll;
  float pitchCorrection = KP_PITCH * pitchError + KI_PITCH * integralPitch + KD_PITCH * derivativePitch;
  prevRollError = rollError;
  prevPitchError = pitchError;
  aileron.write(constrain(90 + rollCorrection, 0, 180));
  elevator.write(constrain(90 + pitchCorrection, 0, 180));
  Serial.println("PID: " + String(rollCorrection) + ", " + String(pitchCorrection));
}

void loop() {
  // if (millis() - lastRecv > SIGNAL_TIMEOUT) {
  //   rxData.armed = false;
  //   setMotor(false, 0);
  // }
  Serial.printf(" armed %d",rxData.armed);
  Serial.printf(" Thr %d",rxData.throttle);
  Serial.printf(" elev %d",rxData.pitch);
  
  // if (rxData.armed) {
  setMotor(true, rxData.throttle);
  // } else {
    // setMotor(false, 0);
  // }
  // if (rxData.selfLevel) {
  //   selfLevelControl();
  // } else {
    // aileron.write(map(rxData.roll, -255, 255, 0, 180));
    // elevator.write(map(rxData.pitch, -255, 255, 0, 180));
  //}
}
