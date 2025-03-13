#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <MPU9250_WE.h>
#include <Wire.h>
// #include <Servo.h>
#include <ESP32Servo.h>
// #include <driver/ledc.h>

#define MOTOR_PWM 4

#define SERVO_EL1 3
#define SERVO_EL2 2
Servo servoEL1, servoEL2;

// Servo Configuration
const int servoEL1_offset = 0;  // Adjust for EL1 neutral position
const int servoEL2_offset = 70;  // Adjust for EL2 neutral position

const int servoEL1_min = 15;     // Minimum angle for EL1 (degrees)
const int servoEL1_max = 160;   // Maximum angle for EL1 (degrees)

const int servoEL2_min = 0;     // Minimum angle for EL2 (degrees)
const int servoEL2_max = 110;   // Maximum angle for EL2 (degrees)

// #define KP_ROLL 2.0  
// #define KI_ROLL 0.02 
// #define KD_ROLL 0.5  

// #define KP_PITCH 2.5  
// #define KI_PITCH 0.03 
// #define KD_PITCH 0.6  

// #define FILTER_ALPHA 0.98  
// #define SIGNAL_TIMEOUT 1000

// #define LEDC_MODE       LEDC_LOW_SPEED_MODE
// #define LEDC_TIMER      LEDC_TIMER_0
// #define LEDC_CHANNEL    LEDC_CHANNEL_0
// #define LEDC_RESOLUTION LEDC_TIMER_8_BIT
// #define LEDC_FREQ       500

uint8_t newMACAddress[] = { 0x00, 0x1A, 0x2B, 0x3C, 0x4D, 0x5E };
#define MPU9250_ADDR 0x68
MPU9250_WE MPU9250 = MPU9250_WE(MPU9250_ADDR);


typedef struct Packet {
  int16_t throttle;
  int16_t roll;
  int16_t pitch;
  bool armed;
  bool selfLevel;
} Packet;


Packet rxData = {0 ,0 ,0, true, true};
unsigned long lastRecv = 0;
bool prevArmed = false;

float rollError, pitchError;
float prevRollError, prevPitchError;
float integralRoll, integralPitch;
unsigned long lastPIDTime = 0;

float rollAngle = 0, pitchAngle = 0;

// Custom analogWrite function using LEDC
// void analogWrite(uint8_t pin, int value) {
//   ledcWrite(LEDC_CHANNEL, value);
// }

void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  if (len == sizeof(Packet)) {
    memcpy(&rxData, incomingData, sizeof(rxData));
    lastRecv = millis();
    //Serial.print("Received data: " + String(rxData.roll) + ", " + String(rxData.pitch) + ", " + String(rxData.throttle));

    if (!prevArmed && rxData.armed) {
      Serial.println("ARMED!");
      integralRoll = 0;
      integralPitch = 0;
    }
    prevArmed = rxData.armed;
  }
}

void setMotor(bool armed, int16_t throttle) {
  int16_t power = armed ? (255 - constrain(throttle, 0, 255)) : 255;
  Serial.println("Motor: " + String(power));
  // if(power >= 255){
  //   digitalWrite(MOTOR_PWM,HIGH);
  // }
  
  analogWrite(MOTOR_PWM, power);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  pinMode(4,OUTPUT);
  setMotor(false, 0);

  if (!MPU9250.init()) {
    Serial.println("MPU9250 does not respond");
  } else {
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
    while (1);
  }

  esp_now_register_recv_cb(OnDataRecv);
  //setupLEDC();
  
  servoEL1.attach(SERVO_EL1);
  servoEL2.attach(SERVO_EL2);

  lastPIDTime = millis();
}

void loop() {
  
  setMotor(true, rxData.throttle);
  
  // Calculate servo angles with offsets and limits
  int el1Angle = map(rxData.pitch,-255,255,servoEL1_min,servoEL1_max) + servoEL1_offset;
  el1Angle = constrain(el1Angle, 1, 179);

  int el2Angle = map(rxData.roll,-255,255,servoEL2_min,servoEL2_max) + servoEL2_offset;
  el2Angle = constrain(el2Angle, 1, 179);

  // Write angles to servos
  servoEL1.write(el1Angle);
  servoEL2.write(el2Angle);

  Serial.printf(" arm %d ", rxData.armed);
  Serial.printf(" Thr %d ", rxData.throttle);
  Serial.printf(" elev %d ", rxData.pitch);
  Serial.printf(" a1 %d ", el1Angle);
  Serial.printf(" rudder %d ", rxData.roll);
  Serial.printf(" a2 %d ", el2Angle);
}