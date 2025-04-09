#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <MPU9250_WE.h>
#include <Wire.h>
// #include <Servo.h>
#include <ESP32Servo.h>
// #include <driver/ledc.h>

#define MOTOR_PWM 4

#define SERVO_AL1 3
#define SERVO_AL2 2
#define SERVO_EL 5
Servo servoAL1, servoAL2, servoEL, esc;


// ==========Servo Configuration==================

const int neg_AL_offset = 10;

const int servoAL1_offset = 0;  // Adjust for AL1 neutral position
const int servoAL1_min = 50;  // Minimum angle
const int servoAL1_max = 120; // Maximum angle

const int servoAL2_offset = 0; // Adjust for AL2 neutral position
const int servoAL2_min = 50;   // Minimum angle
const int servoAL2_max = 120; // Maximum angle

const int servoEL_offset = 70;
const int servoEL_min = 0;
const int servoEL_max = 110; 



// #define KP_ROLL 2.0
// #define KI_ROLL 0.02
// #define KD_ROLL 0.5

// #define KP_PITCH 2.5
// #define KI_PITCH 0.03
// #define KD_PITCH 0.6

// #define FILTER_ALPHA 0.98
// #define SIGNAL_TIMEOUT 1000

uint8_t newMACAddress[] = {0x00, 0x1A, 0x2B, 0x3C, 0x4D, 0x5E};
#define MPU9250_ADDR 0x68
MPU9250_WE MPU9250 = MPU9250_WE(MPU9250_ADDR);

typedef struct Packet
{
  int16_t throttle;
  int16_t roll;
  int16_t pitch;
  bool armed;
  bool selfLevel;
} Packet;

Packet rxData = {0, 0, 0, true, true};
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

void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len)
{
  if (len == sizeof(Packet))
  {
    memcpy(&rxData, incomingData, sizeof(rxData));
    lastRecv = millis();
    // Serial.print("Received data: " + String(rxData.roll) + ", " + String(rxData.pitch) + ", " + String(rxData.throttle));

    if (!prevArmed && rxData.armed)
    {
      Serial.println("ARMED!");
      integralRoll = 0;
      integralPitch = 0;
    }
    prevArmed = rxData.armed;
  }
}

void setMotor(bool armed, int16_t throttle)
{
  int16_t power = armed ? (constrain(throttle, 0, 255)) : 0;
  Serial.println("Motor: " + String(power));
  // if(power >= 255){
  //   digitalWrite(MOTOR_PWM,HIGH);
  // }
  esc.write(power);
  // analogWrite(MOTOR_PWM, power);
  
}

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  pinMode(4, OUTPUT);
  setMotor(false, 0);

  if (!MPU9250.init())
  {
    Serial.println("MPU9250 does not respond");
  }
  else
  {
    Serial.println("MPU9250 is connected");
  }

  WiFi.mode(WIFI_STA);
  if (esp_wifi_set_mac(WIFI_IF_STA, newMACAddress) != ESP_OK)
  {
    Serial.println("Failed to set MAC address");
  }
  else
  {
    Serial.println("MAC address changed successfully");
  }

  delay(100);
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    while (1)
      ;
  }

  esp_now_register_recv_cb(OnDataRecv);
  // setupLEDC();

  servoAL1.attach(SERVO_AL1);
  servoAL2.attach(SERVO_AL2);
  servoEL.attach(SERVO_EL);
  esc.attach(MOTOR_PWM);

  esc.write(255);
  delay(2000);
  esc.write(0);
  
  lastPIDTime = millis();
}

void loop()
{

  setMotor(true, rxData.throttle);

  // Calculate servo angles with offsets and limits
  int elAngle = map(rxData.pitch, -255, 255, servoEL_min, servoEL_max) + servoEL_offset;
  elAngle = constrain(elAngle, 1, 179);

  int al2Angle = map(rxData.roll, -255, 255, servoAL2_min , servoAL2_max) + servoAL2_offset;
  al2Angle = constrain(al2Angle, 1, 179);

  int al1Angle = map(rxData.roll, 255, -255, servoAL1_min, servoAL1_max) + servoAL1_offset;
  al1Angle = constrain(al2Angle, 1, 179);

  // Write angles to servos
  servoAL1.write(al1Angle);
  servoAL2.write(al2Angle);
  servoEL.write(elAngle);

  Serial.printf(" arm %d ", rxData.armed);
  Serial.printf(" Thr %d ", rxData.throttle);
  Serial.printf(" elev %d ", rxData.pitch);
  Serial.printf(" a1 %d ", al1Angle);
  Serial.printf(" rudder %d ", rxData.roll);
  Serial.printf(" a2 %d ", al2Angle);
}