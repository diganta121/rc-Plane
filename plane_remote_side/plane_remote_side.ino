// 
// remote side for rc plane

// remote side dig
#include <esp_now.h>
#include <WiFi.h>

const int deadzone = 5;

const int EAxisPin = 32;
const int TAxisPin = 33;
const int RAxisPin = 34;
const int LAxisPin = 35;

const int LSPButton = 21;
const int RSPButton = 22;

#define LED_PIN 19

int Tvalue = 0;
int Rvalue = 0;
int Evalue = 0;


bool prevArm = false;
// bool RFB = false;
// bool RBB = false;

// bool LFB = false;
// bool LBB = false;

int Rsp = 1;
int Lsp = 1;



// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = { 0x00, 0x1A, 0x2B, 0x3C, 0x4D, 0x5E };  // car

const int send_rate = 30;
int sendvar = 0;

// Structure example to send data
// Must match the receiver structure


typedef struct struct_message {
  int16_t throttle;
  int16_t roll;
  int16_t pitch;
  bool armed;
  bool selfLevel;
} struct_message;

// Create a struct_message called Data & peer
struct_message Data = {0, 0, 0, true, false};
// struct_message PrevData;
esp_now_peer_info_t peerInfo;

// Variables to store default joystick positions (calibration offsets)
int defaultTvalue = 0;
int defaultElvalue = 0;
int defaultRvalue = 0;


// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
  status == ESP_NOW_SEND_SUCCESS ? digitalWrite(LED_PIN, HIGH) : errBlink();
}

void errBlink(){
  for (int i = 0; i < 3; i++) {  // Blink LED for failure
    digitalWrite(LED_PIN, HIGH);
    delay(50);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
}

// Function to calibrate joystick
void calibrateJoystick() {
    int totalT = 0, totalEl = 0,totalR=0;
    int samples = 20; // Number of samples to average for calibration

    Serial.println("Calibrating joystick...");
    for (int i = 0; i < samples; i++) {
        totalT += analogReadSmooth(TAxisPin);
        totalEl += analogReadSmooth(EAxisPin);
        totalR += analogReadSmooth(RAxisPin);
        delay(50); // Small delay between samples
    }

    // Calculate average default position
    defaultTvalue = 2047 -(totalT / samples);
    defaultElvalue = 2047- (totalEl / samples);
    defaultRvalue = 2047 - (totalR / samples);


    Serial.print("Default RAxis: ");
    Serial.println(defaultTvalue);
    Serial.print("Default LAxis: ");
    Serial.println(defaultElvalue);
}
int climit(int n) {
  if (n >= 255) {
    return 255;
  } 
  else if (n <= -255) {
    return -255;
  } 
  else {
    return n;
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  pinMode(TAxisPin, INPUT);
  pinMode(RAxisPin, INPUT);
  pinMode(EAxisPin, INPUT);

  // pinMode(RBButton, INPUT);
  // pinMode(LBButton, INPUT);

  pinMode(RSPButton, INPUT);
  pinMode(LSPButton, INPUT);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  delay(200);
  digitalWrite(LED_PIN, LOW);
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  
  // Calibrate joystick
  calibrateJoystick();
}


bool button_state(int a) {
  int s = 0;
  for (int i = 0; i < 4; i++) {
    s += digitalRead(a);
    delay(3);
  }
  return (s > 2) ? true : false;
}

int analogReadSmooth(int pin) {
    int total = 0;
    for (int i = 0; i < 3; i++) {
        total += analogRead(pin);
        delay(5);
    }
    return total / 3;
}




int stick_value(int sp) {
  int abs_sp = abs(sp);
  int output = 0;

  if (abs_sp <= deadzone) {  // Deadzone
    output = 0;
  } else if (abs_sp <= 255) {                // analog range
    output = map(abs_sp, deadzone, 255, 20, 255);  // Scale to 70-280
  }
  else{
    output = 255;
  }

  if (sp <0){
    output *= -1;
  }
  return output;
}

void loop() {
  Tvalue = analogReadSmooth(TAxisPin) + defaultTvalue;
  Evalue = analogReadSmooth(EAxisPin) + defaultElvalue;
  Rvalue = analogReadSmooth(RAxisPin) + defaultRvalue;

  Rsp = !button_state(RSPButton);
  Lsp = !button_state(LSPButton);

  Serial.print(Tvalue);
  Serial.print(" ");
  Serial.print(Evalue);
  Serial.print(" ");
  Serial.print(Rvalue);
  Serial.print(" ");
  Tvalue = climit(stick_value(map(Tvalue,0,4095,255,-255))); // reversed //stick_value(Rvalue);
  Evalue = climit(stick_value(map(Evalue,0,4095,-255,255))); //stick_value(Lvalue);
  Rvalue = climit(stick_value(map(Rvalue,0,4095,255,-255)));

  if (Rsp){
    // Data.armed =!Data.armed;
    Evalue = Evalue/2;
    Rvalue = Rvalue/2;
    Serial.print(" arm ");
  }
  if (Lsp) {
    Evalue = Evalue/2;
  }

  Data.throttle = Tvalue;
  Data.pitch = Evalue;
  Data.roll = Rvalue;
  
  Serial.print(Tvalue);
  Serial.print(" ");
  Serial.print(Evalue);
  Serial.print(" ");
  Serial.print(Rvalue);
  Serial.print(" ");
  Serial.println(Data.armed);

  // === Send message via ESP-NOW ===
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&Data, sizeof(Data));

  if (result == ESP_OK) {
    Serial.println("S");
  } else {
    Serial.println("E");
    delay(70);
  }

  delay(10);
}

