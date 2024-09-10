
#include <Wire.h>
#include "MAX30105.h"
#include "esp_now.h"
#include "WiFi.h"
#include "heartRate.h"

MAX30105 particleSensor;


typedef struct struct_message {  // Must match the receiver structure
  bool trigger;
  int BPM;
  float O2;
  bool fallDetect;
} struct_message;

esp_now_peer_info_t peerInfo;

long lastBeat = 0;  //Time at which the last beat occurred
int counter = 0;
float finalO2Value = 0;
const int MPU_addr = 0x68;  // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
boolean fall = false;      //stores if a fall has occurred
boolean trigger1 = false;  //stores if first trigger (lower threshold) has occurred
boolean trigger2 = false;  //stores if second trigger (upper threshold) has occurred
boolean trigger3 = false;  //stores if third trigger (orientation change) has occurred
byte trigger1count = 0;    //stores the counts past since trigger 1 was set true
byte trigger2count = 0;    //stores the counts past since trigger 2 was set true
byte trigger3count = 0;    //stores the counts past since trigger 3 was set true
int angleChange = 0;

int finalBPM;
bool trigger = false;
uint8_t broadcastAddress[] = { 0x58, 0xBF, 0x25, 0x33, 0x29, 0xB8 };
struct_message myData = { false, 0, 0, 0 };
struct_message temp = { false, 0, 0, 0 };

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len);
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void mpu_triggers();
void mpu_read();
void RunBpmO2();

void setup() {
  Serial.begin(115200);

  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST))  //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    //while (1);
  }
  particleSensor.setup();                     //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A);  //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0);   //Turn off Green LED

  WiFi.mode(WIFI_STA);  // Set device as a Wi-Fi Station

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
  // Once ESPNow is successfully Init, we will register for Send CB to get the status of Transmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
  }

  Serial.println("Setup done!");
}


void loop() {
  if (trigger == true) {

    RunBpmO2();
    if (myData.O2 != 0 && myData.BPM != 0) {
      esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
      Serial.println(myData.O2);
      Serial.println(myData.BPM);
      myData.O2 = 0;
      myData.BPM = 0;
      trigger = false;
    }
  } else {
    mpu_read();
    mpu_triggers();
  }
}

void RunBpmO2() {
  int infrared = particleSensor.getIR();
  int red = particleSensor.getRed();
  if (checkForBeat(infrared)) {
    if (counter == 0)
      Serial.println("Starting the pulse reading!");

    long delta = millis() - lastBeat;
    lastBeat = millis();
    int beatsPerMinute = 60 / (delta / 1000.0);

    float ratio = infrared / red;
    float percentageFloat = (ratio - 0.7f) / 0.3f * 100.0f;

    if (percentageFloat < 0.0f) {
      percentageFloat = 0;
    } 
    else if (percentageFloat > 100.0f) {
      percentageFloat = 100;
    }
    Serial.println(percentageFloat);
    finalO2Value += percentageFloat;
    finalBPM += beatsPerMinute;
    counter += 1;
  }
  if (counter == 20) {
    finalBPM /= 20;
    finalO2Value /= 20;
    myData.BPM = finalBPM;
    myData.trigger = false;
    myData.O2 = finalO2Value;

    Serial.print("The average BPM recorder is: ");
    Serial.println(finalBPM);
    finalBPM = 0;
    finalO2Value = 0;
    counter = 0;
  }
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  Serial.println("Information received!");
  memcpy(&temp, incomingData, sizeof(temp));
  if (temp.trigger == 1)
    trigger = 1;
}


// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Last Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  trigger = false;
}

void mpu_read() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  delay(1);
  Wire.requestFrom(0x68, 14, true);      // request a total of 14 registers
  AcX = Wire.read() << 8 | Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp = Wire.read() << 8 | Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX = Wire.read() << 8 | Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

void mpu_triggers() {
  ax = (AcX - 2050) / 16384.00;
  ay = (AcY - 77) / 16384.00;
  az = (AcZ - 1947) / 16384.00;
  gx = (GyX + 270) / 131.07;
  gy = (GyY - 351) / 131.07;
  gz = (GyZ + 136) / 131.07;

  // calculating Amplitute vactor for 3 axis
  float raw_amplitude = pow(pow(ax, 2) + pow(ay, 2) + pow(az, 2), 0.5);
  int amplitude = raw_amplitude * 10;         // Mulitiplied by 10 bcz values are between 0 to 1
  if (amplitude <= 4 && trigger2 == false) {  //if AM breaks lower threshold (0.4g)
    trigger1 = true;
    Serial.println("TRIGGER 1 ACTIVATED");
    myData.fallDetect = true;
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
    myData.fallDetect = false;
  }
  if (trigger1 == true) {
    trigger1count++;
    if (amplitude >= 12) {  //if AM breaks upper threshold (3g)
      trigger2 = true;
      Serial.println("TRIGGER 2 ACTIVATED");
      trigger1 = false;
      trigger1count = 0;
    }
  }
  if (trigger2 == true) {
    trigger2count++;
    angleChange = pow(pow(gx, 2) + pow(gy, 2) + pow(gz, 2), 0.5);
    if (angleChange >= 30 && angleChange <= 400) {  //if orientation changes by between 80-100 degrees
      trigger3 = true;
      trigger2 = false;
      trigger2count = 0;
      Serial.println("TRIGGER 3 ACTIVATED");
    }
  }
  if (trigger3 == true) {
    angleChange = pow(pow(gx, 2) + pow(gy, 2) + pow(gz, 2), 0.5);
    fall = true;
    trigger3 = false;
  }
  if (fall == true) {  //in event of a fall detection
    Serial.println("FALL DETECTED");
    myData.fallDetect = true;
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
    myData.fallDetect = false;
    fall = false;
  }
  if (trigger2count >= 6) {  //allow 0.5s for orientation change
    trigger2 = false;
    trigger2count = 0;
    Serial.println("TRIGGER 2 DEACTIVATED");
  }
  if (trigger1count >= 6) {  //allow 0.5s for AM to break upper threshold
    trigger1 = false;
    trigger1count = 0;
    Serial.println("TRIGGER 1 DEACTIVATED");
  }
}
