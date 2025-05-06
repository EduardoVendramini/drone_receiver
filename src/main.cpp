#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h> // For I2C communication
#include <MPU6050_light.h>
#include <ESP32Servo.h> // https://madhephaestus.github.io/ESP32Servo/classServo.html
#include <PID_v1.h>

#define MIN_PULSE_LENGTH 1000 // https://howtomechatronics.com/tutorials/arduino/arduino-brushless-motor-control-tutorial-esc-bldc/
#define MAX_PULSE_LENGTH 2000

// MPU 6050 sensor
MPU6050 mpu(Wire); // TODO: Change on ESP32 -> GPIO 21 (SDA), GPIO 22 (SCL)

// Escs
Servo yellowEsc;
Servo pinkEsc;

// Filter
float getFilteredAngle();
float buff[5], filteredAngle = 0;

// Calibration
void calibrateSensor();
void calibrateEscs();

float measuredAngle;
void getRemoteControlParameters();
void printRemoteControlParameters();
void setMotors();

typedef struct message_struct
{
  float kp = 1.5, ki = 0.05, kd = 0;
  int m1 = MIN_PULSE_LENGTH;
  int m2 = MIN_PULSE_LENGTH;
  float gyr = 0.98;
  float ref = 0;
  int automaticState = 1;
  int disturbanceState = 0;
} message_struct;

message_struct receivedData;

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len); // Callback function for ESP-NOW data reception
void readMacAddress();

double P, I, D;
float deltaT, error, previousError, pidOutput;
void calculatePid();

unsigned long initialTime = 0, finalTime = 0;

void setup()
{
  // espnow communication
  Serial.begin(115200);
  Serial.println("Starting...");

  WiFi.mode(WIFI_STA);
  readMacAddress();
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);

  // MPU 6050 setup
  Wire.begin();
  mpu.begin();
  //calibrateSensor();
  mpu.setFilterGyroCoef(0.98);

  // Escs setup
  yellowEsc.attach(16, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  pinkEsc.attach(17, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  //calibrateEscs();
}

void loop()
{

  initialTime = millis();

  // angle
  mpu.update();
  measuredAngle = mpu.getAngleX();
  filteredAngle = getFilteredAngle();

  Serial.print(">measuredAngle:");
  Serial.println(measuredAngle);

  setMotors();

  finalTime = millis();
  if (finalTime - initialTime < 50)
    delay(50 - (finalTime - initialTime));
}

void readMacAddress()
{
  uint8_t baseMac[6];
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
  if (ret == ESP_OK)
  {
    Serial.println("ESP-NOW Receiver MAC Address:");
    Serial.printf("%02x:%02x:%02x:%02x:%02x:%02x\n",
                  baseMac[0], baseMac[1], baseMac[2],
                  baseMac[3], baseMac[4], baseMac[5]);
  }
  else
  {
    Serial.println("Failed to read MAC address");
  }
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  memcpy(&receivedData, incomingData, sizeof(receivedData));
  /*
  Serial.print("Data received: ");
  // Serial.println(len);
  Serial.print("kp: ");
  Serial.println(receivedData.kp);
  Serial.print("ki: ");
  Serial.println(receivedData.ki);
  Serial.print("kd: ");
  Serial.println(receivedData.kd);
  Serial.print("m1: ");
  Serial.println(receivedData.m1);
  Serial.print("m2: ");
  Serial.println(receivedData.m2);
  Serial.print("gyr: ");
  Serial.println(receivedData.gyr);
  Serial.print("ref: ");
  Serial.println(receivedData.ref);
  Serial.print("automaticState: ");
  Serial.println(receivedData.automaticState);
  Serial.print("disturbanceState: ");
  Serial.println(receivedData.disturbanceState);
  Serial.println();
  */
}

void calculatePid()
{
  deltaT = (millis() - initialTime) / 1000.0;

  error = receivedData.ref - filteredAngle;

  P = receivedData.kp * error;
  I += receivedData.ki * error * deltaT;
  D = receivedData.kd * (error - previousError) / deltaT;

  previousError = error;

  pidOutput = P + I + D;
}

void calibrateEscs()
{
  Serial.println("Calibrating ESCs...");
  delay(1000);

  Serial.println("Writting maximum pulse length...");
  Serial.println("Turn on power source, then wait 2 seconds and press any key.");
  yellowEsc.writeMicroseconds(MAX_PULSE_LENGTH);
  pinkEsc.writeMicroseconds(MAX_PULSE_LENGTH);
  while (!Serial.available())
  {
  }
  delay(1000);

  Serial.println("Writting minimum pulse length...");
  yellowEsc.writeMicroseconds(MIN_PULSE_LENGTH);
  pinkEsc.writeMicroseconds(MIN_PULSE_LENGTH);
  delay(1000);
  Serial.println("ESCs calibration done.");
  return;
}

void calibrateSensor()
{
  Serial.println("Calibrating MPU sensor...");
  mpu.calcOffsets(true, true); // Calculate offsets for gyro and acc
  delay(100);
  Serial.println("MPU calibration done");
}

float getFilteredAngle()
{
  // This function returns the mean from the previous 5 values of angle measured
  buff[0] = buff[1];
  buff[1] = buff[2];
  buff[2] = buff[3];
  buff[3] = buff[4];
  buff[4] = measuredAngle;

  return (buff[0] + buff[1] + buff[2] + buff[3] + buff[4]) / 5;
}

void setMotors()
{
  if (receivedData.automaticState)
  {
    calculatePid();
    yellowEsc.writeMicroseconds(int(1300 - pidOutput));
    pinkEsc.writeMicroseconds(int((1300 + pidOutput)));
  }
  else
  {
    yellowEsc.writeMicroseconds(receivedData.m1);
    pinkEsc.writeMicroseconds(receivedData.m2);
  }
}
