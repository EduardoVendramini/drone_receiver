#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h> // For I2C communication
// #include <MPU6050_light.h>
#include <ESP32Servo.h> // https://madhephaestus.github.io/ESP32Servo/classServo.html
#include <PID_v1.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define MIN_PULSE_LENGTH 1000 // https://howtomechatronics.com/tutorials/arduino/arduino-brushless-motor-control-tutorial-esc-bldc/
#define MAX_PULSE_LENGTH 2000

#define POTENTIOMETER_SENSOR 32
int potentiometerValue = 0;

// Encoder - Variáveis corrigidas
#define ENCODER_PIN_A 12
#define ENCODER_PIN_B 13
volatile long lastPositionEncoder = 0;
volatile int lastEncoded = 0;
volatile unsigned long lastInterruptTime = 0;

// Debouncing time em microssegundos (ajuste conforme necessário)
const unsigned long DEBOUNCE_TIME = 1000; // 1ms

void IRAM_ATTR handleEncoder();
static float encoderAngle = 0.0;

// MPU 6050 sensor -> GPIO 21 (SDA), GPIO 22 (SCL)
Adafruit_MPU6050 mpu;
float complementaryAngle = 0.0;
float gyroAngleX = 0.0;
float accelAngleX = 0.0;
float gyroRate = 0.0; // °/s

// Escs
Servo yellowEsc;
Servo pinkEsc;

// Kalman filter
float kalmanAngle = 0.0;
float K_bias = 0.0;
float K_rate = 0.0;
float PKalman[2][2] = {{0, 0}, {0, 0}};
float K[2];
float S = 0.0;
float dt0Kalman = 0.0;
float dt1Kalman = 0.0;
float y = 0.0;
/*Ruído do processo para o ângulo
Aumentar Q_angle → o filtro responde mais rápido a mudanças no sinal de entrada.
Reduzir Q_angle → o filtro confia mais nas previsões internas, ignorando variações rápidas.*/
float Q_angle = 0.01; // valor muito baixo estoura valores
/*
Ruído do processo para o bias do giroscópio
Aumentar Q_bias → o filtro ajusta o bias mais rapidamente, útil se o bias do giroscópio varia com o tempo.
Reduzir Q_bias → o filtro considera o bias mais constante.
*/
float Q_bias = 0.0003;
/*0.003
Ruído da medição (do acelerômetro, por exemplo)
Reduzir R_measure → o filtro confia mais no sensor, reagindo mais rapidamente a mudanças.
Aumentar R_measure → o filtro considera a medição ruidosa, e responde mais lentamente.
*/
float R_measure = 0.9; 

float kalmanFilter(float newAngle, float newRate);
float K_angle = 0.0;

// Mean filter
float getMeanFilteredAngle(float angle);
float buff[5], meanFilteredAngle = 0.0;

// Calibration
void calibrateSensor();
void calibrateEscs();

void getRemoteControlParameters();
void printRemoteControlParameters();
void setMotors();

typedef struct message_struct
{
  float kp = 1.2, ki = 7.0, kd = 3.2;
  int m1 = MIN_PULSE_LENGTH;
  int m2 = MIN_PULSE_LENGTH;
  float gyr = 0.98; // alpha for complementary filter
  float ref = 0;
  int automaticState = 1;
  int sensorState = 0;
} message_struct;

message_struct receivedData;

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len); // Callback function for ESP-NOW data reception
void readMacAddress();

double P, I, D;
float deltaT, error, previousError = 0.0, pidOutput;
void calculatePid();

long getEncoderPosition();
void resetEncoderPosition();

unsigned long initialTime = 0, finalTime = 0;

// complementary filter variables
float dt0Complementary = 0.0;
float alpha = 0.9; // complementary filter constant
float dt1Complementary = 0.0;

float usedAngle = 0.0; // angle used for PID control

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

  // Encoder setup - Corrigido
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);

  // Inicializar estado do encoder
  bool MSB = digitalRead(ENCODER_PIN_A);
  bool LSB = digitalRead(ENCODER_PIN_B);
  lastEncoded = (MSB << 1) | LSB;

  // Attach interrupts para ambos os pinos
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), handleEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), handleEncoder, CHANGE);

  // MPU 6050 setup
  if (!mpu.begin())
  {
    Serial.println("Failed to find MPU6050 chip");
    while (1)
    {
      delay(10);
    }
  }
  mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);
  mpu.setHighPassFilter(MPU6050_HIGHPASS_DISABLE);
  // mpu.setAccelerometerRange(MPU6050_RANGE_16_G);

  // Escs setup
  yellowEsc.attach(17, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  pinkEsc.attach(16, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  // calibrateEscs();

  dt1Kalman = millis();
  dt1Complementary = millis();
}

void loop()
{
  initialTime = millis();

  // encoder
  static long lastEncoderPosition = 0;
  long currentEncoderPosition = getEncoderPosition();

  if (currentEncoderPosition != lastEncoderPosition)
  {
    // Convert encoder position to angle
    encoderAngle = currentEncoderPosition * 87.0 / 400.0;
    lastEncoderPosition = currentEncoderPosition;
  }

  // mpu 6050
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  accelAngleX = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
  gyroRate = (g.gyro.x + 0.029) * (180.0 / PI); // 0.029 is the bias of the gyroscope

  // Complementary filter
  dt0Complementary = (millis() - dt1Complementary) / 1000.0;
  dt1Complementary = millis();
  gyroAngleX += gyroRate * dt0Complementary;
  complementaryAngle = alpha * gyroAngleX + (1 - alpha) * accelAngleX;

  // Mean filter
  meanFilteredAngle = getMeanFilteredAngle(complementaryAngle);

  // Kalman
  K_angle = kalmanFilter(accelAngleX, gyroRate);

  if (receivedData.sensorState == 1)
  {
    usedAngle = complementaryAngle;
  }
  else
  {
    usedAngle = encoderAngle;
    //usedAngle = K_angle;
  }
  Serial.print(complementaryAngle);
  Serial.print("\t");
  Serial.print(meanFilteredAngle);
  Serial.print("\t");
  Serial.print(K_angle);
  Serial.print("\t");
  Serial.println(encoderAngle);

  setMotors();

  // Serial.print(P);
  // Serial.print("\t");
  // Serial.print(I);
  // Serial.print("\t");
  // Serial.println(D);

  finalTime = millis();
  if (finalTime - initialTime < 50)
    delay(50 - (finalTime - initialTime));
}

void IRAM_ATTR handleEncoder()
{
  // Debouncing por tempo
  unsigned long interruptTime = micros();
  if (interruptTime - lastInterruptTime < DEBOUNCE_TIME)
  {
    return;
  }
  lastInterruptTime = interruptTime;

  // Leitura dos pinos
  bool MSB = digitalRead(ENCODER_PIN_A);
  bool LSB = digitalRead(ENCODER_PIN_B);

  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;

  // Tabela de estados para encoder em quadratura
  // Rotação horária: 00 -> 01 -> 11 -> 10 -> 00
  // Rotação anti-horária: 00 -> 10 -> 11 -> 01 -> 00
  switch (sum)
  {
  case 0b0001: // 00 -> 01
  case 0b0111: // 01 -> 11
  case 0b1110: // 11 -> 10
  case 0b1000: // 10 -> 00
    lastPositionEncoder++;
    break;

  case 0b0010: // 00 -> 10
  case 0b1011: // 10 -> 11
  case 0b1101: // 11 -> 01
  case 0b0100: // 01 -> 00
    lastPositionEncoder--;
    break;

  default:
    // Estados inválidos - ignorar
    break;
  }

  lastEncoded = encoded;
}

long getEncoderPosition()
{
  noInterrupts();
  long pos = lastPositionEncoder;
  interrupts();
  return pos;
}

void resetEncoderPosition()
{
  noInterrupts();
  lastPositionEncoder = 0;
  interrupts();
}

float kalmanFilter(float newAngle, float newRate)
{
  // https://github.com/jarzebski/Arduino-KalmanFilter/tree/master

  dt0Kalman = (millis() - dt1Kalman) / 1000;

  K_rate = newRate - K_bias;
  K_angle += dt0Kalman * K_rate;

  PKalman[0][0] += dt0Kalman * (PKalman[1][1] + PKalman[0][1]) + Q_angle * dt0Kalman;
  PKalman[0][1] -= dt0Kalman * PKalman[1][1];
  PKalman[1][0] -= dt0Kalman * PKalman[1][1];
  PKalman[1][1] += Q_bias * dt0Kalman;

  S = PKalman[0][0] + R_measure;

  K[0] = PKalman[0][0] / S;
  K[1] = PKalman[1][0] / S;

  y = newAngle - K_angle;

  K_angle += K[0] * y;
  K_bias += K[1] * y;

  PKalman[0][0] -= K[0] * PKalman[0][0];
  PKalman[0][1] -= K[0] * PKalman[0][1];
  PKalman[1][0] -= K[1] * PKalman[0][0];
  PKalman[1][1] -= K[1] * PKalman[0][1];

  dt1Kalman = millis();

  return K_angle;
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
  Serial.print("sensorState: ");
  Serial.println(receivedData.sensorState);
  Serial.println();
  */
}

void calculatePid()
{
  deltaT = (millis() - initialTime);

  error = receivedData.ref - usedAngle;

  P = receivedData.kp * error;

  D = 10 * receivedData.kd * (error - previousError) / deltaT;

  previousError = error;

  pidOutput = P + I + D;

  // Anti-windup: saturation and clamping
  if (pidOutput > 200)
    pidOutput = 200;
  else if (pidOutput < -200)
    pidOutput = -200;
  else if (receivedData.ki == 0)
    I = 0;
  else
    I += receivedData.ki * error * deltaT / 1000;
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

float getMeanFilteredAngle(float angle)
{
  // This function returns the mean from the previous 5 values of angle measured
  buff[0] = buff[1];
  buff[1] = buff[2];
  buff[2] = buff[3];
  buff[3] = buff[4];
  buff[4] = angle;

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