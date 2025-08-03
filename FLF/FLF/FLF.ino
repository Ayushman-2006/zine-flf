
#include <Wire.h>
#include <U8g2lib.h>
#include <EEPROM.h>

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// EEPROM Addresses
#define EEPROM_ADDRESS_CAL_MIN 0
#define EEPROM_ADDRESS_CAL_MAX (EEPROM_ADDRESS_CAL_MIN + sizeof(cal_min))
#define EEPROM_KP 100
#define EEPROM_KI 104
#define EEPROM_KD 108

// Motor Pins
#define pwmA 10
#define inA1 5
#define inA2 4
#define inB1 7
#define inB2 8
#define pwmB 9

// Multiplexer Pins88
#define muxSig A0
#define muxS0 2
#define muxS1 3
#define muxS2 4

// Buttons
#define btnSelect 10
#define btnBack 11
#define btnEnter 12

// Potentiometer
#define potPin A1

// Constants
const int sensor_count = 8;
double sensor_reading[sensor_count];
double cal_min[sensor_count]={100.0,100.0,100.0,100.0,100.0,100.0,100.0,100.0};
double cal_max[sensor_count]={800.0,800.0,800.0,800.0,800.0,800.0,800.0,800.0};
float weights[sensor_count / 2] = {40.0, 30.0, 20.0, 10.0};
float Kp = 0, Ki = 0, Kd = 0;
float PID = 0, integral_error = 0, previous_error = 0;

// Menu System
int modeIndex = 0;
String modes[] = {"Calibrate", "PID Tune", "Line Follow"};
const int totalModes = sizeof(modes) / sizeof(modes[0]);
bool running = false;

int muxRead(int channel) {
  digitalWrite(muxS0, bitRead(channel, 0));
  digitalWrite(muxS1, bitRead(channel, 1));
  digitalWrite(muxS2, bitRead(channel, 2));
  delayMicroseconds(5);
  return analogRead(muxSig);
}

void sensor_read() {
  for (int i = 0; i < sensor_count; i++) {
    int raw = muxRead(i);
    raw = constrain(raw,(long)cal_min[i],(long)cal_max[i]);
    sensor_reading[i] = map(raw,(long)cal_min[i],(long)cal_max[i], 0, 500);
  }
}

void calibrate() {
  u8g2.clearBuffer();
  u8g2.setCursor(0, 12);
  u8g2.print("Calibrating...");
  u8g2.sendBuffer();

  unsigned long startTime = millis();
  while (millis() - startTime < 5000) {
    for (int i = 0; i < sensor_count; i++) {
      int raw = muxRead(i);
      if (raw < cal_min[i]) cal_min[i] = raw;
      if (raw > cal_max[i]) cal_max[i] = raw;
    }
    delay(10);
  }

  u8g2.clearBuffer();
  u8g2.setCursor(0, 12);
  u8g2.print("Calibration Done");
  u8g2.sendBuffer();
  delay(1000);

  EEPROM.put(EEPROM_ADDRESS_CAL_MIN, cal_min);
  EEPROM.put(EEPROM_ADDRESS_CAL_MAX, cal_max);
  Serial.println("Calibration saved.");
}

void loadCalibration() {
  EEPROM.get(EEPROM_ADDRESS_CAL_MIN, cal_min);
  EEPROM.get(EEPROM_ADDRESS_CAL_MAX, cal_max);

  for (int i = 0; i < sensor_count; i++) {
    if (cal_min[i] >= 1023.0 || cal_max[i] <= 0.0 || cal_max[i] <= cal_min[i]) {
      u8g2.clearBuffer();
      u8g2.setCursor(0, 12);
      u8g2.print("No valid cal data");
      u8g2.sendBuffer();
      delay(1000);
      calibrate();
      return;
    }
  }
  u8g2.clearBuffer();
  u8g2.setCursor(0, 12);
  u8g2.print("Cal data loaded");
  u8g2.sendBuffer();
  delay(1000);
}

void pid_follower() {
  float error = 0;
  sensor_read();
  for (int i = 0; i < sensor_count / 2; i++) {
    error += (sensor_reading[i] - sensor_reading[7 - i]) * weights[i];
  }
  integral_error += error;
  float derivative = error - previous_error;
  PID = (Kp * error) + (Ki * integral_error) + (Kd * derivative);
  previous_error = error;
}

void line_follow() {
  pid_follower();
  int baseSpeed = 150;
  int leftSpeed = baseSpeed + PID;
  int rightSpeed = baseSpeed - PID;
  motor_speed(leftSpeed, rightSpeed);
}

void motor_speed(int left, int right) {
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);

  digitalWrite(inA1, left > 0);
  digitalWrite(inA2, left <= 0);
  analogWrite(pwmA, abs(left));

  digitalWrite(inB1, right > 0);
  digitalWrite(inB2, right <= 0);
  analogWrite(pwmB, abs(right));
}

void update_display() {
  u8g2.clearBuffer();
  u8g2.setCursor(0, 12);
  u8g2.print("Mode: "); u8g2.print(modes[modeIndex]);
  u8g2.setCursor(0, 24);
  u8g2.print("Kp:"); u8g2.print(Kp);
  u8g2.print(" Ki:"); u8g2.print(Ki);
  u8g2.setCursor(0, 36);
  u8g2.print("Kd:"); u8g2.print(Kd);
  u8g2.setCursor(0, 48);
  u8g2.print("PID:"); u8g2.print(PID);
  u8g2.sendBuffer();
}

void loadPIDGains() {
  EEPROM.get(EEPROM_KP, Kp);
  EEPROM.get(EEPROM_KI, Ki);
  EEPROM.get(EEPROM_KD, Kd);
}

void savePIDGains() {
  EEPROM.put(EEPROM_KP, Kp);
  EEPROM.put(EEPROM_KI, Ki);
  EEPROM.put(EEPROM_KD, Kd);
}

void setup() {
  Serial.begin(9600);
  u8g2.begin();

  u8g2.clearBuffer();
  u8g2.setCursor(0, 12);
  u8g2.print("LFR Booting...");
  u8g2.sendBuffer();
  delay(1000);

  pinMode(muxS0, OUTPUT);
  pinMode(muxS1, OUTPUT);
  pinMode(muxS2, OUTPUT);
  pinMode(inA1, OUTPUT);
  pinMode(inA2, OUTPUT);
  pinMode(inB1, OUTPUT);
  pinMode(inB2, OUTPUT);
  pinMode(pwmA, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(btnSelect, INPUT_PULLUP);
  pinMode(btnBack, INPUT_PULLUP);
  pinMode(btnEnter, INPUT_PULLUP);

  loadCalibration();
  loadPIDGains();
}

void loop() {
  update_display();
  int potVal = analogRead(potPin);
  modeIndex = map(potVal, 0, 1023, 0, totalModes - 1);

  if (!digitalRead(btnEnter)) {
    running = true;
    u8g2.clearBuffer();
    u8g2.setCursor(0, 12);
    u8g2.print("Running: "); u8g2.print(modes[modeIndex]);
    u8g2.sendBuffer();
    delay(500);
  }

  while (running) {
    if (!digitalRead(btnBack)) {
      running = false;
      break;
    }

    if (modes[modeIndex] == "Calibrate") calibrate();
    else if (modes[modeIndex] == "PID Tune") {
      Kp = analogRead(A2) / 100.0;
      Ki = analogRead(A3) / 100.0;
      Kd = analogRead(A4) / 100.0;
    }
    else if (modes[modeIndex] == "Line Follow") line_follow();

    update_display();
    delay(50);
  }
}
