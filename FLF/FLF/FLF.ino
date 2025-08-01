#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// === OLED ===
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// === Motor Driver TB6612 ===
#define pwmA 3
#define inA1 5
#define inA2 4
#define inB1 7
#define inB2 8
#define pwmB 9

// === Multiplexer CD4051 ===
#define muxSig A0
#define muxS0 2
#define muxS1 3
#define muxS2 4

// === Buttons ===
#define btnSelect 10
#define btnBack 11
#define btnEnter 12

// === Potentiometer ===
#define potPin A1

// === Constants ===
const int sensor_count = 8;
double sensor_reading[sensor_count];
double cal_min[sensor_count] = {100, 100, 100, 100, 100, 100, 100, 100};
double cal_max[sensor_count] = {800, 800, 800, 800, 800, 800, 800, 800};
float weights[sensor_count / 2] = {40.0, 30.0, 20.0, 10.0};

float Kp = 0.0, Ki = 0.0, Kd = 0.0;
float PID = 0, integral_error = 0, previous_error = 0;

// === Menu State ===
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
    raw = constrain(raw, cal_min[i], cal_max[i]);
    sensor_reading[i] = map(raw, cal_min[i], cal_max[i], 0, 500);
  }
}

void caliberate() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Calibrating...");
  display.display();

  unsigned long startTime = millis();
  while (millis() - startTime < 5000) {
    for (int i = 0; i < sensor_count; i++) {
      int raw = muxRead(i);
      if (raw < cal_min[i]) cal_min[i] = raw;
      if (raw > cal_max[i]) cal_max[i] = raw;
    }
    delay(10);
  }

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Done");
  display.display();
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
  sensor_read();
  pid_follower();

  int baseSpeed = 150;
  int leftSpeed = baseSpeed + PID;
  int rightSpeed = baseSpeed - PID;

  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  analogWrite(pwmA, leftSpeed);
  analogWrite(pwmB, rightSpeed);

  digitalWrite(inA1, HIGH);
  digitalWrite(inA2, LOW);
  digitalWrite(inB1, HIGH);
  digitalWrite(inB2, LOW);
}

void motor_speed(int left, int right) {
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);

  if (left > 0) {
    digitalWrite(inA1, HIGH);
    digitalWrite(inA2, LOW);
  } else {
    digitalWrite(inA1, LOW);
    digitalWrite(inA2, HIGH);
  }
  analogWrite(pwmA, abs(left));

  if (right > 0) {
    digitalWrite(inB1, HIGH);
    digitalWrite(inB2, LOW);
  } else {
    digitalWrite(inB1, LOW);
    digitalWrite(inB2, HIGH);
  }
  analogWrite(pwmB, abs(right));
}

void update_display() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);

  display.print("Mode: ");
  display.println(modes[modeIndex]);

  display.print("Kp: ");
  display.print(Kp);
  display.print(" Ki: ");
  display.println(Ki);

  display.print("Kd: ");
  display.println(Kd);

  display.setCursor(0, 30);
  display.print("PID: ");
  display.println(PID);

  display.setCursor(0, 40);
  for (int i = 0; i < sensor_count; i++) {
    display.print((int)(sensor_reading[i] / 100));
    display.print(" ");
  }

  display.display();
}

// === Setup ===
void setup() {
  Serial.begin(9600);

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

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    while (1);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("LFR Booting...");
  display.display();
  delay(1000);
}

// === Loop ===
void loop() {
  update_display();

  // Scroll menu with potentiometer
  int potVal = analogRead(potPin);
  modeIndex = map(potVal, 0, 1023, 0, totalModes - 1);

  if (!digitalRead(btnEnter)) {
    running = true;
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("Running: ");
    display.println(modes[modeIndex]);
    display.display();
    delay(500);
  }

  while (running) {
    if (!digitalRead(btnBack)) {
      running = false;
      break;
    }

    if (modes[modeIndex] == "Calibrate") caliberate();
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
