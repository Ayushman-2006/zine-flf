// removed eeprom initializing hc05 went rogue

// THINGS ------ TO DO ------
// motor speed clipping at stall speeds
// if (left > 0 && left < 60) left = 60;
// if (right > 0 && right < 60) right = 60;
// ------------------------------
// weights calculation (1+2+3+4)=10
//------------------------------
// Anti windup 
// lost line handling 
// intersections/ turns
//---------------------------
#include <Wire.h>
#include <EEPROM.h>

// wiring order of your sensors (left to right)
const int sensor_order[8] = {2,4,0,6,5,7,3,1};

// EEPROM Addresses
#define EEPROM_ADDRESS_CAL_MIN 0
#define EEPROM_ADDRESS_CAL_MAX (EEPROM_ADDRESS_CAL_MIN + sizeof(double) * sensor_count)
#define EEPROM_KP 7
#define EEPROM_KI 0
#define EEPROM_KD 0

#define PID_MAX 220
#define PID_MIN -220
#define Basespd 220
// Motor Pins 
#define pwmA 11
#define inA1 9
#define inA2 10
#define inB1 7
#define inB2 8
#define pwmB 6
//stdby pin uda di

// Multiplexer Pins
#define muxSig A0
#define muxS0 A1
#define muxS1 A2
#define muxS2 A3


// Buttons
#define btnSelect 2
#define btnUp 4
#define btnDown 5

// Constants
const int line_threshold = 650;
const int sensor_count = 8;
float sensor_reading[sensor_count];
float cal_min[sensor_count];
float cal_max[sensor_count];
// int8_t weights[sensor_count / 2] = {4,3,2,1};
float Kp = 1.6;
float Ki = 0; 
float Kd = 15;   // default values
double PID = 0, integral_error = 0, previous_error = 0,derivative=0;
int distFromCenter[8] = { -400,-200,-70,-20,20,70,200,400 };
float sensorValue = 0;
int lastSeenIndex = -1;   // which sensor last saw the line
int lastSeenSide = 0;  
int lastLineIndex = 3;   // start roughly at center sensors
bool lineLocked = false;
int sens[8] = {0,0,0,1,0,0,0,0};

bool newLine = false;
int lastSensor = 3;


// Menu...
String modes[] = {"Line Follow", "Calibrate", "PID Tuning", "Threshold Change", "Motor Test", "Sensor Reading"};
const int totalModes = sizeof(modes) / sizeof(modes[0]);
int modeIndex = 0;
bool running = false;

// -------- MUX READING ----------
int muxRead(int channel) {
  digitalWrite(muxS0, bitRead(channel, 0));
  digitalWrite(muxS1, bitRead(channel, 1));
  digitalWrite(muxS2, bitRead(channel, 2));
  delayMicroseconds(5);
  return analogRead(muxSig);
}

// -------- SENSOR READ ----------
void sensor_read() {
  for (int i = 0; i < sensor_count; i++) {
    int idx = sensor_order[i]; // remapped physical index

    int raw = muxRead(idx);
    raw = constrain(raw, (long)cal_min[idx], (long)cal_max[idx]);

    sensor_reading[i] = 1000 - map(raw, cal_min[idx], cal_max[idx], 0, 1000);
    // Print in "min/present/max" format
    // Serial.print("S");
    // Serial.print(i); // logical order (0..7, left to right)
    // Serial.print("[");
    // Serial.print(idx); // physical channel
    // Serial.print("]: ");
    // Serial.print(cal_min[idx], 0);   // min
    // Serial.print("/");
    // Serial.print(sensor_reading[i], 0); // mapped value
    // Serial.print("/");
    // Serial.print(cal_max[idx], 0);   // max
    // Serial.print("  ");
  }
  // Serial.println();
}

// -------- CALIBRATION ----------
void calibrate() {
  for (int i = 0; i < sensor_count; i++) {
    cal_min[i] = 1023.0;
    cal_max[i] = 0.0;
  }
  motor_speed(200,-200);
  unsigned long startTime = millis();
  while (millis() - startTime < 8000) { // 8 sec
    for (int i = 0; i < sensor_count; i++) {
      int raw = muxRead(i);
      if (raw < cal_min[i]) cal_min[i] = raw;
      if (raw > cal_max[i]) cal_max[i] = raw;
    }
    delay(10);
  }
  motor_speed(0,0);

  EEPROM.put(EEPROM_ADDRESS_CAL_MIN, cal_min);
  EEPROM.put(EEPROM_ADDRESS_CAL_MAX, cal_max);
  Serial.println("Calibration Done");
}
unsigned long changeTime=0;
void updateLastSeen() {
  int leftSum = 0;
  int rightSum = 0;

  for (int i = 0; i < sensor_count; i++) {
    if (sensor_reading[i] > 350) {
      if (i < sensor_count / 2) {
        leftSum += sensor_reading[i];
      } else {
        rightSum += sensor_reading[i];
      }
    }
  }


  if (leftSum > 0 || rightSum > 0) {
    if (leftSum > rightSum) {
      lastSeenSide = -1;   
      lastSeenIndex = 0;   
    } else if (rightSum  > leftSum) {
      lastSeenSide = 1;    
      lastSeenIndex = sensor_count - 1;
    } else {
      
    }
  }

    // Serial.print(millis() - changeTime);
  // Serial.print('\n');
  // if(sensor_reading[0] > 350 && millis() - changeTime > 50 && lastSeenSide != -1){
  //   lastSeenSide = -1;
  //   changeTime = millis(); 
  // }
  // else if(sensor_reading[7] > 350 && millis() - changeTime > 50 && lastSeenSide != 1){
  //   lastSeenSide = 1;
  //   changeTime = millis();
  // }
}


// -------- PID FOLLOWER ----------
// void pid_follower() {
//   sensor_read();
//   updateLastSeen();

//   double totalSum = 0;
//   double weightedSum = 0;

//   // if on white then all the elements of the "sens" arr will be 0 !!!
//   // for (int i = 0; i < 8; i++) {
//   //   if(sensor_reading[i] >= line_threshold && i != lastSensor && i != lastSensor - 1 && i != lastSensor + 1){
//   //     if(sensor_reading[lastSensor] < line_threshold){
//   //       sens[lastSensor] = 0;
//   //       newLine = false;
//   //     }
//   //     else{
//   //       newLine = true;
//   //       break;
//   //     }
//   //     sens[i] = 1;
//   //     lastSensor = i;
//   //     break;
//   //   }
//   //   else
//   //     sens[i] = 0;
//   // }

//   for (int i = 0; i < 8; i++) {
//     // if(i == 0){
//     //   if(sensor_reading[i] > line_threshold)

//     // }
//     totalSum += sensor_reading[i];
//     weightedSum += distFromCenter[i] * sensor_reading[i];
//   }

//   float error = 0;
//   if (totalSum > 0) {
//     error = (float)weightedSum / totalSum;
//   }

//   integral_error += error;
//   integral_error = constrain(integral_error, -50, 50);
  

//   float derivative = error - previous_error;

//   PID = (Kp * error) + (Ki * integral_error) + (Kd * derivative);
//   previous_error = error;
//   Serial.print("  ");
//   Serial.print(totalSum);
//   Serial.print(" ");
//   Serial.print(weightedSum);
//   Serial.print(" ");
//   Serial.print(PID);
//   Serial.print("  ");
//   Serial.print(error);
//   Serial.println();
// }

void pid_follower() {
  sensor_read();
  updateLastSeen();
  // --- Weighted error calculation ---
  double totalSum = 0, weightedSum = 0;
  for (int i = 0; i < sensor_count; i++) {
    totalSum += sensor_reading[i];
    if(sensor_reading[i] < 350)
      continue;
    weightedSum += distFromCenter[i] * sensor_reading[i];
  }

  float error = 0;
  if (totalSum > 0) {
    error = (float)weightedSum / totalSum;
  } else {
    // Line completely lost → fallback to lastSeenSide
    if (lastSeenSide == -1) {
      motor_speed(-120, 120); // spin left
    } else if (lastSeenSide == 1) {
      motor_speed(120, -120); // spin right
    } else {
      motor_speed(0, 0);
    }
    return;
  }

  // --- Edge sensor "snap back" ---
  if (sensor_reading[0] > line_threshold) {
    error = -300;  // force hard left
  } else if (sensor_reading[7] > line_threshold) {
    error = 300;   // force hard right
  }

  // --- Anti-windup ---
  if (abs(error) > 120) integral_error = 0;
  else integral_error += error;

  integral_error = constrain(integral_error, -80, 80);

  // --- Derivative (look-ahead) ---
  float derivative = error - previous_error;
  float lookAheadError = 0.7 * error + 0.3 * previous_error;

  // --- PID Calculation ---
  PID = (Kp * lookAheadError) + (Ki * integral_error) + (Kd * derivative);
  previous_error = error;

  // --- Adaptive base speed ---
  int baseSpeed = Basespd;
  if (abs(error) > 100) baseSpeed = Basespd * 0.7;
  if (abs(error) > 180) baseSpeed = Basespd * 0.5;

  // --- Final motor speeds ---
  int leftSpeed  = baseSpeed - PID;
  int rightSpeed = baseSpeed + PID;

  // Stall-speed clipping (prevents weak motor jitters)
  if (leftSpeed > 0 && leftSpeed < 60) leftSpeed = 60;
  if (rightSpeed > 0 && rightSpeed < 60) rightSpeed = 60;

  leftSpeed  = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  motor_speed(leftSpeed, rightSpeed);
}

// -------- LINE FOLLOW ----------
void line_follow() {
  // if(sensor_reading[0] > line_threshold){
  //   while(sensor_reading[3] < line_threshold || sensor_reading[4] < line_threshold){
  //     motor_speed(20, 100);
  //   }
  // }
  // else if(sensor_reading[7] > line_threshold){
  //   while(sensor_reading[3] < line_threshold || sensor_reading[4] < line_threshold){
  //     motor_speed(100, 20);
  //   }
  // }
  pid_follower();
//----------lost line handling------

 bool lineLost = true;
for (int i = 0; i < sensor_count; i++) {
  if (sensor_reading[i] > line_threshold) {
    lineLost = false;
    break;
  }
}


if (lineLost) {
  int recoverSpeed = 100;
  if (lastSeenSide == -1) {
    
     motor_speed(recoverSpeed, -recoverSpeed + 10); // spin left
  } else if (lastSeenSide == 1) {
    motor_speed(-recoverSpeed + 10, recoverSpeed);// spin right
  } else {
    motor_speed(0, 0); // no idea where line went
  }
  return;
}

// ----------tillherebudddyyyy------

// ----------pivoted extremes------
// if (sensor_reading[0] > line_threshold && sensor_reading[1]<line_threshold) {
//   motor_speed(-20, 70); // pivot left
//   return;
// }
// if (sensor_reading[7] > line_threshold && sensor_reading[6]<line_threshold) {
//   motor_speed(70, -20); // pivot right
//   return;
// }



  int baseSpeed = Basespd;
  int leftSpeed = baseSpeed - PID;
  int rightSpeed = baseSpeed + PID;
  //remove for the pivoted movement.
  leftSpeed  = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  motor_speed(leftSpeed, rightSpeed);
}

// -------- MOTOR CONTROL ----------
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

// ---------- PID TUNING MODE ----------
void pidTuningMode() {
  while (true) {
    if (digitalRead(btnDown) == LOW) {
      delay(200);
      return;
    }

    if (Serial.available()) {
      char incoming = Serial.read();
      if (incoming == 'p' || incoming == 'i' || incoming == 'd') {
        String constant = "";
        unsigned long startTime = millis();
        while (millis() - startTime < 100) {
          if (Serial.available()) {
            char data = Serial.read();
            if (data == '\n') break;
            constant += data;
          }
        }
        float value = constant.toFloat();

        if (incoming == 'p') {
          Kp = value;
          EEPROM.put(EEPROM_KP, Kp);
        } 
        else if (incoming == 'i') {
          Ki = value;
          EEPROM.put(EEPROM_KI, Ki);
        } 
        else if (incoming == 'd') {
          Kd = value;
          EEPROM.put(EEPROM_KD, Kd);
        }

        Serial.print("PID Update: ");
        Serial.print(incoming);
        Serial.print(" = ");
        Serial.println(value);
      }
    }
    delay(150);
  }
}

// -------- MODE HANDLERS ----------
void run_mode(int idx) {
  running = true;

  if (idx == 0) { // Line Follow
    Serial.println("Line following!!");
    while (running) {
      line_follow();
      if (!digitalRead(btnDown)) {
         running = false; motor_speed(0, 0);
      }
    }
  }

  else if (idx == 1) { // Calibrate
    calibrate();
    running = false;
  }

  else if (idx == 2) { // PID tuning
    pidTuningMode();
    Serial.println(Kp);
    Serial.println(Kd);
    Serial.println(Ki);
  }

  else if (idx == 3) { // Threshold change
    Serial.println("Threshold Change Mode");
    delay(2000);
    running = false;
  }

  else if (idx == 4) { // Motor Test
    Serial.println("Motor Test");
    motor_speed(150, 0); // left
    delay(4000);
    motor_speed(0, 150); // right
    delay(4000);
    motor_speed(0, 0);
    running = false;
  }

  else if (idx == 5) { // Sensor Reading
    while (running) {
      sensor_read();
      for (int i = 0; i < sensor_count; i++) {
        Serial.print("S"); Serial.print(i);
        Serial.print(": "); Serial.print(int(sensor_reading[i])); Serial.print("  ");
      }
      Serial.println();
      if (!digitalRead(btnDown)) running = false;
      delay(100);
    }
  }
}

// -------- SETUP ----------
void setup() {
  Serial.begin(9600);

  pinMode(btnSelect, INPUT_PULLUP);
  pinMode(btnUp, INPUT_PULLUP);
  pinMode(btnDown, INPUT_PULLUP);

  pinMode(muxS0, OUTPUT);
  pinMode(muxS1, OUTPUT);
  pinMode(muxS2, OUTPUT);
  pinMode(inA1, OUTPUT);
  pinMode(inA2, OUTPUT);
  pinMode(inB1, OUTPUT);
  pinMode(inB2, OUTPUT);
  pinMode(pwmA, OUTPUT);
  pinMode(pwmB, OUTPUT);

  for (int i = 0; i < sensor_count; i++) {
    cal_min[i] = 0;
    cal_max[i] = 1023;
  }
  // EEPROM.get(EEPROM_KP, Kp);
  // EEPROM.get(EEPROM_KI, Ki);
  // EEPROM.get(EEPROM_KD, Kd);
  EEPROM.get(EEPROM_ADDRESS_CAL_MIN, cal_min );
  EEPROM.get(EEPROM_ADDRESS_CAL_MAX, cal_max);

  Serial.println("Menu Ready. Use buttons.");
}

// -------- LOOP ----------
void loop() {
  if (!digitalRead(btnUp)) {
    modeIndex = (modeIndex + 1) % totalModes;
    Serial.print("Mode: "); Serial.println(modes[modeIndex]);
    delay(300);
  }
  if (!digitalRead(btnDown)) {
    modeIndex = (modeIndex - 1 + totalModes) % totalModes;
    Serial.print("Mode: "); Serial.println(modes[modeIndex]);
    delay(300);
  }
  if (!digitalRead(btnSelect)) {
    run_mode(modeIndex);
    delay(300);
  }
  // motor_speed(-255,255);
  // delay(4000);
  // motor_speed(255,-255);
  // delay(4000);
}
