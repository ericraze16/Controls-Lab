#include <Arduino.h>
#include "geeWhiz.h"

// ================== Pins ==================
int MOT_PIN = A0;   // motor angle sensor
int BAL_PIN = A1;   // ball position sensor

// Global Consts
// All pos values measured on station 
const int ZERO_POS = 9418; // 0 rad
const int MAX_POS = 7287; // PI/4 ra5d
const int MIN_POS = 11515; // - PI/4 rad
const float OFFSET = 3.492179902;
const float SLOPE = -0.0003714698048;

// negative voltage spins clockwise
const float DEFAULT_VOLTAGE = -5.5f;  

// ================== Setup ==================
void setup() {

  analogReadResolution(14);
  pinMode(A5, OUTPUT);   // A5 can be used to measure cycle time using an oscilloscope by connecting the scope to the Arduino Box Motor Leads
  Serial.begin(115200);
  delay(300);

  geeWhizBegin();                 
  set_control_interval_ms(100); // 100 ms loop
  setMotorVoltage(0.0f);

  Serial.println("geeWhiz Started");
  Serial.println("time, maxY, minY, ball, motor_raw, motor_rad");
}

// ================== Loop ==================
void loop() {
  // Simple step input to measure overshoot and peak time
  
}

// Convert motor pos to radians using system characterization params
float get_motor_rad() {
  int motor_pos_raw = analogRead(MOT_PIN);
  return (SLOPE * motor_pos_raw) + OFFSET;
}

void move_to_pos(int target) {
  int current_pos = analogRead(MOT_PIN);
  int direction = current_pos > target ? 1 : -1;

  while (analogRead(MOT_PIN) > target) {

  }

    
  
  if (direction > 0) {
      while (analogRead(MOT_PIN) < MIN_POS) {
      setMotorVoltage(direction * DEFAULT_VOLTAGE);
      delay(50);
    }
  } else {
      while (analogRead(MOT_PIN) > MAX_POS) {
      setMotorVoltage(direction * DEFAULT_VOLTAGE);
      delay(50);
  }
  setMotorVoltage(0.0f);
}


// ================== Control ISR ==================
void interval_control_code(void) {
  // ---- For Serial Plotter Scaling
  int maxy = 17000;
  int miny = 0;
  // ---- Read sensors ----
  int motor = analogRead(MOT_PIN);
  int ball  = analogRead(BAL_PIN);


  digitalWrite(A5,HIGH);   // A5 can be used to measure cycle time using an oscilloscope by connecting the scope to the Arduino Box Motor Leads
  Serial.print(millis());
  Serial.print(",");
  Serial.print(maxy);
    Serial.print(",");
  Serial.print(miny);
    Serial.print(",");
  Serial.print(ball);
  Serial.print(",");
  Serial.print(motor);
  Serial.print(",");
  Serial.println(get_motor_rad(), 3); //print to 3 dec
  digitalWrite(A5,LOW);   // A5 can be used to measure cycle time using an oscilloscope by connecting the scope to the Arduino Box Motor Leads
 
}