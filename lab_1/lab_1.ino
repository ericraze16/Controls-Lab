#include <Arduino.h>
#include "geeWhiz.h"

// ================== Pins ==================
int MOT_PIN = A0;   // motor angle sensor
int BAL_PIN = A1;   // ball position sensor

// ADC values measured on station 5
const int ADC_CENTER = 9250; // 0 rad
const int ADC_AMPLITUDE = 2100; // +- PI/4 rad

const float OFFSET = 3.568038909;
const float SLOPE = -0.000381880445;

// negative voltage spins clockwise
const int SATURATION_VOLTAGE = -6;  
const float POS_TOLERANCE = 0.0005f; 

float K_p = -200;

// ================== Setup ==================
void setup() {

  analogReadResolution(14);
  pinMode(A5, OUTPUT);   // A5 can be used to measure cycle time using an oscilloscope by connecting the scope to the Arduino Box Motor Leads
  Serial.begin(115200);
  delay(300);

  geeWhizBegin();                 
  // set_control_interval_ms(100);
  setMotorVoltage(0.0f);

  Serial.println("geeWhiz Started");
  Serial.println("time, maxY, minY, ball, motor_raw, motor_rad");
}

// ================== Loop ==================
void loop() {
  // Step input to measure overshoot and peak time
  move_to_pos(-0.1);
  delay(1000);
  move_to_pos(0.1);
  delay(1000);
}

// Convert motor pos to radians using system characterization params
float get_motor_rad() {
  int motor_pos_raw = analogRead(MOT_PIN);
  return (SLOPE * motor_pos_raw) + OFFSET;
}

void move_to_pos(float target_rad) {
  float current_rad = get_linear_radians();
  float error = target_rad - current_rad;

  Serial.println("Time (us), current_rad, target_rad");
  Serial.print("K_p: ");
  Serial.println(K_p);

  while (fabs(error) > POS_TOLERANCE) {
    current_rad = get_linear_radians();
    error = target_rad - current_rad;

    float voltage = K_p * error;

    float max_volts = fabs((float)SATURATION_VOLTAGE);
    voltage = constrain(voltage, -max_volts, max_volts);

    setMotorVoltage(voltage);

    Serial.print(micros());
    Serial.print(", ");
    Serial.print(current_rad, 4);
    Serial.print(", ");
    Serial.println(target_rad, 4);

    delayMicroseconds(50);
  }

  setMotorVoltage(0.0f);
  Serial.println("Pos achieved");
}

float get_linear_radians() {
    int raw = analogRead(MOT_PIN);
    return (SLOPE * raw) + OFFSET;
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
  Serial.print(", ");
  // Serial.print(maxy);
  //   Serial.print(",");
  // Serial.print(miny);
  //   Serial.print(",");
  // Serial.print(ball);
  // Serial.print(",");
  Serial.println(motor);
  // Serial.print(",");
  // Serial.println(get_motor_rad(), 3); //print to 3 dec
  digitalWrite(A5,LOW);   // A5 can be used to measure cycle time using an oscilloscope by connecting the scope to the Arduino Box Motor Leads
 
}