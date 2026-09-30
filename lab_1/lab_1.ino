#include <Arduino.h>
#include "geeWhiz.h"

// ================== Pins ==================
int MOT_PIN = A0;   // motor angle sensor
int BAL_PIN = A1;   // ball position sensor

// ADC values measured on station 5
const float OFFSET = 3.568038909;
const float SLOPE = -0.000381880445;

// System params
const float SATURATION_ANGLE = PI/4f;
const float POS_TOLERANCE = 0.0005f; 
const float STICTION_VOLTAGE = 0.3;
float K_p = -20;

// ================== Setup ==================
void setup() {

  analogReadResolution(14);
  pinMode(A5, OUTPUT);   // A5 can be used to measure cycle time using an oscilloscope by connecting the scope to the Arduino Box Motor Leads
  Serial.begin(115200);
  delay(300);

  geeWhizBegin();                 
  set_control_interval_ms(10);
  setMotorVoltage(0.0f);

  Serial.println("geeWhiz Started");
  Serial.println("time, motor_raw");
}

// ================== Loop =================
void loop() {
  hold_pos(-0.1, 1000);
  hold_pos( 0.1, 1000);
}

float get_motor_rad() {
  int motor_pos_raw = analogRead(MOT_PIN);
  return (SLOPE * motor_pos_raw) + OFFSET;
}

float get_linear_radians() {
    int raw = analogRead(MOT_PIN);
    return (SLOPE * raw) + OFFSET;
}

void hold_pos(float theta_ref, unsigned long duration_ms) {
  theta_ref = constrain(theta_ref, -SATURATION_ANGLE, SATURATION_ANGLE);
  unsigned long t0 = millis();
  while (millis() - t0 < duration_ms) {
    float theta = get_linear_radians()
    float e = theta_ref - theta;
    float V = K_p * e;
    if (fabs(e) > POS_TOLERANCE)
      V += (V > 0 ? STICTION_VOLTAGE : -STICTION_VOLTAGE);
    setMotorVoltage(V);
    delay(1);
  }
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
  Serial.println(motor);
  digitalWrite(A5,LOW);   // A5 can be used to measure cycle time using an oscilloscope by connecting the scope to the Arduino Box Motor Leads
 
}