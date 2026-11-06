#include <Arduino.h>
#include "geeWhiz.h"

// ================== Pins ==================
const int MOT_PIN = A0;   // motor angle sensor
const int BAL_PIN = A1;   // ball position sensor
const int TIMING_PIN = A5;  // scope this to measure ISR execution time

// ADC values measured on station 5
const float OFFSET = 3.568038909;
const float SLOPE = -0.000381880445;

// ================== Timing ==================
const uint16_t TS_MS = 1;                   // must match the Ts used for D1[z]
const uint32_t HALF_PERIOD_SAMPLES = 1000;  // 1 s at Ts = 1 ms -> 2 s period, 50% duty

// ================== Reference ==================
const float REF_AMPLITUDE = 0.7f;  // square wave between -0.7 and +0.7 rad
const float REF_SAT = 0.7f;        // gear angle saturator on theta_ref
const float SAFETY_ANGLE = 0.8f;   // motor off if |theta| exceeds this

// ================== Stiction ==================
const float STICTION_POS = 0.15f;  // V added when driving in the + direction
const float STICTION_NEG = 0.15f;  // V added when driving in the - direction
const float POS_TOLERANCE = 0.005f;

// ================== Controller ==================
// D1[z] = (a0 + a1 z^-1 + a2 z^-2) / (b0 + b1 z^-1 + b2 z^-2)
// Tustin discretization of C1(s) = (-148.5178047 s - 6429.342196) / (s^2 + 62 s + 1336.390625),
// designed for tau = 0.0231 s, Ts = 1 ms
const double a0 = -0.07356126214266934;
const double a1 = -0.0031170026331643896;
const double a2 = 0.07044425950950561;
const double b0 = 1.0;
const double b1 = -1.9385879020494574;
const double b2 = 0.9398836903182997;

double e1 = 0, e2 = 0;  // e[k-1], e[k-2]
double u1 = 0, u2 = 0;  // u[k-1], u[k-2]
uint32_t k = 0;         // sample counter

// ================== Logging ==================
// The ISR runs too often to print every sample, so it queues every
// LOG_DIV-th sample and loop() prints them.
const uint32_t LOG_DIV = 5;  // 5 ms between logged samples
const int LOG_SIZE = 128;

struct Sample {
  uint32_t t_ms;
  float theta_ref_sat;
  float theta;
  float V;
};

Sample log_buf[LOG_SIZE];
volatile int log_head = 0;
volatile int log_tail = 0;

// ================== Setup ==================
void setup() {

  analogReadResolution(14);
  pinMode(TIMING_PIN, OUTPUT);
  Serial.begin(115200);
  delay(300);

  geeWhizBegin();
  setMotorVoltage(0.0f);

  Serial.println("geeWhiz Started");
  Serial.println("t_ms, theta_ref_sat, theta, V");

  set_control_interval_ms(TS_MS);
}

// ================== Loop ==================
void loop() {
  while (log_tail != log_head) {
    noInterrupts();
    Sample s = log_buf[log_tail];
    log_tail = (log_tail + 1) % LOG_SIZE;
    interrupts();

    Serial.print(s.t_ms);
    Serial.print(", ");
    Serial.print(s.theta_ref_sat, 4);
    Serial.print(", ");
    Serial.print(s.theta, 4);
    Serial.print(", ");
    Serial.println(s.V, 3);
  }
}

float get_motor_rad() {
  int raw = analogRead(MOT_PIN);
  return (SLOPE * raw) + OFFSET;
}

// ================== Control ISR ==================
void interval_control_code(void) {
  digitalWrite(TIMING_PIN, HIGH);

  float theta = get_motor_rad();

  // Square wave reference, starting at -0.7 rad
  float theta_ref = ((k / HALF_PERIOD_SAMPLES) % 2 == 0) ? -REF_AMPLITUDE : REF_AMPLITUDE;
  float theta_ref_sat = constrain(theta_ref, -REF_SAT, REF_SAT);

  // u[k] = (1/b0)(-b1 u1 - b2 u2 + a0 e + a1 e1 + a2 e2)
  double e = theta_ref_sat - theta;
  double u = (-b1 * u1 - b2 * u2 + a0 * e + a1 * e1 + a2 * e2) / b0;
  e2 = e1;
  e1 = e;
  u2 = u1;
  u1 = u;  // controller history holds u, not the stiction-compensated voltage

  float V = u;
  if (fabs(e) > POS_TOLERANCE) {
    if (V > 0) V += STICTION_POS;
    else if (V < 0) V -= STICTION_NEG;
  }
  if (fabs(theta) > SAFETY_ANGLE)
    V = 0.0f;
  setMotorVoltage(V);

  if (k % LOG_DIV == 0) {
    int next = (log_head + 1) % LOG_SIZE;
    if (next != log_tail) {  // drop the sample if the buffer is full
      log_buf[log_head] = {k * TS_MS, theta_ref_sat, theta, V};
      log_head = next;
    }
  }
  k++;

  digitalWrite(TIMING_PIN, LOW);
}
