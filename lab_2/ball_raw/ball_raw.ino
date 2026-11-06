#include <Arduino.h>
#include "geeWhiz.h"

// K3 experiment: holds the gear at THETA_HOLD with the Lab 2 controller and
// logs every 10 ms. Place the ball against the high end stop and let go; it
// rolls from rest with constant acceleration a = K2 K3 theta.

const float THETA_HOLD = -0.2f;  // rad; change and re-upload for other angles

const int MOT_PIN = A0;
const int BAL_PIN = A1;

// ADC values measured on station 5
const float OFFSET_MOTOR = 3.568038909;
const float SLOPE_MOTOR = -0.000381880445;
const float SLOPE_BALL = 0.00002470155253;  // m per count
const float OFFSET_BALL = 0.01162435967;    // m

const uint16_t TS_MS = 1;
const float MAX_ANGLE = 0.8f;  // safety check: motor off if |theta| exceeds this

// Load and friction compensation from the beam stiction tests
const float LOAD_V = -0.275f;     // always applied, cancels the beam's pull
const float FRICTION_V = 0.30f;   // added in the direction of u when |e| > tolerance
const float POS_TOLERANCE = 0.005f;

// Tustin D1[z] for tau = 0.0231 s, Ts = 1 ms (same as lab_2.ino)
const double a0 = -0.07356126214266934;
const double a1 = -0.0031170026331643896;
const double a2 = 0.07044425950950561;
const double b0 = 1.0;
const double b1 = -1.9385879020494574;
const double b2 = 0.9398836903182997;

double e1 = 0, e2 = 0, u1 = 0, u2 = 0;
uint32_t k = 0;

// ================== Logging ==================
// Every LOG_DIV-th sample is queued by the ISR and printed by loop()
const uint32_t LOG_DIV = 10;  // 10 ms between logged samples
const int LOG_SIZE = 128;

struct Sample {
  uint32_t t_ms;
  float theta;
  float V;
  int ball;
};

Sample log_buf[LOG_SIZE];
volatile int log_head = 0;
volatile int log_tail = 0;
volatile uint32_t n_dropped = 0;

void setup() {
  analogReadResolution(14);
  Serial.begin(115200);
  delay(300);

  geeWhizBegin();
  setMotorVoltage(0.0f);

  Serial.print("Holding theta = ");
  Serial.println(THETA_HOLD, 3);
  Serial.println("t_ms, theta_ref, theta, V, ball_raw, ball_m, dropped");

  set_control_interval_ms(TS_MS);
}

void loop() {
  while (log_tail != log_head) {
    noInterrupts();
    Sample s = log_buf[log_tail];
    log_tail = (log_tail + 1) % LOG_SIZE;
    interrupts();

    Serial.print(s.t_ms);
    Serial.print(", ");
    Serial.print(THETA_HOLD, 3);
    Serial.print(", ");
    Serial.print(s.theta, 4);
    Serial.print(", ");
    Serial.print(s.V, 3);
    Serial.print(", ");
    Serial.print(s.ball);
    Serial.print(", ");
    Serial.print(SLOPE_BALL * s.ball + OFFSET_BALL, 4);
    Serial.print(", ");
    Serial.println(n_dropped);
  }
}

void interval_control_code(void) {
  float theta = SLOPE_MOTOR * analogRead(MOT_PIN) + OFFSET_MOTOR;
  int ball = analogRead(BAL_PIN);
  float V = 0.0f;

  if (fabsf(theta) <= MAX_ANGLE) {
    double e = THETA_HOLD - theta;
    double u = (-b1 * u1 - b2 * u2 + a0 * e + a1 * e1 + a2 * e2) / b0;
    e2 = e1;
    e1 = e;
    u2 = u1;
    u1 = u;

    V = u + LOAD_V;
    if (fabs(e) > POS_TOLERANCE) V += (u > 0 ? FRICTION_V : -FRICTION_V);
  }
  setMotorVoltage(V);

  if (k % LOG_DIV == 0) {
    int next = (log_head + 1) % LOG_SIZE;
    if (next != log_tail) {
      log_buf[log_head] = {k * TS_MS, theta, V, ball};
      log_head = next;
    } else {
      n_dropped = n_dropped + 1;  // shows up in the last column if printing falls behind
    }
  }
  k++;
}
