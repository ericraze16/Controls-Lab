#include <Arduino.h>
#include "geeWhiz.h"

// Lab 3 (b): ball position control with the part (a) lead controller.
//   outer loop (10 ms): e_y = y_ref - y -> D2[z] -> saturator -> theta_ref_sat
//   inner loop (1 ms):  e = theta_ref_sat - theta -> D1[z] + load/friction comp -> V
// D2[z] = Tustin of C2(s) = -7 (s + 0.35) / (s + 2.5) at 10 ms.
// Reference: square wave 0.10 m <-> 0.25 m, 20 s period.

const int MOT_PIN = A0;
const int BAL_PIN = A1;

// Sensor scaling: gear angle from Lab 1 (station 5), ball recalibrated 2026-11-26 at the beam ticks
const float SLOPE_MOTOR = -0.000381880445, OFFSET_MOTOR = 3.568038909;        // rad
const float SLOPE_BALL = 0.0000252403332759, OFFSET_BALL = 0.00487879462523;  // m

const uint32_t OUTER_DIV = 10;        // outer loop every 10 inner (1 ms) ticks
const uint32_t HALF_PERIOD_MS = 10000;
const float Y_LOW = 0.10f, Y_HIGH = 0.25f;   // m
const float REF_SAT = 0.7f;           // gear angle saturator, rad
const float MAX_ANGLE = 0.8f;         // safety: motor off beyond this, rad

// Inner loop (Lab 2): D1[z] and load/friction compensation
const float LOAD_V = -0.275f, FRICTION_V = 0.30f, POS_TOLERANCE = 0.005f;
const double a0 = -0.07356126214266934, a1 = -0.0031170026331643896, a2 = 0.07044425950950561;
const double b1 = -1.9385879020494574, b2 = 0.9398836903182997;
double e1 = 0, e2 = 0, u1 = 0, u2 = 0;

// Outer loop: D2[z] = (c0 + c1 z^-1) / (1 + d1 z^-1), m -> rad
const double c0 = -6.925679012345679, c1 = 6.901481481481482, d1 = -0.9753086419753088;
double ey1 = 0, th1 = 0;              // previous error and previous (unsaturated) output
float theta_ref_sat = 0.0f;

uint32_t k = 0;
int32_t ball_sum = 0;                 // sums the 1 ms ball readings over one outer sample

volatile bool fresh = false;          // one sample handed from the interrupt to loop()
volatile uint32_t log_t;
volatile float log_ref, log_y, log_th_ref, log_th, log_V;

void setup() {
  analogReadResolution(14);
  Serial.begin(115200);
  delay(300);
  geeWhizBegin();
  setMotorVoltage(0.0f);
  Serial.println("t_ms, y_ref, y, theta_ref_sat, theta, V");
  set_control_interval_ms(1);
}

void loop() {
  if (!fresh) return;
  noInterrupts();
  uint32_t t = log_t;
  float r = log_ref, y = log_y, tr = log_th_ref, th = log_th, V = log_V;
  fresh = false;
  interrupts();
  Serial.print(t);        Serial.print(", ");
  Serial.print(r, 3);     Serial.print(", ");
  Serial.print(y, 4);     Serial.print(", ");
  Serial.print(tr, 4);    Serial.print(", ");
  Serial.print(th, 4);    Serial.print(", ");
  Serial.println(V, 3);
}

void interval_control_code(void) {
  float theta = SLOPE_MOTOR * analogRead(MOT_PIN) + OFFSET_MOTOR;
  ball_sum += analogRead(BAL_PIN);

  // ---- inner loop, every 1 ms ----
  float V = 0.0f;
  if (fabsf(theta) <= MAX_ANGLE) {
    double e = theta_ref_sat - theta;
    double u = -b1 * u1 - b2 * u2 + a0 * e + a1 * e1 + a2 * e2;
    e2 = e1; e1 = e;
    u2 = u1; u1 = u;
    V = u + LOAD_V;
    if (fabs(e) > POS_TOLERANCE) V += (u > 0 ? FRICTION_V : -FRICTION_V);
  }
  setMotorVoltage(V);

  // ---- outer loop, every 10 ms ----
  if (k % OUTER_DIV == OUTER_DIV - 1) {
    float y = SLOPE_BALL * ((float)ball_sum / OUTER_DIV) + OFFSET_BALL;
    ball_sum = 0;
    float y_ref = ((k / HALF_PERIOD_MS) % 2 == 0) ? Y_LOW : Y_HIGH;

    double ey = y_ref - y;
    double th_ref = -d1 * th1 + c0 * ey + c1 * ey1;
    ey1 = ey; th1 = th_ref;           // keep the unsaturated output in the controller history
    theta_ref_sat = constrain((float)th_ref, -REF_SAT, REF_SAT);

    log_t = k; log_ref = y_ref; log_y = y; log_th_ref = theta_ref_sat; log_th = theta; log_V = V;
    fresh = true;
  }
  k++;
}
