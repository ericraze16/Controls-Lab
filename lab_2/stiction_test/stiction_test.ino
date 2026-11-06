#include <Arduino.h>
#include "geeWhiz.h"

const int MOT_PIN = A0;

// ADC values measured on station 5
const float OFFSET = 3.568038909;
const float SLOPE = -0.000381880445;

const float TEST_ANGLES[] = {0.0f, -0.35f, -0.7f, 0.35f, 0.7f};  // rad
const int N_ANGLES = sizeof(TEST_ANGLES) / sizeof(TEST_ANGLES[0]);
const float MAX_ANGLE = 0.8f;  // safety check

// D1[z], Tustin, tau = 0.0231 s, Ts = 1 ms
const double a0 = -0.07356126214266934;
const double a1 = -0.0031170026331643896;
const double a2 = 0.07044425950950561;
const double b0 = 1.0;
const double b1 = -1.9385879020494574;
const double b2 = 0.9398836903182997;
double e1 = 0, e2 = 0, u1 = 0, u2 = 0;

const uint32_t MIN_POSITION_MS = 300;    // let the move finish before measuring stiction
const uint32_t SETTLE_MS = 200;
const float V_SETTLE_TOL = 0.03f;        // V
const float THETA_SETTLE_TOL = 0.003f;   // rad
const float RAMP_RATE = 0.5f / 1000.0f;  // V per ms
const float RAMP_MAX = 3.0f;             // sanity safety checks
const float MOVE_THRESHOLD = 0.01f;      // breakaway limit (rad)
const float FILTER_ALPHA = 0.05f;        // low pass filter theta, ~20 ms time constant

enum State { POSITION, RAMP, ABORTED };
volatile State state = POSITION;
int angle_idx = 0, dir = 1;  // dir +1: ramp up from V_hold, -1: ramp down
uint32_t state_ms = 0, settle_ms = 0;
float theta_f = 0, theta0 = 0, V_ref = 0, th_ref = 0, V_sum = 0, V_hold = 0, V_ramp = 0;

struct Result {
  float target, theta0;
  int dir;
  float V_hold, V_break, moved;
};
volatile Result result;   // last finished ramp
volatile int n_results = 0;

void setup() {
  analogReadResolution(14);
  Serial.begin(115200);
  delay(300);
  geeWhizBegin();
  setMotorVoltage(0.0f);
  theta_f = SLOPE * analogRead(MOT_PIN) + OFFSET;
  Serial.println("target, theta0, ramp, V_hold, breakaway V, motion");
  set_control_interval_ms(1);
}

void loop() {
  static int n_printed = 0;
  if (n_printed < n_results) {
    n_printed++;
    noInterrupts();
    Result r = {result.target, result.theta0, result.dir, result.V_hold, result.V_break, result.moved};
    interrupts();
    Serial.print(r.target, 2);
    Serial.print(", ");
    Serial.print(r.theta0, 4);
    Serial.print(", ");
    Serial.print(r.dir > 0 ? "up" : "down");
    Serial.print(", ");
    Serial.print(r.V_hold, 3);
    Serial.print(", ");
    if (isnan(r.V_break)) Serial.print("none");
    else Serial.print(r.V_break, 3);
    Serial.print(", ");
    Serial.println(r.moved > 0 ? "theta increasing" : "theta decreasing");
  }

  static bool reported = false;
  if (state == ABORTED && !reported) {
    Serial.println("# ABORTED: |theta| > 0.8 rad, motor off");
    reported = true;
  }
}

void interval_control_code(void) {
  float theta = SLOPE * analogRead(MOT_PIN) + OFFSET;
  theta_f += FILTER_ALPHA * (theta - theta_f);
  state_ms++;

  float V = 0.0f;
  if (fabsf(theta) > MAX_ANGLE) state = ABORTED;

  if (state == POSITION) {
    double e = TEST_ANGLES[angle_idx] - theta;
    V = (-b1 * u1 - b2 * u2 + a0 * e + a1 * e1 + a2 * e2) / b0;
    e2 = e1;
    e1 = e;
    u2 = u1;
    u1 = V;

    // settled once V and theta stay inside a small window for SETTLE_MS
    if (fabsf(V - V_ref) > V_SETTLE_TOL || fabsf(theta_f - th_ref) > THETA_SETTLE_TOL) {
      V_ref = V;
      th_ref = theta_f;
      settle_ms = 0;
      V_sum = 0;
    } else {
      settle_ms++;
      V_sum += V;
    }
    if (state_ms >= MIN_POSITION_MS && settle_ms >= SETTLE_MS) {
      V_hold = V_sum / settle_ms;
      theta0 = theta_f;
      V_ramp = 0;
      state = RAMP;
    }
  } else if (state == RAMP) {
    // Once settled enter open loop ramp to find deadzone
    V_ramp += RAMP_RATE;
    V = V_hold + dir * V_ramp;
    float moved = theta_f - theta0;
    bool broke_free = fabsf(moved) > MOVE_THRESHOLD;
    if (broke_free || V_ramp > RAMP_MAX) {
      result.target = TEST_ANGLES[angle_idx];
      result.theta0 = theta0;
      result.dir = dir;
      result.V_hold = V_hold;
      result.V_break = broke_free ? V : NAN;
      result.moved = moved;
      n_results = n_results + 1;

      // other direction, then next angle
      dir = -dir;
      if (dir > 0) angle_idx = (angle_idx + 1) % N_ANGLES;
      state = POSITION;
      state_ms = 0;
      settle_ms = 0;
      V = V_hold;
    }
  }

  setMotorVoltage(V);
}
