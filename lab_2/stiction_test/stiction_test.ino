#include <Arduino.h>
#include "geeWhiz.h"

// Finds where the gear breaks free in each direction at several gear angles,
// for a load that can back-drive the motor.
//
// With friction Fs and a load needing G volts to hold (signed), the gear stays
// still for any voltage in the dead zone [G - Fs, G + Fs]. At each angle:
//   1. POSITION: PI control with stiction offset moves the gear to the angle;
//                the integral term takes over holding the load.
//   2. HOLD:     PI control without the offset; the gear stops with the
//                voltage inside the dead zone. Its mean is V_hold.
//   3. RAMP:     open loop from V_hold, up (or down) until the gear moves.
// The two breakaway voltages are the dead zone edges:
//   V_up = G + Fs, V_down = G - Fs  ->  Fs = (V_up - V_down)/2, G = (V_up + V_down)/2

// ================== Pins ==================
const int MOT_PIN = A0;   // motor angle sensor

// ADC values measured on station 5
const float OFFSET = 3.568038909;
const float SLOPE = -0.000381880445;

// ================== Test plan ==================
const float TEST_ANGLES[] = {0.0f, -0.35f, -0.7f, 0.35f, 0.7f};  // rad, starting from level
const int N_ANGLES = sizeof(TEST_ANGLES) / sizeof(TEST_ANGLES[0]);
const int N_REPEATS = 3;  // ramps per direction per angle
const int N_RESULTS = N_ANGLES * N_REPEATS * 2;
const float PARK_ANGLE = 0.0f;  // held here when the test is done

const uint16_t TS_MS = 1;
const float MAX_ANGLE = 0.8f;  // safety check: motor off and abort beyond this

// ================== Positioning (PI control) ==================
const float K_P = -10.0f;            // V/rad, well damped for 0.35 rad moves
const float K_I = -50.0f;            // V/(rad s), holds the load with zero error
const float I_MAX = 4.0f;            // V, integral clamp
const float I_ZONE = 0.25f;          // rad, integrate only this close to the target
const float MOVE_STICTION = 0.3f;    // V added in the direction of the P output
const float POS_TOLERANCE = 0.01f;   // rad
const uint32_t POSITION_MS = 1500;   // time allowed to reach the test angle
const uint32_t HOLD_MS = 400;        // PI, no offset
const uint32_t HOLD_AVG_MS = 100;    // V_hold = mean voltage over the end of HOLD

// ================== Ramp ==================
const float RAMP_RATE = 0.5f / 1000.0f;  // V per ms (0.5 V/s)
const float RAMP_MAX = 3.0f;             // give up after this much change from V_hold
const float MOVE_THRESHOLD = 0.01f;      // rad of filtered motion that counts as breakaway
const float FILTER_ALPHA = 0.05f;        // EMA on theta, ~20 ms time constant at 1 kHz

// ================== State ==================
enum State { WAIT, POSITION, HOLD, RAMP, DONE, ABORTED };
const char *STATE_NAMES[] = {"WAIT", "POSITION", "HOLD", "RAMP", "DONE", "ABORTED"};
volatile State state = WAIT;
const float START_ANGLE = 0.75f;   // WAIT ends once |theta| stays below this
const uint32_t START_MS = 20;       // short: the load drags the beam while the motor is off
const uint32_t STATUS_MS = 250;    // status line period

int angle_idx = 0;
int rep = 0;
int dir = 1;            // +1: ramp up from V_hold, -1: ramp down
uint32_t state_ms = 0;  // time spent in the current state
float theta_f = 0;      // filtered angle
float theta0 = 0;       // filtered angle at the start of the ramp
float V_sum = 0;        // for averaging the HOLD voltage
float V_hold = 0;
float V_ramp = 0;
float I_v = 0;          // integral term, V
volatile float V_cmd = 0;   // last commanded voltage, for the status line
volatile float theta_now = 0;

struct Result {
  float target;   // test angle
  float theta0;   // actual angle at the start of the ramp
  int dir;        // +1 ramp up, -1 ramp down
  float V_hold;   // holding voltage the ramp started from
  float V_break;  // signed voltage at breakaway, NAN if no motion
  float moved;    // signed motion that was detected
};

Result results[N_RESULTS];
volatile int n_results = 0;
int n_printed = 0;

void set_voltage(float V) {
  V_cmd = V;
  setMotorVoltage(V);
}

float get_motor_rad() {
  int raw = analogRead(MOT_PIN);
  return (SLOPE * raw) + OFFSET;
}

void enter(State s) {
  state = s;
  state_ms = 0;
}

// PI control toward target, optionally with the stiction offset
float pi_control(float target, float theta, bool with_offset) {
  float e = target - theta;
  if (fabsf(e) < I_ZONE) I_v = constrain(I_v + K_I * e * TS_MS / 1000.0f, -I_MAX, I_MAX);
  float V = K_P * e + I_v;
  if (with_offset && fabsf(e) > POS_TOLERANCE) V += (V > 0 ? MOVE_STICTION : -MOVE_STICTION);
  return V;
}

// ================== Setup ==================
void setup() {
  analogReadResolution(14);
  Serial.begin(115200);
  delay(300);

  geeWhizBegin();
  setMotorVoltage(0.0f);
  theta_f = get_motor_rad();

  Serial.println("Stiction test started");
  Serial.println("Waiting for |theta| < 0.75 rad: move the beam into range by hand if needed");
  Serial.println("target, theta0, ramp, V_hold, breakaway V, motion");

  set_control_interval_ms(TS_MS);
}

// ================== Loop ==================
void loop() {
  static uint32_t last_status = 0;
  if (state != DONE && millis() - last_status >= STATUS_MS) {
    last_status = millis();
    Serial.print("# ");
    Serial.print(STATE_NAMES[state]);
    Serial.print("  angle ");
    Serial.print(angle_idx);
    Serial.print("  theta ");
    Serial.print(theta_now, 3);
    Serial.print("  V ");
    Serial.println(V_cmd, 3);
  }

  while (n_printed < n_results) {
    Result r = results[n_printed++];
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
  if (reported) return;
  if (state == ABORTED) {
    Serial.print("ABORTED: |theta| exceeded the safety limit, motor off. theta = ");
    Serial.println(theta_now, 3);
    reported = true;
  } else if (state == DONE) {
    print_summary();
    reported = true;
  }
}

// Mean dead zone edges per angle, then friction and load
void print_summary() {
  Serial.println();
  Serial.println("Summary (mean of repeats, signed volts)");
  Serial.println("target, V_up, V_down, friction Fs, load G");
  for (int a = 0; a < N_ANGLES; a++) {
    float sum[2] = {0, 0};
    int cnt[2] = {0, 0};
    for (int i = 0; i < n_results; i++) {
      if (results[i].target != TEST_ANGLES[a] || isnan(results[i].V_break)) continue;
      int d = results[i].dir > 0 ? 0 : 1;
      sum[d] += results[i].V_break;
      cnt[d]++;
    }
    float v_up = cnt[0] ? sum[0] / cnt[0] : NAN;
    float v_down = cnt[1] ? sum[1] / cnt[1] : NAN;
    Serial.print(TEST_ANGLES[a], 2);
    Serial.print(", ");
    Serial.print(v_up, 3);
    Serial.print(", ");
    Serial.print(v_down, 3);
    Serial.print(", ");
    Serial.print((v_up - v_down) / 2, 3);
    Serial.print(", ");
    Serial.println((v_up + v_down) / 2, 3);
  }
}

// ================== Control ISR ==================
void interval_control_code(void) {
  float theta = get_motor_rad();
  theta_f += FILTER_ALPHA * (theta - theta_f);
  theta_now = theta;
  state_ms += TS_MS;

  if (state == ABORTED) {
    set_voltage(0.0f);
    return;
  }

  // Motor off until the gear has been in range for a while
  if (state == WAIT) {
    set_voltage(0.0f);
    if (fabsf(theta) > START_ANGLE) state_ms = 0;
    if (state_ms >= START_MS) enter(POSITION);
    return;
  }

  // Safety check, +-0.8 rad
  if (fabsf(theta) > MAX_ANGLE) {
    set_voltage(0.0f);
    enter(ABORTED);
    return;
  }

  switch (state) {
    case POSITION:
      set_voltage(pi_control(TEST_ANGLES[angle_idx], theta, true));
      if (state_ms >= POSITION_MS) {
        V_sum = 0;
        enter(HOLD);
      }
      break;

    case HOLD: {
      float V = pi_control(TEST_ANGLES[angle_idx], theta, false);
      set_voltage(V);
      if (state_ms > HOLD_MS - HOLD_AVG_MS) V_sum += V;
      if (state_ms >= HOLD_MS) {
        V_hold = V_sum / HOLD_AVG_MS;
        theta0 = theta_f;
        V_ramp = 0.0f;
        enter(RAMP);
      }
      break;
    }

    case RAMP: {
      V_ramp += RAMP_RATE * TS_MS;
      float V = V_hold + dir * V_ramp;
      float moved = theta_f - theta0;
      bool broke_free = fabsf(moved) > MOVE_THRESHOLD;
      if (broke_free || V_ramp > RAMP_MAX) {
        results[n_results] = {TEST_ANGLES[angle_idx], theta0, dir, V_hold,
                              broke_free ? V : NAN, moved};
        n_results = n_results + 1;

        // next: other direction, then next repeat, then next angle
        if (dir > 0) {
          dir = -1;
        } else {
          dir = 1;
          if (++rep >= N_REPEATS) {
            rep = 0;
            if (++angle_idx >= N_ANGLES) {
              enter(DONE);
              break;
            }
          }
        }
        enter(POSITION);  // closed loop again right away, so the load can't drop the beam
      } else {
        set_voltage(V);
      }
      break;
    }

    case DONE:
      // keep the beam up instead of letting the load back-drive it
      set_voltage(pi_control(PARK_ANGLE, theta, true));
      break;

    default:
      break;
  }
}
