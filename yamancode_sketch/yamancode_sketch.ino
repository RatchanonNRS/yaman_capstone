/*
 * yamancode.cpp — AGV Arduino Mega Firmware
 *
 * Hardware:
 *   Encoders  : E6B2-CWZ6C 600P/R
 *     Left  OutA→D2,  OutB→D3
 *     Right OutA→D18, OutB→D19
 *   Motor Driver: Cytron MDD20A
 *     Left  PWM→D6,  DIR→D22
 *     Right PWM→D7,  DIR→D23
 *   IMU : MPU6050 moved to RPi I2C (GPIO 2/3) — handled by imu_node.py
 *
 * Gear train:
 *   Motor gear 9T → Encoder gear 9T (1:1 with motor)
 *   Motor gear 9T → Wheel gear 32T
 *   => counts per wheel rev = 600 × 4 × (32/9) ≈ 8533.33
 *   => wheel circumference  = π × 0.200 m = 0.6283 m
 *   => meters per count     = 0.6283 / 8533.33 ≈ 7.363e-5 m
 *   Wheelbase (L)           = 0.445 m  (from SolidWorks CAD)
 *
 * ── Serial protocol (115200 baud, '\n' terminated) ──────────────────────────
 *
 *  RPi (or keyboard) → Arduino
 *  ┌──────────────────────────────────────────────────────────────────────────┐
 *  │ "V:<vx>,<wz>\n"    velocity command from ROS2 bridge                    │
 *  │   vx  = linear  velocity (m/s)  e.g.  0.30                              │
 *  │   wz  = angular velocity (rad/s) e.g. -0.50                             │
 *  │                                                                          │
 *  │ Single-char keyboard teleop (same keys as teleop_twist_keyboard):        │
 *  │   i = forward       , = backward                                         │
 *  │   j = turn left     l = turn right                                       │
 *  │   u = fwd+left      o = fwd+right                                        │
 *  │   m = bwd+left      . = bwd+right                                        │
 *  │   k = STOP                                                               │
 *  │   + / - = increase / decrease speed step                                 │
 *  └──────────────────────────────────────────────────────────────────────────┘
 *
 *  Arduino → RPi  (50 Hz)
 *  ┌──────────────────────────────────────────────────────────────────────────┐
 *  │ "O:<x>,<y>,<th>,<vl>,<vr>\n"   odometry                                 │
 *  │   x, y  in metres  (float, 4 dp)                                        │
 *  │   th    in radians (float, 4 dp)                                        │
 *  │   vl,vr actual wheel velocities (m/s, 4 dp)                             │
 *  └──────────────────────────────────────────────────────────────────────────┘
 */

#include <Arduino.h>
#include <math.h>
#include <Servo.h>

// ── Pin definitions ──────────────────────────────────────────────────────────
#define ENC_L_A   2
#define ENC_L_B   3
#define ENC_R_A  18
#define ENC_R_B  19

#define MOT_L_PWM  6
#define MOT_L_DIR 22
#define MOT_R_PWM  7
#define MOT_R_DIR 23

// ── Robot geometry ───────────────────────────────────────────────────────────
constexpr float COUNTS_PER_REV  = 600.0f * 4.0f * (32.0f / 9.0f);
constexpr float WHEEL_DIAM_M    = 0.200f;
constexpr float WHEEL_CIRCUM_M  = 3.14159265f * WHEEL_DIAM_M;
constexpr float M_PER_COUNT     = WHEEL_CIRCUM_M / COUNTS_PER_REV;
constexpr float WHEELBASE_M     = 0.445f;

// ── PID gains (tune on real robot) ───────────────────────────────────────────
constexpr float PID_KP           = 300.0f;
constexpr float PID_KI           = 20.0f;
constexpr float PID_KD           = 8.0f;
constexpr float PID_INTEGRAL_MAX = 40.0f;
constexpr float VEL_DEADBAND     = 0.03f;  // m/s — targets below this cut motors
constexpr float MAX_VEL_MS       = 0.5f;

// ── Encoder state (ISR-updated) ──────────────────────────────────────────────
volatile int32_t enc_left  = 0;
volatile int32_t enc_right = 0;

void enc_left_a()  { digitalRead(ENC_L_A) == digitalRead(ENC_L_B) ? enc_left--  : enc_left++;  }
void enc_left_b()  { digitalRead(ENC_L_A) == digitalRead(ENC_L_B) ? enc_left++  : enc_left--;  }
void enc_right_a() { digitalRead(ENC_R_A) == digitalRead(ENC_R_B) ? enc_right++ : enc_right--; }
void enc_right_b() { digitalRead(ENC_R_A) == digitalRead(ENC_R_B) ? enc_right-- : enc_right++; }

// ── Motor driver ─────────────────────────────────────────────────────────────
void setMotor(uint8_t pwmPin, uint8_t dirPin, int16_t speed) {
  speed = constrain(speed, -255, 255);
  digitalWrite(dirPin, speed >= 0 ? HIGH : LOW);
  analogWrite(pwmPin, abs(speed));
}

// ── PID state per wheel ───────────────────────────────────────────────────────
struct PID {
  float target    = 0.0f;
  float integral  = 0.0f;
  float prev_err  = 0.0f;

  int16_t compute(float actual, float dt) {
    if (dt <= 0.0f) return 0;
    float err   = target - actual;
    integral   += err * dt;
    integral    = constrain(integral, -PID_INTEGRAL_MAX, PID_INTEGRAL_MAX);
    float deriv = (err - prev_err) / dt;
    prev_err    = err;
    float out   = PID_KP * err + PID_KI * integral + PID_KD * deriv;
    return (int16_t)constrain(out, -255.0f, 255.0f);
  }

  void reset() { integral = 0.0f; prev_err = 0.0f; }
};

PID pid_left, pid_right;

// ── Odometry state ────────────────────────────────────────────────────────────
float   odom_x   = 0.0f;
float   odom_y   = 0.0f;
float   odom_th  = 0.0f;
float   vel_left_actual  = 0.0f;
float   vel_right_actual = 0.0f;
int32_t enc_left_prev    = 0;
int32_t enc_right_prev   = 0;

// ── Watchdog ──────────────────────────────────────────────────────────────────
uint32_t last_cmd_ms = 0;
constexpr uint32_t CMD_TIMEOUT_MS = 200;

// ── Timed brake ───────────────────────────────────────────────────────────────
// When target transitions to 0, apply fixed reverse PWM for BRAKE_DURATION_MS.
// Direction determined from the old target (not noisy encoder velocity).
uint32_t brake_until_ms = 0;
int8_t   brake_dir_l    = 0;  // -1, 0, +1
int8_t   brake_dir_r    = 0;
constexpr uint32_t BRAKE_DURATION_MS = 200;
constexpr int16_t  BRAKE_PWM         = 0;     // DISABLED — was causing Pi brownout; slowdown zone handles precision stop

// ── Keyboard teleop speed step ───────────────────────────────────────────────
float key_vx = 0.20f;
float key_wz = 0.40f;

// ── Heartbeat (debug) ─────────────────────────────────────────────────────────
uint32_t last_hb_ms = 0;
constexpr uint32_t HB_INTERVAL_MS = 3000;

void startBrake() {
  brake_dir_l    = (pid_left.target  > VEL_DEADBAND) ? -1 : (pid_left.target  < -VEL_DEADBAND) ? 1 : 0;
  brake_dir_r    = (pid_right.target > VEL_DEADBAND) ? -1 : (pid_right.target < -VEL_DEADBAND) ? 1 : 0;
  brake_until_ms = millis() + BRAKE_DURATION_MS;
  Serial.print("INFO:BRAKE:");
  Serial.print(brake_dir_l);
  Serial.print(",");
  Serial.println(brake_dir_r);
}

void setVelocity(float vx, float wz) {
  vx = constrain(vx, -MAX_VEL_MS, MAX_VEL_MS);
  float new_l = constrain(vx - wz * (WHEELBASE_M / 2.0f), -MAX_VEL_MS, MAX_VEL_MS);
  float new_r = constrain(vx + wz * (WHEELBASE_M / 2.0f), -MAX_VEL_MS, MAX_VEL_MS);
  // Only start brake on transition FROM moving TO stop (not on repeated V:0,0 calls)
  if (fabsf(new_l) < VEL_DEADBAND && fabsf(new_r) < VEL_DEADBAND &&
      (fabsf(pid_left.target) >= VEL_DEADBAND || fabsf(pid_right.target) >= VEL_DEADBAND)) {
    startBrake();
  }
  pid_left.target  = new_l;
  pid_right.target = new_r;
  last_cmd_ms = millis();
}

// ── Sequence (mock) ───────────────────────────────────────────────────────────
// Real hardware: replace mock delays with actual actuator control + sensor reads
// Mock pressure: fails on first attempt once per mission, succeeds on retry

bool seq_running      = false;
uint8_t seq_step      = 0;
uint32_t seq_step_ms  = 0;   // when current step started
bool mock_fail_done   = false; // track if we already did one fake fail this mission

void seqStep(uint8_t n, const char* desc, uint16_t duration_ms) {
  Serial.print("SEQ:STEP:");
  Serial.print(n);
  Serial.print(":");
  Serial.println(desc);
  seq_step_ms = millis();
  seq_step    = n;
  (void)duration_ms; // used by caller via delay pattern
}

// Called from loop() — non-blocking sequence runner using millis()
// Each step waits SEQ_STEP_MS before advancing (mock duration)
#define SEQ_STEP_MS 800

// Vacuum retry state
uint8_t vacuum_retries = 0;
#define VACUUM_MAX_RETRIES 3

void runSequence() {
  if (!seq_running) return;
  if (millis() - seq_step_ms < SEQ_STEP_MS) return;  // wait for step duration

  switch (seq_step) {
    // ── PHASE 2: Retrieve medicine box ─────────────────────────────────────
    case 0:  seqStep(3,  "Vertical rail UP",              SEQ_STEP_MS); break;
    case 3:  seqStep(4,  "Horizontal rail EXTEND",        SEQ_STEP_MS); break;
    case 4:  seqStep(5,  "Gripper GRIP box",              SEQ_STEP_MS); break;
    case 5:  seqStep(6,  "Horizontal rail RETRACT",       SEQ_STEP_MS); break;
    case 6:  seqStep(7,  "Vertical rail DOWN",            SEQ_STEP_MS); break;

    // ── PHASE 3: Medicine extraction ───────────────────────────────────────
    case 7:  seqStep(8,  "SCARA move to box",             SEQ_STEP_MS); break;
    case 8:  seqStep(9,  "Camera capture image",          SEQ_STEP_MS); break;
    case 9:  seqStep(11, "Vacuum pump OPEN",              SEQ_STEP_MS); break;
    case 11: seqStep(12, "EE rack extend DOWN",           SEQ_STEP_MS); break;
    case 12: {
      // Mock pressure check — fail once per mission, then succeed
      bool pressure_ok = mock_fail_done || (random(0, 100) > 30);
      if (!pressure_ok && vacuum_retries < VACUUM_MAX_RETRIES) {
        vacuum_retries++;
        mock_fail_done = true;
        Serial.print("SEQ:RETRY:");
        Serial.println(vacuum_retries);
        // Retract, shift, try again
        seqStep(12, "EE rack extend DOWN (retry)",        SEQ_STEP_MS);
      } else if (vacuum_retries >= VACUUM_MAX_RETRIES && !pressure_ok) {
        Serial.println("SEQ:FAIL:VACUUM_NO_PRESSURE");
        seq_running = false;
      } else {
        seqStep(14, "Rack retract UP (pack gripped)",     SEQ_STEP_MS);
      }
      break;
    }
    case 14: seqStep(15, "SCARA move to container",       SEQ_STEP_MS); break;
    case 15: seqStep(16, "Vacuum pump CLOSE — pack in container", SEQ_STEP_MS); break;

    // ── PHASE 4: Return medicine box ───────────────────────────────────────
    case 16: seqStep(17, "SCARA retract to safe position",SEQ_STEP_MS); break;
    case 17: seqStep(18, "Vertical rail UP",              SEQ_STEP_MS); break;
    case 18: seqStep(19, "Horizontal rail EXTEND",        SEQ_STEP_MS); break;
    case 19: seqStep(20, "Gripper PLACE box on shelf",    SEQ_STEP_MS); break;
    case 20: seqStep(21, "Gripper RELEASE",               SEQ_STEP_MS); break;
    case 21: seqStep(22, "Horizontal rail RETRACT",       SEQ_STEP_MS); break;
    case 22: seqStep(23, "Vertical rail DOWN",            SEQ_STEP_MS); break;

    case 23:
      Serial.println("SEQ:DONE");
      seq_running    = false;
      vacuum_retries = 0;
      mock_fail_done = false;
      break;

    default:
      seq_step++;
      seq_step_ms = millis();
      break;
  }
}

// ── Real hardware sequence (included here so runRealSequence() is visible to parseCommand below) ──
#include "sequence_hardware.h"

// ── Serial parsing ────────────────────────────────────────────────────────────
String serialBuf = "";

void parseCommand(const String &cmd) {
  if (cmd.startsWith("SEQ:START")) {
    seq_running = false;  // disable mock runner
    Serial.println("SEQ:STEP:1:AGV at shelf - sequence starting");
    runRealSequence();    // blocking — returns when done or failed
    return;
  }

  if (cmd.startsWith("V:")) {
    int comma = cmd.indexOf(',', 2);
    if (comma < 0) return;
    float vx = cmd.substring(2, comma).toFloat();
    float wz = cmd.substring(comma + 1).toFloat();
    setVelocity(vx, wz);
    return;
  }

  if (cmd.length() == 1) {
    char c = cmd.charAt(0);
    switch (c) {
      case 'i': setVelocity( key_vx,  0.0f);   break;
      case ',': setVelocity(-key_vx,  0.0f);   break;
      case 'j': setVelocity( 0.0f,    key_wz); break;
      case 'l': setVelocity( 0.0f,   -key_wz); break;
      case 'u': setVelocity( key_vx,  key_wz); break;
      case 'o': setVelocity( key_vx, -key_wz); break;
      case 'm': setVelocity(-key_vx,  key_wz); break;
      case '.': setVelocity(-key_vx, -key_wz); break;
      case 'k':
        pid_left.target = 0.0f;  pid_right.target = 0.0f;
        pid_left.reset();         pid_right.reset();
        setMotor(MOT_L_PWM, MOT_L_DIR, 0);
        setMotor(MOT_R_PWM, MOT_R_DIR, 0);
        last_cmd_ms = millis();
        break;
      case '+': key_vx = min(key_vx + 0.05f, MAX_VEL_MS);
                key_wz = min(key_wz + 0.10f, 2.0f);       break;
      case '-': key_vx = max(key_vx - 0.05f, 0.05f);
                key_wz = max(key_wz - 0.10f, 0.10f);      break;
      default: break;
    }
  }
}

// ── Timing ────────────────────────────────────────────────────────────────────
constexpr uint32_t LOOP_INTERVAL_MS = 20;  // 50 Hz
uint32_t last_loop_ms = 0;

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  // Motors
  pinMode(MOT_L_PWM, OUTPUT); pinMode(MOT_L_DIR, OUTPUT);
  pinMode(MOT_R_PWM, OUTPUT); pinMode(MOT_R_DIR, OUTPUT);
  setMotor(MOT_L_PWM, MOT_L_DIR, 0);
  setMotor(MOT_R_PWM, MOT_R_DIR, 0);

  // Encoders
  pinMode(ENC_L_A, INPUT_PULLUP); pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), enc_left_a,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_L_B), enc_left_b,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), enc_right_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_B), enc_right_b, CHANGE);

  last_cmd_ms  = millis();
  last_loop_ms = millis();

  setupSequenceHardware();
}

// ── Main loop ─────────────────────────────────────────────────────────────────
void loop() {
  // ── 1. Read incoming serial ──────────────────────────────────────────────────
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      serialBuf.trim();
      if (serialBuf.length() > 0) parseCommand(serialBuf);
      serialBuf = "";
    } else if (c != '\r') {
      serialBuf += c;
    }
  }

  // ── 2. Run sequence state machine ────────────────────────────────────────────
  runSequence();

  // ── 3. Fixed-rate control + publish ──────────────────────────────────────────
  uint32_t now = millis();
  if (now - last_loop_ms < LOOP_INTERVAL_MS) return;
  float dt = (now - last_loop_ms) * 1e-3f;
  last_loop_ms = now;

  // ── 3. Watchdog ───────────────────────────────────────────────────────────────
  if (now - last_cmd_ms > CMD_TIMEOUT_MS) {
    if (fabsf(pid_left.target) >= VEL_DEADBAND || fabsf(pid_right.target) >= VEL_DEADBAND) {
      startBrake();
    }
    pid_left.target  = 0.0f;
    pid_right.target = 0.0f;
  }

  // ── 4. Encoder snapshot ───────────────────────────────────────────────────────
  noInterrupts();
  int32_t lc = enc_left;
  int32_t rc = enc_right;
  interrupts();

  int32_t dl_counts = lc - enc_left_prev;
  int32_t dr_counts = rc - enc_right_prev;
  enc_left_prev  = lc;
  enc_right_prev = rc;

  float dl = -dl_counts * M_PER_COUNT;  // Left encoder physically inverted
  float dr = -dr_counts * M_PER_COUNT;  // Right encoder physically inverted

  // ── 5. Actual wheel velocities ────────────────────────────────────────────────
  vel_left_actual  = dl / dt;
  vel_right_actual = dr / dt;

  // ── 6. PID → motor output ─────────────────────────────────────────────────────
  int16_t pwm_l = pid_left.compute(vel_left_actual,  dt);
  int16_t pwm_r = pid_right.compute(vel_right_actual, dt);

  if (fabsf(pid_left.target)  < VEL_DEADBAND) {
    pid_left.reset();
    pwm_l = (now < brake_until_ms && brake_dir_l != 0) ? brake_dir_l * BRAKE_PWM : 0;
  }
  if (fabsf(pid_right.target) < VEL_DEADBAND) {
    pid_right.reset();
    pwm_r = (now < brake_until_ms && brake_dir_r != 0) ? brake_dir_r * BRAKE_PWM : 0;
  }

  setMotor(MOT_L_PWM, MOT_L_DIR, pwm_l);
  setMotor(MOT_R_PWM, MOT_R_DIR, pwm_r);

  // ── 7. Odometry integration ───────────────────────────────────────────────────
  float d_center = (dl + dr) * 0.5f;
  float d_theta  = (dr - dl) / WHEELBASE_M;
  odom_th += d_theta;
  while (odom_th >  3.14159265f) odom_th -= 2.0f * 3.14159265f;
  while (odom_th < -3.14159265f) odom_th += 2.0f * 3.14159265f;
  odom_x += d_center * cos(odom_th - d_theta * 0.5f);
  odom_y += d_center * sin(odom_th - d_theta * 0.5f);

  // ── 8. Heartbeat — fires when idle/driving (seqKeepAlive fires during sequence) ─
  if (now - last_hb_ms >= HB_INTERVAL_MS) {
    last_hb_ms = now;
    Serial.println("SEQ:HB");
  }

  // ── 9. Publish odometry ───────────────────────────────────────────────────────
  Serial.print("O:");
  Serial.print(odom_x, 4);          Serial.print(',');
  Serial.print(odom_y, 4);          Serial.print(',');
  Serial.print(odom_th, 4);         Serial.print(',');
  Serial.print(vel_left_actual, 4); Serial.print(',');
  Serial.print(vel_right_actual, 4);
  Serial.print('\n');

}
