/*
 * yamancode.cpp — AGV Arduino Mega Firmware
 *
 * Hardware:
 *   Encoders  : E6B2-CWZ6C 600P/R
 *     Left  OutA→D2,  OutB→D3
 *     Right OutA→D18, OutB→D19
 *   Motor Driver: Cytron MDD20A
 *     Left  PWM→D6,  DIR→D52
 *     Right PWM→D7,  DIR→D53
 *   IMU : MPU9250  SCL→D21, SDA→D20
 *
 * Gear train:
 *   Motor gear 9T → Encoder gear 9T (1:1 with motor)
 *   Motor gear 9T → Wheel gear 32T
 *   => counts per wheel rev = 600 × 4 × (32/9) ≈ 8533.33
 *   => wheel circumference  = π × 0.200 m = 0.6283 m
 *   => meters per count     = 0.6283 / 8533.33 ≈ 7.363e-5 m
 *   Wheelbase (L)           = 0.400 m
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
 *  │                                                                          │
 *  │ "I:<ax>,<ay>,<az>,<gx>,<gy>,<gz>\n"   IMU (50 Hz)                      │
 *  │   accelerations in m/s², gyro in rad/s  (float, 3 dp)                  │
 *  └──────────────────────────────────────────────────────────────────────────┘
 *
 *  ROS2 side: run a serial bridge node that converts:
 *    /cmd_vel (Twist) → "V:<vx>,<wz>\n"
 *    "O:..." line     → nav_msgs/Odometry + odom→base_link TF
 *    "I:..." line     → sensor_msgs/Imu
 *
 * Requires library: MPU9250 by hideakitai (install via Library Manager)
 */

#include <Arduino.h>
#include <Wire.h>
#include <MPU9250.h>
#include <math.h>

// ── Pin definitions ──────────────────────────────────────────────────────────
#define ENC_L_A   2
#define ENC_L_B   3
#define ENC_R_A  18
#define ENC_R_B  19

#define MOT_L_PWM  6
#define MOT_L_DIR 52
#define MOT_R_PWM  7
#define MOT_R_DIR 53

// ── Robot geometry ───────────────────────────────────────────────────────────
constexpr float COUNTS_PER_REV  = 600.0f * 4.0f * (32.0f / 9.0f); // ≈ 8533.33
constexpr float WHEEL_DIAM_M    = 0.200f;
constexpr float WHEEL_CIRCUM_M  = 3.14159265f * WHEEL_DIAM_M;      // 0.6283 m
constexpr float M_PER_COUNT     = WHEEL_CIRCUM_M / COUNTS_PER_REV; // ~7.363e-5 m
constexpr float WHEELBASE_M     = 0.445f;  // from SolidWorks CAD (joint y: -0.22249 + 0.22251)

// ── PID gains (tune on real robot) ───────────────────────────────────────────
constexpr float PID_KP          = 150.0f;
constexpr float PID_KI          = 80.0f;
constexpr float PID_KD          = 3.0f;
constexpr float PID_INTEGRAL_MAX = 80.0f;   // anti-windup clamp
constexpr float MAX_VEL_MS      = 0.5f;     // maximum wheel velocity (m/s)

// ── Encoder state (ISR-updated) ──────────────────────────────────────────────
volatile int32_t enc_left  = 0;
volatile int32_t enc_right = 0;

void enc_left_a()  { digitalRead(ENC_L_A) == digitalRead(ENC_L_B) ? enc_left--  : enc_left++;  }
void enc_left_b()  { digitalRead(ENC_L_A) == digitalRead(ENC_L_B) ? enc_left++  : enc_left--;  }
void enc_right_a() { digitalRead(ENC_R_A) == digitalRead(ENC_R_B) ? enc_right++ : enc_right--; }
void enc_right_b() { digitalRead(ENC_R_A) == digitalRead(ENC_R_B) ? enc_right-- : enc_right++; }

// ── IMU ──────────────────────────────────────────────────────────────────────
MPU9250 imu;
bool imu_ok = false;

// ── Motor driver ─────────────────────────────────────────────────────────────
void setMotor(uint8_t pwmPin, uint8_t dirPin, int16_t speed) {
  speed = constrain(speed, -255, 255);
  digitalWrite(dirPin, speed >= 0 ? HIGH : LOW);
  analogWrite(pwmPin, abs(speed));
}

// ── PID state per wheel ───────────────────────────────────────────────────────
struct PID {
  float target   = 0.0f;   // desired velocity (m/s)
  float integral = 0.0f;
  float prev_err = 0.0f;

  int16_t compute(float actual, float dt) {
    if (dt <= 0.0f) return 0;
    float err  = target - actual;
    integral  += err * dt;
    integral   = constrain(integral, -PID_INTEGRAL_MAX, PID_INTEGRAL_MAX);
    float deriv = (err - prev_err) / dt;
    prev_err   = err;
    float out  = PID_KP * err + PID_KI * integral + PID_KD * deriv;
    return (int16_t)constrain(out, -255.0f, 255.0f);
  }

  void reset() { integral = 0.0f; prev_err = 0.0f; }
};

PID pid_left, pid_right;

// ── Odometry state ────────────────────────────────────────────────────────────
float odom_x   = 0.0f;
float odom_y   = 0.0f;
float odom_th  = 0.0f;
float vel_left_actual  = 0.0f;
float vel_right_actual = 0.0f;
int32_t enc_left_prev  = 0;
int32_t enc_right_prev = 0;

// ── Watchdog — stop if no command for 500 ms ─────────────────────────────────
uint32_t last_cmd_ms = 0;
constexpr uint32_t CMD_TIMEOUT_MS = 500;

// ── Keyboard teleop speed step ───────────────────────────────────────────────
float key_vx = 0.20f;   // m/s linear step
float key_wz = 0.40f;   // rad/s angular step

void setVelocity(float vx, float wz) {
  vx = constrain(vx, -MAX_VEL_MS, MAX_VEL_MS);
  // differential drive: v_L = vx - wz*L/2,  v_R = vx + wz*L/2
  pid_left.target  = constrain(vx - wz * (WHEELBASE_M / 2.0f), -MAX_VEL_MS, MAX_VEL_MS);
  pid_right.target = constrain(vx + wz * (WHEELBASE_M / 2.0f), -MAX_VEL_MS, MAX_VEL_MS);
  last_cmd_ms = millis();
}

// ── Serial parsing ────────────────────────────────────────────────────────────
String serialBuf = "";

void parseCommand(const String &cmd) {
  // "V:<vx>,<wz>" — velocity command from ROS2 bridge
  if (cmd.startsWith("V:")) {
    int comma = cmd.indexOf(',', 2);
    if (comma < 0) return;
    float vx = cmd.substring(2, comma).toFloat();
    float wz = cmd.substring(comma + 1).toFloat();
    setVelocity(vx, wz);
    return;
  }

  // Single-character keyboard teleop (teleop_twist_keyboard layout)
  if (cmd.length() == 1) {
    char c = cmd.charAt(0);
    switch (c) {
      case 'i': setVelocity( key_vx,     0.0f);    break;  // forward
      case ',': setVelocity(-key_vx,     0.0f);    break;  // backward
      case 'j': setVelocity( 0.0f,       key_wz);  break;  // rotate left
      case 'l': setVelocity( 0.0f,      -key_wz);  break;  // rotate right
      case 'u': setVelocity( key_vx,     key_wz);  break;  // fwd + left
      case 'o': setVelocity( key_vx,    -key_wz);  break;  // fwd + right
      case 'm': setVelocity(-key_vx,     key_wz);  break;  // bwd + left
      case '.': setVelocity(-key_vx,    -key_wz);  break;  // bwd + right
      case 'k':                                              // stop
        pid_left.target = 0.0f;
        pid_right.target = 0.0f;
        pid_left.reset();
        pid_right.reset();
        setMotor(MOT_L_PWM, MOT_L_DIR, 0);
        setMotor(MOT_R_PWM, MOT_R_DIR, 0);
        last_cmd_ms = millis();
        break;
      case '+': key_vx = min(key_vx + 0.05f, MAX_VEL_MS);
                key_wz = min(key_wz + 0.10f, 2.0f);        break;
      case '-': key_vx = max(key_vx - 0.05f, 0.05f);
                key_wz = max(key_wz - 0.10f, 0.10f);       break;
      default: break;
    }
  }
}

// ── Timing ────────────────────────────────────────────────────────────────────
constexpr uint32_t LOOP_INTERVAL_MS = 20;   // 50 Hz
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

  // IMU
  Wire.begin();
  imu_ok = imu.setup(0x68);
  if (!imu_ok) Serial.println("ERR:IMU_NOT_FOUND");

  last_cmd_ms   = millis();
  last_loop_ms  = millis();
}

// ── Main loop ─────────────────────────────────────────────────────────────────
void loop() {
  // ── 1. Read incoming serial ─────────────────────────────────────────────────
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

  // ── 2. Fixed-rate control + publish ─────────────────────────────────────────
  uint32_t now = millis();
  if (now - last_loop_ms < LOOP_INTERVAL_MS) return;
  float dt = (now - last_loop_ms) * 1e-3f;
  last_loop_ms = now;

  // ── 3. Watchdog — stop motors if no recent command ──────────────────────────
  if (now - last_cmd_ms > CMD_TIMEOUT_MS) {
    pid_left.target  = 0.0f;
    pid_right.target = 0.0f;
  }

  // ── 4. Encoder snapshot ──────────────────────────────────────────────────────
  noInterrupts();
  int32_t lc = enc_left;
  int32_t rc = enc_right;
  interrupts();

  int32_t dl_counts = lc - enc_left_prev;
  int32_t dr_counts = rc - enc_right_prev;
  enc_left_prev  = lc;
  enc_right_prev = rc;

  float dl = dl_counts * M_PER_COUNT;  // metres travelled by left wheel
  float dr = dr_counts * M_PER_COUNT;  // metres travelled by right wheel

  // ── 5. Actual wheel velocities ───────────────────────────────────────────────
  vel_left_actual  = dl / dt;
  vel_right_actual = dr / dt;

  // ── 6. PID → motor output ────────────────────────────────────────────────────
  int16_t pwm_l = pid_left.compute(vel_left_actual,  dt);
  int16_t pwm_r = pid_right.compute(vel_right_actual, dt);

  // If target is zero and robot is near stopped, cut PWM completely
  if (pid_left.target  == 0.0f && abs(vel_left_actual)  < 0.01f) { pwm_l = 0; pid_left.reset();  }
  if (pid_right.target == 0.0f && abs(vel_right_actual) < 0.01f) { pwm_r = 0; pid_right.reset(); }

  setMotor(MOT_L_PWM, MOT_L_DIR, pwm_l);
  setMotor(MOT_R_PWM, MOT_R_DIR, pwm_r);

  // ── 7. Odometry integration ──────────────────────────────────────────────────
  float d_center = (dl + dr) * 0.5f;
  float d_theta  = (dr - dl) / WHEELBASE_M;
  odom_th += d_theta;
  // wrap to [-π, π]
  while (odom_th >  3.14159265f) odom_th -= 2.0f * 3.14159265f;
  while (odom_th < -3.14159265f) odom_th += 2.0f * 3.14159265f;
  odom_x  += d_center * cos(odom_th - d_theta * 0.5f);
  odom_y  += d_center * sin(odom_th - d_theta * 0.5f);

  // ── 8. Publish odometry ──────────────────────────────────────────────────────
  // "O:<x>,<y>,<th>,<vl>,<vr>"
  Serial.print("O:");
  Serial.print(odom_x, 4);          Serial.print(',');
  Serial.print(odom_y, 4);          Serial.print(',');
  Serial.print(odom_th, 4);         Serial.print(',');
  Serial.print(vel_left_actual, 4); Serial.print(',');
  Serial.print(vel_right_actual, 4);
  Serial.print('\n');

  // ── 9. Publish IMU ───────────────────────────────────────────────────────────
  if (imu_ok) {
    imu.update();
    // "I:<ax>,<ay>,<az>,<gx>,<gy>,<gz>"
    // getAcc* returns g → convert to m/s²; getGyro* returns deg/s → rad/s
    Serial.print("I:");
    Serial.print(imu.getAccX()  * 9.80665f, 3); Serial.print(',');
    Serial.print(imu.getAccY()  * 9.80665f, 3); Serial.print(',');
    Serial.print(imu.getAccZ()  * 9.80665f, 3); Serial.print(',');
    Serial.print(imu.getGyroX() * 0.01745f, 3); Serial.print(',');
    Serial.print(imu.getGyroY() * 0.01745f, 3); Serial.print(',');
    Serial.print(imu.getGyroZ() * 0.01745f, 3);
    Serial.print('\n');
  }
}
