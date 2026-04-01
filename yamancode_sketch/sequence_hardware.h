/**
 * sequence_hardware.h — Real hardware sequence for AGV SCARA arm + rail system
 *
 * Include this file in yamancode_sketch.ino AFTER #include <Servo.h>
 *
 * Based on arduinonoraspi.txt testloop2 (full shelf height = 13800 steps).
 *
 * ── Hardware controlled here ──────────────────────────────────────────────────
 *   Vertical rail  : two steppers DIR1/PUL1 (D44/45) + DIR2/PUL2 (D46/47) together
 *                    LIMIT_VERTICAL (D28) stops downward travel at bottom
 *   Horizontal rail: stepper DIR3/PUL3 (D48/49)
 *   SCARA J1       : stepper DIR_J1/PUL_J1 (D52/53), limit LIMIT_J1 (D29)
 *   SCARA J2       : stepper DIR_J2/PUL_J2 (D50/51), limit LIMIT_J2 (D27)
 *   EE rack        : continuous servo D4 (CW=retract up, CCW=extend down)
 *                    EE_LIMIT (D26) stops retract when fully up
 *   Grip servo     : servo D5
 *   Vacuum pump    : D13 (LOW=ON, HIGH=OFF)
 *   LEDs           : WHITE=D8, GREEN=D9, BLUE=D10, YELLOW=D11
 *
 * NOTE: No pressure sensor (HX711 broken — skipped).
 *       No camera (handled by RPi later).
 *       EE extends down for SEQ_EE_DOWN_MS and assumes grab success.
 *
 * ── DC motor pin sharing ──────────────────────────────────────────────────────
 *   D6/D22 (left) and D7/D23 (right) are the AGV drive motors in yamancode.
 *   seqAGVNudge() uses setMotor() from yamancode directly for shelf alignment.
 *   seqKeepAlive() resets the watchdog so it doesn't fight these moves.
 */

#pragma once

// ─── Pin definitions ──────────────────────────────────────────────────────────
#define LED_WHITE   8
#define LED_GREEN   9
#define LED_BLUE   10
#define LED_YELLOW 11

#define VAC_PUMP 13   // LOW = ON, HIGH = OFF

#define EE_LIMIT        26
#define LIMIT_J2        27
#define LIMIT_VERTICAL  28
#define LIMIT_J1        29

#define EE_SERVO_PIN   4
#define GRIP_SERVO_PIN 5

// Vertical rail
#define DIR1 44
#define PUL1 45
#define DIR2 46
#define PUL2 47

// Horizontal rail
#define DIR3 48
#define PUL3 49

// SCARA joints
#define DIR_J1 52
#define PUL_J1 53
#define DIR_J2 50
#define PUL_J2 51

// ─── Tunable constants ────────────────────────────────────────────────────────
static const int SEQ_LINEAR_SPEED    = 1200;   // µs/step — vertical + horizontal rails
static const int SEQ_JOINT1_SPEED    = 2000;   // µs/step — SCARA J1
static const int SEQ_JOINT2_SPEED    = 2500;   // µs/step — SCARA J2
static const int SEQ_LIFT_HEIGHT     = 13800;  // steps to full shelf height (testloop2)
static const int SEQ_HORIZ_STEPS     = 1500;   // horizontal extend/retract steps
static const int SEQ_DC_ALIGN_PWM    = 30;     // AGV nudge PWM during sequence

// EE servo timings (ms)
static const uint32_t SEQ_EE_DOWN_PICK_MS  = 3050;   // descend to pick medicine pack
static const uint32_t SEQ_EE_DOWN_PLACE_MS = 3000;   // descend to drop into container
static const uint32_t SEQ_EE_UP_MS         = 10000;  // retract up (limit switch stops early)
static const uint32_t SEQ_EE_HOME_MS       = 3500;   // retract at sequence start (reduced from 12000 — limit switch stops early anyway)

// Servo positions
static const int SERVO_CW   = 0;
static const int SERVO_CCW  = 180;
static const int SERVO_STOP = 90;

// Grip servo positions
static const int GRIP_SAFE    = 125;  // open/safe — startup and transit
static const int GRIP_READY   = 90;   // ready to grip box
static const int GRIP_GRIPPED = 70;   // gripping pack
static const int GRIP_RELEASE = 120;  // release box back onto shelf

// ─── Global objects ───────────────────────────────────────────────────────────
Servo eeServo;
Servo gripServo;

static unsigned long _seq_lastBlink  = 0;
static unsigned long _seq_lastHB     = 0;
static bool          _seq_blinkState = false;

// ─── seqKeepAlive ─────────────────────────────────────────────────────────────
// Call inside every blocking loop.
// Resets yamancode watchdog (last_cmd_ms) + blinks LEDs + fires SEQ:HB every 3s.
// Without this, the main loop never runs during the ~30-60s blocking sequence
// so the RPi serial_bridge would see no heartbeat and assume the Arduino died.
void seqKeepAlive() {
  last_cmd_ms = millis();
  unsigned long now = millis();
  if (now - _seq_lastBlink >= 500) {
    _seq_lastBlink  = now;
    _seq_blinkState = !_seq_blinkState;
    digitalWrite(LED_BLUE,   _seq_blinkState ? LOW  : HIGH);
    digitalWrite(LED_YELLOW, _seq_blinkState ? HIGH : LOW);
    digitalWrite(LED_WHITE,  HIGH);
  }
  if (now - _seq_lastHB >= 3000) {
    _seq_lastHB = now;
    Serial.println("SEQ:HB");
  }
}

// Blocking delay with watchdog keepalive
void seqDelay(uint32_t ms) {
  uint32_t start = millis();
  while (millis() - start < ms) seqKeepAlive();
}

// ─── Actuator functions ───────────────────────────────────────────────────────

void seqMoveLinear(int steps, bool dir) {
  digitalWrite(DIR1, dir); digitalWrite(DIR2, dir);
  for (int i = 0; i < steps; i++) {
    if (dir == LOW && digitalRead(LIMIT_VERTICAL) == HIGH) break;
    digitalWrite(PUL1, HIGH); digitalWrite(PUL2, HIGH);
    delayMicroseconds(2);
    digitalWrite(PUL1, LOW);  digitalWrite(PUL2, LOW);
    delayMicroseconds(SEQ_LINEAR_SPEED);
    seqKeepAlive();
  }
}

void seqHomeLinear() {
  digitalWrite(DIR1, LOW); digitalWrite(DIR2, LOW);
  while (digitalRead(LIMIT_VERTICAL) == LOW) {
    digitalWrite(PUL1, HIGH); digitalWrite(PUL2, HIGH);
    delayMicroseconds(2);
    digitalWrite(PUL1, LOW);  digitalWrite(PUL2, LOW);
    delayMicroseconds(SEQ_LINEAR_SPEED);
    seqKeepAlive();
  }
}

void seqMoveHorizontal(int steps, bool dir) {
  digitalWrite(DIR3, dir);
  for (int i = 0; i < steps; i++) {
    digitalWrite(PUL3, HIGH); delayMicroseconds(2);
    digitalWrite(PUL3, LOW);  delayMicroseconds(SEQ_LINEAR_SPEED);
    seqKeepAlive();
  }
}

void seqMoveJointsSync(long stepsJ1, bool dirJ1, long stepsJ2, bool dirJ2) {
  digitalWrite(DIR_J1, dirJ1);
  digitalWrite(DIR_J2, dirJ2);
  long c1 = 0, c2 = 0;
  int spd = (SEQ_JOINT1_SPEED > SEQ_JOINT2_SPEED) ? SEQ_JOINT1_SPEED : SEQ_JOINT2_SPEED;
  while (c1 < stepsJ1 || c2 < stepsJ2) {
    if (dirJ1 == HIGH && digitalRead(LIMIT_J1) == HIGH) c1 = stepsJ1;
    if (dirJ2 == HIGH && digitalRead(LIMIT_J2) == HIGH) c2 = stepsJ2;
    if (c1 < stepsJ1) { digitalWrite(PUL_J1, HIGH); }
    if (c2 < stepsJ2) { digitalWrite(PUL_J2, HIGH); }
    delayMicroseconds(2);
    digitalWrite(PUL_J1, LOW); digitalWrite(PUL_J2, LOW);
    c1++; c2++;
    delayMicroseconds(spd);
    seqKeepAlive();
  }
}

void seqScaraGo1() {
  seqMoveJointsSync(2500, LOW, 300, LOW);
  seqMoveJointsSync(500,  LOW,  50, LOW);
}

void seqScaraHome() {
  seqMoveJointsSync(10000, HIGH, 10000, HIGH);
}

void seqRunEEServo(int sVal, uint32_t d) {
  eeServo.write(sVal);
  uint32_t start = millis();
  while (millis() - start < d) {
    if (sVal == SERVO_CW && digitalRead(EE_LIMIT) == HIGH) break;
    seqKeepAlive();
  }
  eeServo.write(SERVO_STOP);
}

// Small AGV alignment nudge using yamancode's setMotor()
// pwm > 0 = forward, pwm < 0 = backward
void seqAGVNudge(int pwm, uint32_t ms) {
  setMotor(MOT_L_PWM, MOT_L_DIR, pwm);
  setMotor(MOT_R_PWM, MOT_R_DIR, pwm);
  seqDelay(ms);
  setMotor(MOT_L_PWM, MOT_L_DIR, 0);
  setMotor(MOT_R_PWM, MOT_R_DIR, 0);
}

void seqResetLEDs() {
  digitalWrite(LED_BLUE,   HIGH);
  digitalWrite(LED_YELLOW, HIGH);
  digitalWrite(LED_GREEN,  LOW);
  seqDelay(500);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_WHITE, LOW);
}

// ─── setupSequenceHardware() — call from yamancode setup() ───────────────────
void setupSequenceHardware() {
  pinMode(LED_WHITE,  OUTPUT); pinMode(LED_GREEN,  OUTPUT);
  pinMode(LED_BLUE,   OUTPUT); pinMode(LED_YELLOW, OUTPUT);
  digitalWrite(LED_WHITE, LOW);  digitalWrite(LED_GREEN,  HIGH);
  digitalWrite(LED_BLUE,  HIGH); digitalWrite(LED_YELLOW, HIGH);

  pinMode(VAC_PUMP, OUTPUT);
  digitalWrite(VAC_PUMP, HIGH);  // pump OFF at startup

  pinMode(EE_LIMIT,       INPUT_PULLUP);
  pinMode(LIMIT_J1,       INPUT_PULLUP);
  pinMode(LIMIT_J2,       INPUT_PULLUP);
  pinMode(LIMIT_VERTICAL, INPUT_PULLUP);

  pinMode(DIR1, OUTPUT); pinMode(PUL1, OUTPUT);
  pinMode(DIR2, OUTPUT); pinMode(PUL2, OUTPUT);
  pinMode(DIR3, OUTPUT); pinMode(PUL3, OUTPUT);
  pinMode(DIR_J1, OUTPUT); pinMode(PUL_J1, OUTPUT);
  pinMode(DIR_J2, OUTPUT); pinMode(PUL_J2, OUTPUT);

  eeServo.attach(EE_SERVO_PIN);
  eeServo.write(SERVO_STOP);
  gripServo.attach(GRIP_SERVO_PIN);
  gripServo.write(GRIP_SAFE);

  // EE homing moved to runRealSequence() — avoids 12s current draw on every boot

  seqResetLEDs();
  Serial.println("SEQ:HW:READY");
}

// ─── runRealSequence() — called when SEQ:START received ──────────────────────
// Blocking (~30-60s). AGV must already be stopped at shelf position.
// Sends SEQ:STEP:n messages throughout. Ends with SEQ:DONE.
void runRealSequence() {
  // Stop AGV PID — sequence controls DC motors directly for nudges
  pid_left.target  = 0.0f; pid_left.reset();
  pid_right.target = 0.0f; pid_right.reset();

  // Home EE servo before starting (3.5s — limit switch stops it early if already up)
  Serial.println("SEQ:STEP:2:EE servo homing");
  seqRunEEServo(SERVO_CW, SEQ_EE_HOME_MS);

  // ── PART 1: Retrieve medicine box from shelf ──────────────────────────────
  Serial.println("SEQ:STEP:3:Vertical rail UP");
  seqMoveLinear(SEQ_LIFT_HEIGHT, HIGH);

  Serial.println("SEQ:STEP:4:Horizontal rail EXTEND");
  seqMoveHorizontal(SEQ_HORIZ_STEPS, HIGH);

  Serial.println("SEQ:STEP:5:Gripper GRIP box");
  gripServo.write(GRIP_READY); seqDelay(500);

  Serial.println("SEQ:STEP:6:Horizontal rail RETRACT");
  seqMoveHorizontal(SEQ_HORIZ_STEPS, LOW);

  // Backward nudge — clears box from shelf edge
  seqAGVNudge(-SEQ_DC_ALIGN_PWM, 1000);

  Serial.println("SEQ:STEP:7:Vertical rail DOWN");
  seqHomeLinear();

  // ── PART 2: Pick medicine pack ────────────────────────────────────────────
  Serial.println("SEQ:STEP:8:SCARA move to box");
  seqScaraGo1();

  Serial.println("SEQ:STEP:9:Vacuum pump OPEN");
  gripServo.write(GRIP_GRIPPED); seqDelay(500);
  digitalWrite(VAC_PUMP, LOW);  // pump ON

  Serial.println("SEQ:STEP:11:EE rack extend DOWN");
  seqRunEEServo(SERVO_CCW, SEQ_EE_DOWN_PICK_MS);  // down for 3050ms
  seqDelay(1000);  // dwell 1s with vacuum engaged — allows pack to seat before retract

  Serial.println("SEQ:STEP:14:EE rack retract UP");
  seqRunEEServo(SERVO_CW, SEQ_EE_UP_MS);
  gripServo.write(GRIP_READY); seqDelay(500);

  // ── PART 2b: Transfer pack to container ──────────────────────────────────
  Serial.println("SEQ:STEP:15:SCARA move to container");
  seqMoveJointsSync(0,    LOW,  200, HIGH);
  seqMoveJointsSync(1500, HIGH,   0, LOW);
  seqMoveJointsSync(0,    LOW,  100, LOW);

  Serial.println("SEQ:STEP:16:EE rack extend DOWN to container");
  seqRunEEServo(SERVO_CCW, SEQ_EE_DOWN_PLACE_MS);

  Serial.println("SEQ:STEP:17:Vacuum pump CLOSE — pack in container");
  digitalWrite(VAC_PUMP, HIGH);  // pump OFF → pack released
  seqDelay(3000);

  seqRunEEServo(SERVO_CW, SEQ_EE_UP_MS);

  Serial.println("SEQ:STEP:18:SCARA retract to safe position");
  seqScaraHome();

  // ── PART 3: Return medicine box to shelf ──────────────────────────────────
  // Forward nudge — re-aligns AGV under shelf for box return
  seqAGVNudge(SEQ_DC_ALIGN_PWM, 900);

  Serial.println("SEQ:STEP:19:Vertical rail UP");
  seqMoveLinear(SEQ_LIFT_HEIGHT, HIGH);

  Serial.println("SEQ:STEP:20:Horizontal rail EXTEND");
  seqMoveHorizontal(SEQ_HORIZ_STEPS, HIGH);

  Serial.println("SEQ:STEP:21:Gripper RELEASE — box placed on shelf");
  gripServo.write(GRIP_RELEASE); seqDelay(500);

  Serial.println("SEQ:STEP:22:Horizontal rail RETRACT");
  seqMoveHorizontal(SEQ_HORIZ_STEPS, LOW);

  Serial.println("SEQ:STEP:23:Vertical rail DOWN");
  seqHomeLinear();

  seqResetLEDs();
  Serial.println("SEQ:DONE");
}
