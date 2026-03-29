/*
 * motor_test_sketch.ino — Forward/Backward confirmed test
 * Combo B confirmed: L=HIGH, R=HIGH = forward
 * Forward 2s → stop 1s → backward 2s → stop
 */

#include <Arduino.h>

#define ENC_L_A   2
#define ENC_L_B   3
#define ENC_R_A  18
#define ENC_R_B  19

#define MOT_L_PWM  6
#define MOT_L_DIR 22
#define MOT_R_PWM  7
#define MOT_R_DIR 23

#define TEST_PWM  50

volatile int32_t enc_left  = 0;
volatile int32_t enc_right = 0;

void enc_left_a()  { digitalRead(ENC_L_A) == digitalRead(ENC_L_B) ? enc_left--  : enc_left++;  }
void enc_left_b()  { digitalRead(ENC_L_A) == digitalRead(ENC_L_B) ? enc_left++  : enc_left--;  }
void enc_right_a() { digitalRead(ENC_R_A) == digitalRead(ENC_R_B) ? enc_right++ : enc_right--; }
void enc_right_b() { digitalRead(ENC_R_A) == digitalRead(ENC_R_B) ? enc_right-- : enc_right++; }

void stopAll() {
  analogWrite(MOT_L_PWM, 0);
  analogWrite(MOT_R_PWM, 0);
}

void driveStep(const char* label, bool forward) {
  noInterrupts();
  int32_t l0 = enc_left, r0 = enc_right;
  interrupts();

  digitalWrite(MOT_L_DIR, forward ? HIGH : LOW);  // Both same: HIGH=fwd, LOW=bwd
  digitalWrite(MOT_R_DIR, forward ? HIGH : LOW);
  analogWrite(MOT_L_PWM, TEST_PWM);
  analogWrite(MOT_R_PWM, TEST_PWM);

  delay(2000);
  stopAll();

  noInterrupts();
  int32_t dl = enc_left  - l0;
  int32_t dr = enc_right - r0;
  interrupts();

  Serial.print("[ ");
  Serial.print(label);
  Serial.print(" ]  enc_left=");
  Serial.print(dl);
  Serial.print("  enc_right=");
  Serial.println(dr);

  delay(1000);
}

void setup() {
  Serial.begin(115200);

  pinMode(MOT_L_PWM, OUTPUT); pinMode(MOT_L_DIR, OUTPUT);
  pinMode(MOT_R_PWM, OUTPUT); pinMode(MOT_R_DIR, OUTPUT);
  stopAll();

  pinMode(ENC_L_A, INPUT_PULLUP); pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), enc_left_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_L_B), enc_left_b, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), enc_right_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_B), enc_right_b, CHANGE);

  Serial.println("=== Forward/Backward Test ===");
  Serial.println("Starting in 3 seconds...");
  delay(3000);
}

bool done = false;

void loop() {
  if (done) return;
  done = true;

  Serial.println("Step 1: FORWARD");
  driveStep("FORWARD", true);

  Serial.println("Step 2: BACKWARD");
  driveStep("BACKWARD", false);

  Serial.println("=== DONE ===");
}
