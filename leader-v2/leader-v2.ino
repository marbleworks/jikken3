#include "pins.h"
#include "sensors.h"
#include "wheel_control.h"

// ===== デバッグ設定 =====
#ifndef DEBUG_SENSOR_PRINT
#define DEBUG_SENSOR_PRINT 0  // 1: センサ出力を有効化
#endif
#ifndef DEBUG_PWM_PLOTTER
#define DEBUG_PWM_PLOTTER 1   // 1: PWMシリアルプロッター出力を有効化
#endif

// ===== センサパラメータ =====
int THRESHOLD = 500;
int HYST = 50;
float LINE_WHITE = 300.0f;
float LINE_BLACK = 900.0f;
float LINE_EPS = 0.01f;

// ===== モータパラメータ =====
int MAX_PWM = 255;
int MIN_PWM = 0;  // 逆回転でブレーキ効果
int BASE_SPEED = 200;
unsigned long RAMP_UP_MS = 1000;  // ソフトスタート時間（ms）
int LOST_TURN_PWM = 200;           // ライン見失い時の旋回PWM差分
int LOST_SPEED = 200;              // ライン見失い時のベース速度

// ===== PIDパラメータ =====
float Kp = 200.0f;
float Ki = 0.0f;
float Kd = 20.0f;
float PID_I_LIMIT = 100.0f;

struct PIDState {
  float integral = 0.0f;
  float lastError = 0.0f;
  unsigned long lastTime = 0;

  void reset() {
    integral = 0.0f;
    lastError = 0.0f;
    lastTime = 0;
  }
};

float computePidCorrection(PIDState& pid, float error) {
  unsigned long now = millis();
  float dt = 0.0f;
  if (pid.lastTime != 0) {
    dt = (now - pid.lastTime) / 1000.0f;
  }
  pid.lastTime = now;

  float derivative = 0.0f;
  if (dt > 0.0f) {
    pid.integral += error * dt;
    pid.integral = constrain(pid.integral, -PID_I_LIMIT, PID_I_LIMIT);
    derivative = (error - pid.lastError) / dt;
  }
  pid.lastError = error;

  return Kp * error + Ki * pid.integral + Kd * derivative;
}

namespace {
PIDState pid;
unsigned long startTime = 0;
}

void setup() {
  Serial.begin(115200);
  setupWheelPins();

  startTime = millis();
}

void loop() {
  Sense s = readSensors();
  unsigned long now = millis();

  int leftSpeed;
  int rightSpeed;
  float error = 0.0f;
  float correction = 0.0f;

  if (s.allWhite) {
    // 内側のタイヤだけ下げて旋回
    // lastBlackDirState=-1(ラインが左) → 左旋回 → 左を下げる
    // lastBlackDirState>=0(ラインが右or中央) → 右旋回 → 右を下げる
    if (s.lastBlackDirState < 0) {
      // 左旋回: 左を下げる
      leftSpeed = LOST_SPEED - LOST_TURN_PWM;
      rightSpeed = LOST_SPEED;
    } else {
      // 右旋回: 右を下げる
      leftSpeed = LOST_SPEED;
      rightSpeed = LOST_SPEED - LOST_TURN_PWM;
    }
    pid.reset();
  } else {
    error = computeError(s);
    correction = computePidCorrection(pid, error);

    // error正 = 機体が右 → 左旋回 → 左を下げる
    // error負 = 機体が左 → 右旋回 → 右を下げる
    int corr = (int)fabsf(correction);
    if (correction > 0) {
      // 左旋回: 左を下げる
      leftSpeed = BASE_SPEED - corr;
      rightSpeed = BASE_SPEED;
    } else {
      // 右旋回: 右を下げる
      leftSpeed = BASE_SPEED;
      rightSpeed = BASE_SPEED - corr;
    }
  }

  // PWM上限を決定
  int currentMaxPwm = MAX_PWM;

  // ソフトスタート: 経過時間に応じてPWM上限を制限
  unsigned long elapsed = now - startTime;
  if (elapsed < RAMP_UP_MS) {
    currentMaxPwm = (int)((long)MAX_PWM * elapsed / RAMP_UP_MS);
  }

  // ラインロスト時はさらに上限を下げる
  if (s.allWhite && currentMaxPwm > LOST_SPEED) {
    currentMaxPwm = LOST_SPEED;
  }

  leftSpeed = constrain(leftSpeed, MIN_PWM, currentMaxPwm);
  rightSpeed = constrain(rightSpeed, MIN_PWM, currentMaxPwm);

  setWheels(leftSpeed, rightSpeed);

#if DEBUG_SENSOR_PRINT
  // センサデバッグ出力
  for (size_t i = 0; i < SENSOR_COUNT; ++i) {
    Serial.print(F("S"));
    Serial.print(i);
    Serial.print(F(":"));
    Serial.print(s.raw[i]);
    Serial.print(F("\t"));
  }
  Serial.print(F("Err:"));
  Serial.print(error);
  Serial.print(F("\tCorr:"));
  Serial.print(correction);
  Serial.print(F("\tL:"));
  Serial.print(leftSpeed);
  Serial.print(F("\tR:"));
  Serial.println(rightSpeed);
#endif

#if DEBUG_PWM_PLOTTER
  // シリアルプロッター用PWM出力
  Serial.print(F("Left:"));
  Serial.print(leftSpeed);
  Serial.print(F("\tRight:"));
  Serial.print(rightSpeed);
  Serial.print(F("\tMax:"));
  Serial.print(MAX_PWM);
  Serial.print(F("\tMin:"));
  Serial.println(MIN_PWM);
#endif
}
