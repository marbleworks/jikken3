#include "pins.h"
#include "sensors.h"
#include "wheel_control.h"

// ===== センサパラメータ =====
int THRESHOLD = 500;
int HYST = 50;
float LINE_WHITE = 100.0f;
float LINE_BLACK = 900.0f;
float LINE_EPS = 0.01f;

// ===== モータパラメータ =====
int MAX_PWM = 255;
int MIN_PWM = 0;
int BASE_SPEED = 255;

// ===== PIDパラメータ =====
float Kp = 100.0f;
float Ki = 0.0f;
float Kd = 10.0f;

// ===== 補正係数 =====
float CORRECTION_LEFT = 1.0f;   // 左輪への補正適用率
float CORRECTION_RIGHT = 1.0f;  // 右輪への補正適用率

namespace {
float integral = 0.0f;
float lastError = 0.0f;
unsigned long lastTime = 0;
}

void setup() {
  Serial.begin(115200);
  setupWheelPins();
  lastTime = millis();
}

void loop() {
  Sense s = readSensors();

  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0f;
  lastTime = now;
  if (dt <= 0.0f) dt = 0.001f;

  float error;
  if (s.allWhite) {
    // ラインロスト: 最後に見た方向に旋回
    // lastBlackDirState=-1(ラインが左) → 機体が右 → error正
    error = (s.lastBlackDirState < 0) ? 1.0f : -1.0f;
    integral = 0.0f;
  } else {
    error = computeError(s);
    integral += error * dt;
  }

  float derivative = (error - lastError) / dt;
  lastError = error;

  float correction = Kp * error + Ki * integral + Kd * derivative;

  // error正 = 機体が右 → 左旋回で戻す
  int leftSpeed = BASE_SPEED - (int)(correction * CORRECTION_LEFT);
  int rightSpeed = BASE_SPEED + (int)(correction * CORRECTION_RIGHT);

  leftSpeed = constrain(leftSpeed, -MAX_PWM, MAX_PWM);
  rightSpeed = constrain(rightSpeed, -MAX_PWM, MAX_PWM);

  setWheels(leftSpeed, rightSpeed);
}
