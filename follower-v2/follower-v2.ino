#include <Arduino.h>
#include <math.h>

#include "distance_sensor.h"
#include "obstacle_sensor.h"
#include "wheel_control.h"
#include "lcd_display.h"
#include "pins.h"

// Serial Plotter出力を有効化する場合は 1 に設定する。
#ifndef SERIAL_PLOTTER_PRINT
#define SERIAL_PLOTTER_PRINT 1
#endif

// ------------------ 制御パラメータ ------------------
const int BASE_PWM = 255;           // 固定の前進速度
const int TURN_PWM = 160;           // 旋回時のPWM差分
const float LOST_THRESHOLD_CM = 30.0f;  // 見失い閾値
const float MAX_DISTANCE_CM = 40.0f;

const uint8_t MIN_PWM = 0;
const uint8_t MAX_PWM = 255;
const uint8_t DEAD_PWM = 30;

const unsigned long SONAR_INTERVAL_MS = 90;
const unsigned long LCD_UPDATE_MS     = 200;  // LCD更新間隔（ドット応答速度に合わせる）
const unsigned long SOFT_START_MS     = 1000; // ソフトスタート時間（0%→100%）

// ------------------ PID制御パラメータ ------------------
float KP = 0.4f;             // 比例ゲイン（距離差 → 補正量）
float KI = 0.0f;             // 積分ゲイン
float KD = 0.03f;            // 微分ゲイン
float PID_I_LIMIT = 50.0f;   // 積分項の上限

// ------------------ モジュール初期化 ------------------
WheelControl wheelController({PIN_LEFT_IN1, PIN_LEFT_IN2, PIN_LEFT_PWM,
                              PIN_RIGHT_IN1, PIN_RIGHT_IN2, PIN_RIGHT_PWM,
                              MIN_PWM, MAX_PWM, DEAD_PWM});
DistanceSensor leftDistanceSensor(PIN_TRIG_LEFT, PIN_ECHO_LEFT, MAX_DISTANCE_CM);
DistanceSensor rightDistanceSensor(PIN_TRIG_RIGHT, PIN_ECHO_RIGHT, MAX_DISTANCE_CM);
ObstacleSensor obstacleSensor(PIN_IR_OBST, true);
LCDDisplay lcdDisplay(PIN_LCD_RS, PIN_LCD_E, PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7);

// ------------------ 状態変数 ------------------
unsigned long lastPingTime   = 0;
unsigned long lastLcdTime    = 0;
unsigned long softStartTime  = 0;  // ソフトスタート開始時刻
int          lastDirection   = 0;  // 最後に検知した方向（内部保持用）

// ------------------ センサ読み取り構造体 ------------------
struct Sense {
  float left;         // 左距離（NAN可能）
  float right;        // 右距離（NAN可能）
  bool leftDetected;  // 左センサで検出したか
  bool rightDetected; // 右センサで検出したか
  bool bothDetected;  // 両方検出
  bool bothLost;      // 両方見失い
  bool obstacle;      // 障害物検知
  int direction;      // 右検出+1, 左検出-1, 両方/なし=0
  int lastDirection;  // 最後に検知した方向（リカバリ用）
};

Sense readSensors()
{
  Sense s;
  s.left = leftDistanceSensor.readDistanceCm();
  s.right = rightDistanceSensor.readDistanceCm();
  s.leftDetected = (s.left < LOST_THRESHOLD_CM);
  s.rightDetected = (s.right < LOST_THRESHOLD_CM);
  s.bothDetected = (s.leftDetected && s.rightDetected);
  s.bothLost = (!s.leftDetected && !s.rightDetected);
  s.obstacle = obstacleSensor.detected();
  s.direction = (s.rightDetected ? 1 : 0) - (s.leftDetected ? 1 : 0);

  if (s.leftDetected || s.rightDetected)
  {
    lastDirection = s.direction;
  }
  s.lastDirection = lastDirection;

  return s;
}

// ------------------ PID制御 ------------------
struct PIDState {
  float integral = 0.0f;
  float lastError = 0.0f;
  unsigned long lastTimeMs = 0;

  void reset() {
    integral = 0.0f;
    lastError = 0.0f;
    lastTimeMs = 0;
  }
} pid;

// 補正量を計算（正=右に曲がる、負=左に曲がる）
int computePidCorrection(float error)
{
  unsigned long now = millis();
  float dt = (pid.lastTimeMs != 0) ? (now - pid.lastTimeMs) / 1000.0f : 0.0f;
  pid.lastTimeMs = now;

  float derivative = 0.0f;
  if (dt > 0.0f) {
    pid.integral = constrain(pid.integral + error * dt, -PID_I_LIMIT, PID_I_LIMIT);
    derivative = (error - pid.lastError) / dt;
  }
  pid.lastError = error;

  return (int)(KP * error + KI * pid.integral + KD * derivative);
}

void setup()
{
  leftDistanceSensor.begin();
  rightDistanceSensor.begin();
  obstacleSensor.begin();
  wheelController.begin();
  lcdDisplay.begin();

  Serial.begin(115200);
  Serial.println(F("Follower-v2 ready"));
}

Sense lastSense;  // 最新のセンサ読み取り結果

void loop()
{
  unsigned long now = millis();

  // センサ読み取り
  if (now - lastPingTime >= SONAR_INTERVAL_MS)
  {
    lastSense = readSensors();
    lastPingTime = now;
  }

  // LCD表示更新
  if (now - lastLcdTime >= LCD_UPDATE_MS)
  {
    lcdDisplay.showDistances(lastSense.left, lastSense.right, lastSense.lastDirection);
    lastLcdTime = now;
  }

  // 補正値を計算
  int correction = 0;
  if (lastSense.bothDetected)
  {
    correction = computePidCorrection(lastSense.left - lastSense.right);
  }
  else
  {
    pid.reset();
    int dir = lastSense.bothLost ? lastSense.lastDirection : lastSense.direction;
    correction = dir * TURN_PWM;
  }

  // モーター制御（ソフトスタート付き）
  int leftPwm = 0;
  int rightPwm = 0;

  if (lastSense.obstacle)
  {
    softStartTime = 0;  // リセット
    wheelController.stop();
  }
  else
  {
    // ソフトスタート開始時刻を記録
    if (softStartTime == 0)
    {
      softStartTime = now;
    }

    // 経過時間からスケールを計算（0.0〜1.0）
    unsigned long elapsed = now - softStartTime;
    float scale = (elapsed >= SOFT_START_MS) ? 1.0f : (float)elapsed / SOFT_START_MS;

    int targetLeftPwm = constrain(BASE_PWM + correction, MIN_PWM, MAX_PWM);
    int targetRightPwm = constrain(BASE_PWM - correction, MIN_PWM, MAX_PWM);
    leftPwm = (int)(targetLeftPwm * scale);
    rightPwm = (int)(targetRightPwm * scale);
    wheelController.drive(leftPwm, rightPwm);
  }

#if SERIAL_PLOTTER_PRINT
  Serial.print(F("L:"));
  Serial.print(lastSense.left);
  Serial.print(F("\tR:"));
  Serial.print(lastSense.right);
  Serial.print(F("\tCorr:"));
  Serial.print(correction);
  Serial.print(F("\tLPWM:"));
  Serial.print(leftPwm);
  Serial.print(F("\tRPWM:"));
  Serial.print(rightPwm);
  Serial.print(F("\tIR:"));
  Serial.println(lastSense.obstacle ? 1 : 0);
#endif
}
