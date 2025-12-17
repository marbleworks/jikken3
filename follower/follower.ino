#include <Arduino.h>

#include "distance_sensor.h"
#include "obstacle_sensor.h"
#include "wheel_control.h"
#include "lcd_display.h"
#include "moving_average.h"
#include "pins.h"

// Serial Plotter出力を有効化する場合は 1 に設定する。
#ifndef SERIAL_PLOTTER_PRINT
#define SERIAL_PLOTTER_PRINT 1
#endif

// ------------------ 制御パラメータ ------------------
const int BASE_PWM = 100;           // 固定の前進速度
const int TURN_PWM = 50;            // 旋回時のPWM差分
const float LOST_THRESHOLD_CM = 30.0f;  // 見失い閾値
const float MAX_DISTANCE_CM = 40.0f;

const uint8_t MIN_PWM = 0;
const uint8_t MAX_PWM = 255;
const uint8_t DEAD_PWM = 30;

const size_t MOVING_AVG_SIZE = 5;

const unsigned long SONAR_INTERVAL_MS = 60;
const unsigned long LCD_UPDATE_MS     = 200;  // LCD更新間隔（ドット応答速度に合わせる）

// ------------------ モジュール初期化 ------------------
WheelControl wheelController({PIN_LEFT_IN1, PIN_LEFT_IN2, PIN_LEFT_PWM,
                              PIN_RIGHT_IN1, PIN_RIGHT_IN2, PIN_RIGHT_PWM,
                              MIN_PWM, MAX_PWM, DEAD_PWM});
DistanceSensor leftDistanceSensor(PIN_TRIG_LEFT, PIN_ECHO_LEFT, MAX_DISTANCE_CM);
DistanceSensor rightDistanceSensor(PIN_TRIG_RIGHT, PIN_ECHO_RIGHT, MAX_DISTANCE_CM);
ObstacleSensor obstacleSensor(PIN_IR_OBST, true);
LCDDisplay lcdDisplay(PIN_LCD_RS, PIN_LCD_E, PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7);

// ------------------ 状態変数 ------------------
unsigned long lastPingTime     = 0;
unsigned long lastLcdTime      = 0;
float        lastLeftDistance  = 0.0f;
float        lastRightDistance = 0.0f;
float        rawLeftDistance   = NAN;  // センサ生値（LCD表示用）
float        rawRightDistance  = NAN;  // センサ生値（LCD表示用）

MovingAverage<MOVING_AVG_SIZE> leftDistanceFilter;
MovingAverage<MOVING_AVG_SIZE> rightDistanceFilter;

void setup()
{
  leftDistanceSensor.begin();
  rightDistanceSensor.begin();
  obstacleSensor.begin();
  wheelController.begin();
  lcdDisplay.begin();

  Serial.begin(115200);
  Serial.println(F("Follower ready"));
}

void loop()
{
  unsigned long now = millis();

  // センサ読み取り（常に実行）
  if (now - lastPingTime >= SONAR_INTERVAL_MS)
  {
    // モーターを一時停止してノイズを排除
    wheelController.stop();
    delayMicroseconds(500);

    float leftDistance = leftDistanceSensor.readDistanceCm();
    delay(5);
    float rightDistance = rightDistanceSensor.readDistanceCm();
    lastPingTime = now;

    // 生値を保存（LCD表示用）
    rawLeftDistance = leftDistance;
    rawRightDistance = rightDistance;

    bool leftValid = !isnan(leftDistance);
    bool rightValid = !isnan(rightDistance);

    // NANは最大距離として扱う（何も検出できない＝遠い）
    float leftValue = leftValid ? leftDistance : MAX_DISTANCE_CM;
    float rightValue = rightValid ? rightDistance : MAX_DISTANCE_CM;

    lastLeftDistance = leftDistanceFilter.add(leftValue);
    lastRightDistance = rightDistanceFilter.add(rightValue);
  }

  // LCD表示更新（独立した間隔で）
  if (now - lastLcdTime >= LCD_UPDATE_MS)
  {
    lcdDisplay.showDistances(rawLeftDistance, rawRightDistance);
    lastLcdTime = now;
  }

  // 検出判定: 検出=1, 見失い=0
  int rightDetect = (lastRightDistance < LOST_THRESHOLD_CM) ? 1 : 0;
  int leftDetect = (lastLeftDistance < LOST_THRESHOLD_CM) ? 1 : 0;

  // 旋回値: 右検出+1, 左検出-1
  int turnDirection = rightDetect - leftDetect;

  bool irDetected = obstacleSensor.detected();
  int leftPwm = 0;
  int rightPwm = 0;

  if (irDetected)
  {
    wheelController.stop();
  }
  else
  {
    leftPwm = BASE_PWM + turnDirection * TURN_PWM;
    rightPwm = BASE_PWM - turnDirection * TURN_PWM;
    wheelController.drive(leftPwm, rightPwm);
  }

#if SERIAL_PLOTTER_PRINT
  // Serial Plotter用出力
  Serial.print(F("RawL:"));
  Serial.print(rawLeftDistance);
  Serial.print(F("\tRawR:"));
  Serial.print(rawRightDistance);
  Serial.print(F("\tLeft:"));
  Serial.print(lastLeftDistance);
  Serial.print(F("\tRight:"));
  Serial.print(lastRightDistance);
  Serial.print(F("\tTurn:"));
  Serial.print(turnDirection);
  Serial.print(F("\tLPWM:"));
  Serial.print(leftPwm);
  Serial.print(F("\tRPWM:"));
  Serial.print(rightPwm);
  Serial.print(F("\tIR:"));
  Serial.println(irDetected ? 1 : 0);
#endif
}
