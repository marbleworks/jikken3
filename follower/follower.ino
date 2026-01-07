#include <Arduino.h>
#include <math.h>

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
const int BASE_PWM = 255;           // 固定の前進速度
const int TURN_PWM = 160;            // 旋回時のPWM差分
const float LOST_THRESHOLD_CM = 30.0f;  // 見失い閾値
const float MAX_DISTANCE_CM = 40.0f;

const uint8_t MIN_PWM = 0;
const uint8_t MAX_PWM = 255;
const uint8_t DEAD_PWM = 30;

const size_t MOVING_AVG_SIZE = 1;

const unsigned long SONAR_INTERVAL_MS = 80;
const unsigned long LCD_UPDATE_MS     = 200;  // LCD更新間隔（ドット応答速度に合わせる）
const unsigned long RECOVER_TIMEOUT_MS = 500; // リカバリモードに移行するまでの時間
const int RECOVER_TURN_PWM = 160;             // リカバリ時の旋回PWM差分

// ------------------ PID制御パラメータ ------------------
float KP = 0.5f;             // 比例ゲイン（距離差 → 補正量）
float KI = 0.0f;             // 積分ゲイン
float KD = 0.03f;            // 微分ゲイン
float PID_I_LIMIT = 50.0f;   // 積分項の上限
float CORR_EXP = 1.0f;       // 補正量の非線形指数（1で線形）
int   CORR_MAX = 100;        // 補正量の最大PWM値

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
float        lastLeftDistance  = MAX_DISTANCE_CM;  // 初期値は検知なし
float        lastRightDistance = MAX_DISTANCE_CM; // 初期値は検知なし
float        rawLeftDistance   = NAN;  // センサ生値（LCD表示用）
float        rawRightDistance  = NAN;  // センサ生値（LCD表示用）
int          lastDetectDirection = 0;  // 最後に検知した方向 (+1:右, -1:左, 0:両方/なし)
unsigned long turnStartTime    = 0;    // 旋回開始時刻（片方見失い or 両方見失い）
bool         readLeftNext      = true; // 次に読むセンサー（交互読み取り用）

// ------------------ PID状態 ------------------
struct PIDState {
  float integral;
  float lastError;
  unsigned long lastTimeMs;
};
PIDState pidState = {0.0f, 0.0f, 0};

MovingAverage<MOVING_AVG_SIZE> leftDistanceFilter;
MovingAverage<MOVING_AVG_SIZE> rightDistanceFilter;

// ------------------ PID制御関数 ------------------
// 両方検知時に左右距離差をエラーとしてPID制御を行う
// 戻り値: 補正量（正=右に曲がる、負=左に曲がる）
int computePidCorrection(float leftDist, float rightDist)
{
  // エラー: 左距離 - 右距離（正=リーダーが右寄り→右に曲がる）
  // 右が近い→error正→左輪加速・右輪減速→右に曲がる
  float error = leftDist - rightDist;

  unsigned long now = millis();
  float dt = 0.0f;
  if (pidState.lastTimeMs != 0)
  {
    dt = (now - pidState.lastTimeMs) / 1000.0f;
  }
  pidState.lastTimeMs = now;

  // 積分項
  if (dt > 0.0f)
  {
    pidState.integral += error * dt;
    pidState.integral = constrain(pidState.integral, -PID_I_LIMIT, PID_I_LIMIT);
  }

  // 微分項
  float derivative = 0.0f;
  if (dt > 0.0f)
  {
    derivative = (error - pidState.lastError) / dt;
  }
  pidState.lastError = error;

  // PID出力
  float output = KP * error + KI * pidState.integral + KD * derivative;

  // leaderと同じ方式: 正規化 → 非線形変換 → スケーリング
  float corrNorm = constrain(output, -1.0f, 1.0f);
  float corrMagnitude = powf(fabsf(corrNorm), CORR_EXP);
  float corrScaled = copysignf(corrMagnitude, corrNorm);
  int corr = (int)(corrScaled * (float)CORR_MAX);

  return corr;
}

// PID状態をリセット
void resetPidState()
{
  pidState.integral = 0.0f;
  pidState.lastError = 0.0f;
  pidState.lastTimeMs = 0;
}

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

  // センサ読み取り（交互に片方ずつ読んで停止時間を短縮）
  if (now - lastPingTime >= SONAR_INTERVAL_MS)
  {
    // モーターを一時停止してノイズを排除
    wheelController.stop();
    delayMicroseconds(500);

    if (readLeftNext)
    {
      float leftDistance = leftDistanceSensor.readDistanceCm();
      rawLeftDistance = leftDistance;
      bool leftValid = !isnan(leftDistance);
      float leftValue = leftValid ? leftDistance : MAX_DISTANCE_CM;
      lastLeftDistance = leftDistanceFilter.add(leftValue);
    }
    else
    {
      float rightDistance = rightDistanceSensor.readDistanceCm();
      rawRightDistance = rightDistance;
      bool rightValid = !isnan(rightDistance);
      float rightValue = rightValid ? rightDistance : MAX_DISTANCE_CM;
      lastRightDistance = rightDistanceFilter.add(rightValue);
    }

    readLeftNext = !readLeftNext;
    lastPingTime = now;
  }

  // LCD表示更新（独立した間隔で）
  if (now - lastLcdTime >= LCD_UPDATE_MS)
  {
    lcdDisplay.showDistances(rawLeftDistance, rawRightDistance, lastDetectDirection);
    lastLcdTime = now;
  }

  // 検出判定: 検出=1, 見失い=0
  int rightDetect = (lastRightDistance < LOST_THRESHOLD_CM) ? 1 : 0;
  int leftDetect = (lastLeftDistance < LOST_THRESHOLD_CM) ? 1 : 0;

  // 旋回値: 右検出+1, 左検出-1
  int turnDirection = rightDetect - leftDetect;

  // 片方だけ検出している場合、その方向を記憶
  if (turnDirection != 0)
  {
    lastDetectDirection = turnDirection;
  }

  // 状態判定
  bool bothLost = (rightDetect == 0 && leftDetect == 0);
  bool bothDetected = (rightDetect == 1 && leftDetect == 1);
  bool oneLost = (turnDirection != 0);  // 片方だけ検出
  int currentTurnPwm = TURN_PWM;
  int pidCorr = 0;

  if (bothDetected)
  {
    // 両方検出時: PID制御を適用
    turnStartTime = 0;
    pidCorr = computePidCorrection(lastLeftDistance, lastRightDistance);
    // 距離差から方向を記憶（見失い時のリカバリ用）
    if (lastLeftDistance > lastRightDistance)
    {
      lastDetectDirection = 1;  // 右寄り
    }
    else if (lastLeftDistance < lastRightDistance)
    {
      lastDetectDirection = -1; // 左寄り
    }
  }
  else if (bothLost || oneLost)
  {
    // 片方または両方見失い: リカバリモード
    // PID状態をリセット
    resetPidState();

    // 旋回開始時刻を記録
    if (turnStartTime == 0)
    {
      turnStartTime = now;
    }

    // 経過時間に応じて旋回PWMを切り替え
    unsigned long elapsed = now - turnStartTime;
    if (elapsed >= RECOVER_TIMEOUT_MS)
    {
      currentTurnPwm = RECOVER_TURN_PWM;
    }
    // else: currentTurnPwm = TURN_PWM (初期値のまま)

    // 両方見失った場合は右旋回
    if (bothLost)
    {
      turnDirection = 1;  // 常に右旋回
    }
  }

  bool irDetected = obstacleSensor.detected();
  int leftPwm = 0;
  int rightPwm = 0;

  if (irDetected)
  {
    wheelController.stop();
  }
  else if (bothDetected)
  {
    // 両方検知時: PID制御で追従
    leftPwm = constrain(BASE_PWM + pidCorr, MIN_PWM, MAX_PWM);
    rightPwm = constrain(BASE_PWM - pidCorr, MIN_PWM, MAX_PWM);
    wheelController.drive(leftPwm, rightPwm);
  }
  else
  {
    // 片方検知 or 見失い: 従来の旋回制御
    leftPwm = constrain(BASE_PWM + turnDirection * currentTurnPwm, MIN_PWM, MAX_PWM);
    rightPwm = constrain(BASE_PWM - turnDirection * currentTurnPwm, MIN_PWM, MAX_PWM);
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
  Serial.print(F("\tPID:"));
  Serial.print(pidCorr);
  Serial.print(F("\tLPWM:"));
  Serial.print(leftPwm);
  Serial.print(F("\tRPWM:"));
  Serial.print(rightPwm);
  Serial.print(F("\tIR:"));
  Serial.println(irDetected ? 1 : 0);
#endif
}
