#include <Arduino.h>

// ------------------ ピン設定 ------------------
// HC-SR04 超音波距離センサ
const uint8_t PIN_TRIG = 8;
const uint8_t PIN_ECHO = 9;

// SSR1017 赤外線障害物センサ（デジタル出力）
const uint8_t PIN_IR_OBST = 7;

// 左右モータ（TA7291P）制御ピン
const uint8_t PIN_LEFT_IN1  = 2;
const uint8_t PIN_LEFT_IN2  = 3;
const uint8_t PIN_LEFT_PWM  = 5;  // タイマ0系以外のPWMピンを推奨

const uint8_t PIN_RIGHT_IN1 = 4;
const uint8_t PIN_RIGHT_IN2 = 12;
const uint8_t PIN_RIGHT_PWM = 6;

// ------------------ 制御パラメータ ------------------
const float TARGET_DISTANCE_CM = 25.0f;  // 追従したい距離
const float KP = 6.0f;                   // 前後制御の比例ゲイン
const float KD = 2.5f;                   // 微分ゲイン（距離変化の減衰）
const float MAX_DISTANCE_CM = 200.0f;    // センサ信頼範囲
const float MIN_DISTANCE_CM = 5.0f;      // これ以下は安全停止

const uint8_t MIN_PWM = 0;
const uint8_t MAX_PWM = 200;             // 0-255 の範囲で適宜調整
const uint8_t DEAD_PWM = 30;             // モータデッドゾーン補正

const unsigned long SONAR_INTERVAL_MS = 60;  // 測距周期（ms）
const unsigned long LOST_TIMEOUT_MS   = 1500; // ターゲット喪失判定

// ------------------ 状態変数 ------------------
unsigned long lastPingTime = 0;
float        lastDistance  = TARGET_DISTANCE_CM;
float        lastError     = 0.0f;
unsigned long lastSeenTime = 0;

// ------------------ ユーティリティ ------------------
long pingMicroseconds()
{
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  // pulseIn のタイムアウトを200cm換算に設定
  const unsigned long timeoutUs = (unsigned long)(MAX_DISTANCE_CM * 58UL * 2UL);
  return pulseIn(PIN_ECHO, HIGH, timeoutUs);
}

float measureDistanceCm()
{
  long duration = pingMicroseconds();
  if (duration == 0)
  {
    return NAN;
  }
  // 音速換算（往復）: 1cm ≒ 58us
  float distance = duration / 58.0f;
  return distance;
}

void setMotor(int leftPwm, int rightPwm)
{
  auto driveOne = [](uint8_t in1, uint8_t in2, uint8_t pwmPin, int pwm)
  {
    int magnitude = constrain(abs(pwm), MIN_PWM, MAX_PWM);
    if (magnitude == 0)
    {
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      analogWrite(pwmPin, 0);
      return;
    }

    if (magnitude > 0)
    {
      magnitude = max(magnitude, (int)DEAD_PWM);
    }

    bool forward = pwm >= 0;
    digitalWrite(in1, forward ? HIGH : LOW);
    digitalWrite(in2, forward ? LOW : HIGH);
    analogWrite(pwmPin, magnitude);
  };

  driveOne(PIN_LEFT_IN1, PIN_LEFT_IN2, PIN_LEFT_PWM, leftPwm);
  driveOne(PIN_RIGHT_IN1, PIN_RIGHT_IN2, PIN_RIGHT_PWM, rightPwm);
}

void stopMotors()
{
  setMotor(0, 0);
}

bool obstacleDetected()
{
  // SSR1017 は近接時に LOW を返す仕様が一般的
  return digitalRead(PIN_IR_OBST) == LOW;
}

void setup()
{
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_IR_OBST, INPUT_PULLUP);

  pinMode(PIN_LEFT_IN1, OUTPUT);
  pinMode(PIN_LEFT_IN2, OUTPUT);
  pinMode(PIN_LEFT_PWM, OUTPUT);

  pinMode(PIN_RIGHT_IN1, OUTPUT);
  pinMode(PIN_RIGHT_IN2, OUTPUT);
  pinMode(PIN_RIGHT_PWM, OUTPUT);

  stopMotors();

  Serial.begin(115200);
  Serial.println(F("Follower ready"));
}

void loop()
{
  unsigned long now = millis();

  if (obstacleDetected())
  {
    stopMotors();
    Serial.println(F("Obstacle detected by IR sensor - stopping"));
    delay(50);
    return;
  }

  if (now - lastPingTime >= SONAR_INTERVAL_MS)
  {
    float distance = measureDistanceCm();
    lastPingTime = now;

    if (!isnan(distance) && distance <= MAX_DISTANCE_CM)
    {
      lastSeenTime = now;
      lastDistance = distance;
    }
  }

  if (now - lastSeenTime > LOST_TIMEOUT_MS)
  {
    stopMotors();
    Serial.println(F("Target lost - waiting"));
    delay(50);
    return;
  }

  float error = TARGET_DISTANCE_CM - lastDistance;
  float derivative = (error - lastError) / (SONAR_INTERVAL_MS / 1000.0f);
  lastError = error;

  float control = KP * error + KD * derivative;

  // 前後移動を左右同じ速度で出力
  int pwm = (int)round(control);

  // 距離が近すぎる場合はバック（負のPWM）
  pwm = constrain(pwm, -MAX_PWM, MAX_PWM);

  // 安全距離以下は最優先で後退
  if (lastDistance <= MIN_DISTANCE_CM)
  {
    pwm = -MAX_PWM;
  }

  setMotor(pwm, pwm);

  // ログ
  if (now % 500 < 20)
  {
    Serial.print(F("distance="));
    Serial.print(lastDistance);
    Serial.print(F("cm, pwm="));
    Serial.println(pwm);
  }
}
