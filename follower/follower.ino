#include <Arduino.h>

#include "distance_sensor.h"
#include "obstacle_sensor.h"
#include "wheel_control.h"

// ------------------ ピン設定 ------------------
const uint8_t PIN_TRIG = 8;
const uint8_t PIN_ECHO = 9;

const uint8_t PIN_IR_OBST = 7;

const uint8_t PIN_LEFT_IN1  = 2;
const uint8_t PIN_LEFT_IN2  = 3;
const uint8_t PIN_LEFT_PWM  = 5;

const uint8_t PIN_RIGHT_IN1 = 4;
const uint8_t PIN_RIGHT_IN2 = 12;
const uint8_t PIN_RIGHT_PWM = 6;

// ------------------ 制御パラメータ ------------------
const float TARGET_DISTANCE_CM = 25.0f;
const float KP = 6.0f;
const float KD = 2.5f;
const float MAX_DISTANCE_CM = 200.0f;
const float MIN_DISTANCE_CM = 5.0f;

const uint8_t MIN_PWM = 0;
const uint8_t MAX_PWM = 200;
const uint8_t DEAD_PWM = 30;

const unsigned long SONAR_INTERVAL_MS = 60;
const unsigned long LOST_TIMEOUT_MS   = 1500;

// ------------------ モジュール初期化 ------------------
WheelControl wheelController({PIN_LEFT_IN1, PIN_LEFT_IN2, PIN_LEFT_PWM,
                              PIN_RIGHT_IN1, PIN_RIGHT_IN2, PIN_RIGHT_PWM,
                              MIN_PWM, MAX_PWM, DEAD_PWM});
DistanceSensor distanceSensor(PIN_TRIG, PIN_ECHO, MAX_DISTANCE_CM);
ObstacleSensor obstacleSensor(PIN_IR_OBST, true);

// ------------------ 状態変数 ------------------
unsigned long lastPingTime = 0;
float        lastDistance  = TARGET_DISTANCE_CM;
float        lastError     = 0.0f;
unsigned long lastSeenTime = 0;

void setup()
{
  distanceSensor.begin();
  obstacleSensor.begin();
  wheelController.begin();

  Serial.begin(115200);
  Serial.println(F("Follower ready"));
}

void loop()
{
  unsigned long now = millis();

  if (obstacleSensor.detected())
  {
    wheelController.stop();
    Serial.println(F("Obstacle detected by IR sensor - stopping"));
    delay(50);
    return;
  }

  if (now - lastPingTime >= SONAR_INTERVAL_MS)
  {
    float distance = distanceSensor.readDistanceCm();
    lastPingTime = now;

    if (!isnan(distance))
    {
      lastSeenTime = now;
      lastDistance = distance;
    }
  }

  if (now - lastSeenTime > LOST_TIMEOUT_MS)
  {
    wheelController.stop();
    Serial.println(F("Target lost - waiting"));
    delay(50);
    return;
  }

  float error = TARGET_DISTANCE_CM - lastDistance;
  float derivative = (error - lastError) / (SONAR_INTERVAL_MS / 1000.0f);
  lastError = error;

  float control = KP * error + KD * derivative;
  int pwm = (int)round(control);
  pwm = constrain(pwm, -MAX_PWM, MAX_PWM);

  if (lastDistance <= MIN_DISTANCE_CM)
  {
    pwm = -MAX_PWM;
  }

  wheelController.drive(pwm, pwm);

  if (now % 500 < 20)
  {
    Serial.print(F("distance="));
    Serial.print(lastDistance);
    Serial.print(F("cm, pwm="));
    Serial.println(pwm);
  }
}
