#include "wheel_control.h"

WheelControl::WheelControl(const WheelControlConfig &config) : cfg(config) {}

void WheelControl::begin() const
{
  pinMode(cfg.leftIn1, OUTPUT);
  pinMode(cfg.leftIn2, OUTPUT);
  pinMode(cfg.leftPwm, OUTPUT);

  pinMode(cfg.rightIn1, OUTPUT);
  pinMode(cfg.rightIn2, OUTPUT);
  pinMode(cfg.rightPwm, OUTPUT);

  stop();
}

void WheelControl::drive(int leftPwm, int rightPwm) const
{
  driveSingle(cfg.leftIn1, cfg.leftIn2, cfg.leftPwm, leftPwm);
  driveSingle(cfg.rightIn1, cfg.rightIn2, cfg.rightPwm, rightPwm);
}

void WheelControl::stop() const
{
  drive(0, 0);
}

void WheelControl::driveSingle(uint8_t in1, uint8_t in2, uint8_t pwmPin, int pwm) const
{
  int magnitude = constrain(abs(pwm), cfg.minPwm, cfg.maxPwm);
  if (magnitude == 0)
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(pwmPin, 0);
    return;
  }

  if (magnitude > 0)
  {
    magnitude = max(magnitude, (int)cfg.deadPwm);
  }

  bool forward = pwm >= 0;
  digitalWrite(in1, forward ? HIGH : LOW);
  digitalWrite(in2, forward ? LOW : HIGH);
  analogWrite(pwmPin, magnitude);
}
