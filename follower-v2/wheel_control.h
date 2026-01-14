#pragma once

#include <Arduino.h>

struct WheelControlConfig
{
  uint8_t leftIn1;
  uint8_t leftIn2;
  uint8_t leftPwm;
  uint8_t rightIn1;
  uint8_t rightIn2;
  uint8_t rightPwm;
  uint8_t minPwm;
  uint8_t maxPwm;
  uint8_t deadPwm;
};

class WheelControl
{
public:
  explicit WheelControl(const WheelControlConfig &config);

  void begin() const;
  void drive(int leftPwm, int rightPwm) const;
  void stop() const;

private:
  void driveSingle(uint8_t in1, uint8_t in2, uint8_t pwmPin, int pwm) const;

  WheelControlConfig cfg;
};
