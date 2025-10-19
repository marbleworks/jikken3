#pragma once

#include <Arduino.h>

class ObstacleSensor
{
public:
  ObstacleSensor(uint8_t inputPin, bool activeLow = true);

  void begin() const;
  bool detected() const;

private:
  uint8_t pin;
  bool activeLow;
};
