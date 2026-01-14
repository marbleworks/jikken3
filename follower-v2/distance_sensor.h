#pragma once

#include <Arduino.h>

class DistanceSensor
{
public:
  DistanceSensor(uint8_t trigPin, uint8_t echoPin, float maxDistanceCm);

  void begin() const;
  float readDistanceCm() const;

private:
  long pingMicroseconds() const;

  uint8_t trig;
  uint8_t echo;
  float maxDistance;
};
