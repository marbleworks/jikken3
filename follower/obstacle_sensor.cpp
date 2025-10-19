#include "obstacle_sensor.h"

ObstacleSensor::ObstacleSensor(uint8_t inputPin, bool activeLow)
    : pin(inputPin), activeLow(activeLow) {}

void ObstacleSensor::begin() const
{
  pinMode(pin, activeLow ? INPUT_PULLUP : INPUT);
}

bool ObstacleSensor::detected() const
{
  int value = digitalRead(pin);
  return activeLow ? (value == LOW) : (value == HIGH);
}
