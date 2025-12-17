#include "distance_sensor.h"

DistanceSensor::DistanceSensor(uint8_t trigPin, uint8_t echoPin, float maxDistanceCm)
    : trig(trigPin), echo(echoPin), maxDistance(maxDistanceCm) {}

void DistanceSensor::begin() const
{
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  digitalWrite(trig, LOW);
}

float DistanceSensor::readDistanceCm() const
{
  long duration = pingMicroseconds();
  if (duration <= 0)
  {
    return NAN;
  }

  float distance = duration / 58.0f;
  return distance;
}

long DistanceSensor::pingMicroseconds() const
{
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  const unsigned long timeoutUs = (unsigned long)(maxDistance * 58.0f);
  return pulseIn(echo, HIGH, timeoutUs);
}
