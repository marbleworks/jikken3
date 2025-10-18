#include "sensors.h"

#include "pins.h"
#include "sensor_leds.h"

extern int THRESHOLD;
extern int HYST;

Sense readSensors() {
  Sense s{};
  s.rawL = analogRead(pinL);
  s.rawC = analogRead(pinC);
  s.rawR = analogRead(pinR);

  static bool lastLBlack = false;
  static bool lastCBlack = false;
  static bool lastRBlack = false;
  int thH = THRESHOLD + HYST;
  int thL = THRESHOLD - HYST;

  if (lastLBlack) s.isBlackL = (s.rawL > thL);
  else            s.isBlackL = (s.rawL > thH);

  if (lastCBlack) s.isBlackC = (s.rawC > thL);
  else            s.isBlackC = (s.rawC > thH);

  if (lastRBlack) s.isBlackR = (s.rawR > thL);
  else            s.isBlackR = (s.rawR > thH);

  lastLBlack = s.isBlackL;
  lastCBlack = s.isBlackC;
  lastRBlack = s.isBlackR;

  s.anyBlack = s.isBlackL || s.isBlackC || s.isBlackR;
  s.allBlack = s.isBlackL && s.isBlackC && s.isBlackR;
  s.allWhite = !s.anyBlack;

  displaySensorStates(s.isBlackL, s.isBlackC, s.isBlackR);

  return s;
}

int getBlackDirState(const Sense& s) {
  if (s.isBlackL && !s.isBlackR) {
    return -1;
  }
  if (s.isBlackR && !s.isBlackL) {
    return +1;
  }
  if (s.isBlackC || s.allBlack) {
    return 0;
  }
  return 0;
}

float computeError(int rawL, int rawC, int rawR) {
  const float span = 1000.0f - 40.0f;
  float weightL = max(0.0f, (float)(rawL - THRESHOLD)) / span;
  float weightC = max(0.0f, (float)(rawC - THRESHOLD)) / span;
  float weightR = max(0.0f, (float)(rawR - THRESHOLD)) / span;
  float total = weightL + weightC + weightR;
  if (total < 0.001f) {
    return 0.0f;
  }
  float position = (-1.0f * weightL + 1.0f * weightR) / total;
  return position;
}
