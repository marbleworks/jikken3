#include "sensors.h"

#include "pins.h"
#include "sensor_leds.h"

extern int THRESHOLD;
extern int HYST;

namespace {
int lastBlackDirState = 0;  // -1=左, +1=右, 0=中央/不明
}

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

  if (s.isBlackL && !s.isBlackR)      lastBlackDirState = -1;
  else if (s.isBlackR && !s.isBlackL) lastBlackDirState = +1;
  else if (s.isBlackC)                lastBlackDirState = 0;
  else if (s.allBlack)                lastBlackDirState = 0;

  displaySensorStates(s.isBlackL, s.isBlackC, s.isBlackR);

  return s;
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

int getLastBlackDir() {
  return lastBlackDirState;
}

