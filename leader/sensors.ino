#include "sensors.h"

#include "pins.h"
#include "sensor_leds.h"

extern int THRESHOLD;
extern int HYST;
extern float LINE_WHITE;
extern float LINE_BLACK;
extern float LINE_EPS;

static bool applyHysteresis(int raw, bool lastState, int thH, int thL) {
  return lastState ? (raw > thL) : (raw > thH);
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

  s.isBlackL = applyHysteresis(s.rawL, lastLBlack, thH, thL);
  s.isBlackC = applyHysteresis(s.rawC, lastCBlack, thH, thL);
  s.isBlackR = applyHysteresis(s.rawR, lastRBlack, thH, thL);

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
  static float lastErr = 0.0f;

  auto norm = [&](int v) -> float {
    float x = (v - LINE_WHITE) / (LINE_BLACK - LINE_WHITE);
    if (x < 0.0f) x = 0.0f;
    if (x > 1.0f) x = 1.0f;
    return x;
  };

  float bL = norm(rawL);
  float bC = norm(rawC);
  float bR = norm(rawR);

  float s = bL + bC + bR;
  if (s < LINE_EPS) {
    return lastErr;
  }

  float err = (-1.0f * bL + 1.0f * bR) / s;

  lastErr = err;
  return err;
}
