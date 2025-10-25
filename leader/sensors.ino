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
  s.rawRL = analogRead(pinRL);
  s.rawRR = analogRead(pinRR);

  static bool lastLBlack = false;
  static bool lastCBlack = false;
  static bool lastRBlack = false;
  static bool lastRLBlack = false;
  static bool lastRRBlack = false;
  int thH = THRESHOLD + HYST;
  int thL = THRESHOLD - HYST;

  s.isBlackL = applyHysteresis(s.rawL, lastLBlack, thH, thL);
  s.isBlackC = applyHysteresis(s.rawC, lastCBlack, thH, thL);
  s.isBlackR = applyHysteresis(s.rawR, lastRBlack, thH, thL);
  s.isBlackRL = applyHysteresis(s.rawRL, lastRLBlack, thH, thL);
  s.isBlackRR = applyHysteresis(s.rawRR, lastRRBlack, thH, thL);

  lastLBlack = s.isBlackL;
  lastCBlack = s.isBlackC;
  lastRBlack = s.isBlackR;
  lastRLBlack = s.isBlackRL;
  lastRRBlack = s.isBlackRR;

  s.anyBlack = s.isBlackL || s.isBlackC || s.isBlackR || s.isBlackRL || s.isBlackRR;
  s.allBlack = s.isBlackL && s.isBlackC && s.isBlackR && s.isBlackRL && s.isBlackRR;
  s.allWhite = !s.anyBlack;

  displaySensorStates(s.isBlackL,
                      s.isBlackC,
                      s.isBlackR,
                      s.isBlackRL,
                      s.isBlackRR);

  return s;
}

int getBlackDirState(const Sense& s) {
  int leftCount = (s.isBlackL ? 1 : 0) + (s.isBlackRL ? 1 : 0);
  int rightCount = (s.isBlackR ? 1 : 0) + (s.isBlackRR ? 1 : 0);

  if (leftCount > rightCount) {
    return -1;
  }
  if (rightCount > leftCount) {
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
