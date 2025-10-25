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

  s.anyBlackFront = s.isBlackL || s.isBlackC || s.isBlackR;
  s.allBlackFront = s.isBlackL && s.isBlackC && s.isBlackR;
  s.allWhiteFront = !s.anyBlackFront;

  s.anyBlackRear = s.isBlackRL || s.isBlackRR;
  s.allBlackRear = s.isBlackRL && s.isBlackRR;
  s.allWhiteRear = !s.anyBlackRear;

  s.anyBlack = s.anyBlackFront || s.anyBlackRear;
  s.allBlack = s.allBlackFront && s.allBlackRear;
  s.allWhite = s.allWhiteFront && s.allWhiteRear;

  s.frontBlackDirState = getFrontBlackDirState(s);
  s.rearBlackDirState = getRearBlackDirState(s);
  if (s.anyBlackFront) {
    s.lastBlackSensorPosition = SensorPosition::Front;
  } else if (s.anyBlackRear) {
    s.lastBlackSensorPosition = SensorPosition::Rear;
  } else {
    s.lastBlackSensorPosition = SensorPosition::Front;
  }

  displaySensorStates(s.isBlackL,
                      s.isBlackC,
                      s.isBlackR,
                      s.isBlackRL,
                      s.isBlackRR);

  return s;
}

int getFrontBlackDirState(const Sense& s) {
  if (s.isBlackL && !s.isBlackR) {
    return -1;
  }
  if (s.isBlackR && !s.isBlackL) {
    return +1;
  }
  if (s.isBlackC || s.allBlackFront) {
    return 0;
  }
  return 0;
}

int getRearBlackDirState(const Sense& s) {
  if (s.isBlackRL && !s.isBlackRR) {
    return -1;
  }
  if (s.isBlackRR && !s.isBlackRL) {
    return +1;
  }
  if (s.allBlackRear) {
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
