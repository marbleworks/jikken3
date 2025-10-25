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

Sense readSensors(SensorOrientation orientation) {
  Sense s{};
  s.orientation = orientation;
  s.hasCenter = (orientation == SensorOrientation::Front);

  static bool lastFrontLBlack = false;
  static bool lastFrontCBlack = false;
  static bool lastFrontRBlack = false;
  static bool lastBackLBlack  = false;
  static bool lastBackRBlack  = false;

  int thH = THRESHOLD + HYST;
  int thL = THRESHOLD - HYST;

  if (orientation == SensorOrientation::Front) {
    s.rawL = analogRead(pinL);
    s.rawC = analogRead(pinC);
    s.rawR = analogRead(pinR);

    s.isBlackL = applyHysteresis(s.rawL, lastFrontLBlack, thH, thL);
    s.isBlackC = applyHysteresis(s.rawC, lastFrontCBlack, thH, thL);
    s.isBlackR = applyHysteresis(s.rawR, lastFrontRBlack, thH, thL);

    lastFrontLBlack = s.isBlackL;
    lastFrontCBlack = s.isBlackC;
    lastFrontRBlack = s.isBlackR;
  } else {
    s.rawL = analogRead(pinBackL);
    s.rawR = analogRead(pinBackR);
    s.rawC = (s.rawL + s.rawR) / 2;

    s.isBlackL = applyHysteresis(s.rawL, lastBackLBlack, thH, thL);
    s.isBlackR = applyHysteresis(s.rawR, lastBackRBlack, thH, thL);
    s.isBlackC = false;

    lastBackLBlack = s.isBlackL;
    lastBackRBlack = s.isBlackR;
  }

  bool centerActive = s.hasCenter && s.isBlackC;
  s.anyBlack = s.isBlackL || centerActive || s.isBlackR;
  s.allBlack = s.isBlackL && s.isBlackR && (!s.hasCenter || s.isBlackC);
  s.allWhite = !s.anyBlack;

  displaySensorStates(orientation, s.isBlackL, centerActive, s.isBlackR);

  return s;
}

int getBlackDirState(const Sense& s) {
  if (s.isBlackL && !s.isBlackR) {
    return -1;
  }
  if (s.isBlackR && !s.isBlackL) {
    return +1;
  }

  if (s.hasCenter) {
    if (s.isBlackC || s.allBlack) {
      return 0;
    }
  } else {
    if (s.isBlackL && s.isBlackR) {
      return 0;
    }
  }

  return 0;
}

float computeError(const Sense& sense) {
  static float lastErrFront = 0.0f;
  static float lastErrBack  = 0.0f;

  auto norm = [&](int v) -> float {
    float x = (v - LINE_WHITE) / (LINE_BLACK - LINE_WHITE);
    if (x < 0.0f) x = 0.0f;
    if (x > 1.0f) x = 1.0f;
    return x;
  };

  float bL = norm(sense.rawL);
  float bR = norm(sense.rawR);
  float bC = sense.hasCenter ? norm(sense.rawC) : 0.0f;

  float sum = bL + bR + (sense.hasCenter ? bC : 0.0f);

  float& lastErr = (sense.orientation == SensorOrientation::Front) ?
                   lastErrFront : lastErrBack;

  if (sum < LINE_EPS) {
    return lastErr;
  }

  float err = (-1.0f * bL + 1.0f * bR) / sum;

  lastErr = err;
  return err;
}
