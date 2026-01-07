#include "sensors.h"
#include "pins.h"

extern int THRESHOLD;
extern int HYST;
extern float LINE_WHITE;
extern float LINE_BLACK;
extern float LINE_EPS;

namespace {

bool applyHysteresis(int raw, bool lastState, int thH, int thL) {
  return lastState ? (raw > thL) : (raw > thH);
}

int computeBlackDirState(const Sense& s) {
  bool left = false;
  bool right = false;
  bool center = false;

  for (size_t i = 0; i < SENSOR_COUNT; ++i) {
    if (!s.isBlack[i]) continue;
    if (i < 2)      left = true;
    else if (i > 2) right = true;
    else            center = true;
  }

  if (left && !right) return -1;
  if (right && !left) return +1;
  return 0;
}

}  // namespace

Sense readSensors() {
  static bool lastBlack[SENSOR_COUNT] = {};
  static int lastDirState = 0;

  Sense s{};
  int thH = THRESHOLD + HYST;
  int thL = THRESHOLD - HYST;

  for (size_t i = 0; i < SENSOR_COUNT; ++i) {
    s.raw[i] = analogRead(SENSOR_PINS[i]);
    s.isBlack[i] = applyHysteresis(s.raw[i], lastBlack[i], thH, thL);
    lastBlack[i] = s.isBlack[i];
  }

  s.anyBlack = false;
  s.allBlack = true;
  for (size_t i = 0; i < SENSOR_COUNT; ++i) {
    if (s.isBlack[i]) s.anyBlack = true;
    else              s.allBlack = false;
  }
  s.allWhite = !s.anyBlack;

  s.blackDirState = computeBlackDirState(s);
  if (s.anyBlack) {
    lastDirState = s.blackDirState;
  }
  s.lastBlackDirState = lastDirState;

  return s;
}

float computeError(const Sense& s) {
  static float lastErr = 0.0f;

  auto norm = [](int v) -> float {
    extern float LINE_WHITE, LINE_BLACK;
    float x = (v - LINE_WHITE) / (LINE_BLACK - LINE_WHITE);
    if (x < 0.0f) x = 0.0f;
    if (x > 1.0f) x = 1.0f;
    return x;
  };

  // 機体が右にずれている → error正
  static const float weights[SENSOR_COUNT] = {1.0f, 0.5f, 0.0f, -0.5f, -1.0f};
  float sum = 0.0f;
  float weighted = 0.0f;

  for (size_t i = 0; i < SENSOR_COUNT; ++i) {
    float b = norm(s.raw[i]);
    sum += b;
    weighted += b * weights[i];
  }

  if (sum < LINE_EPS) {
    return lastErr;
  }

  float err = weighted / sum;
  lastErr = err;
  return err;
}

void debugPrintSensors(const Sense& s) {
  Serial.print(F("[Sensors]"));
  for (size_t i = 0; i < SENSOR_COUNT; ++i) {
    Serial.print(F(" "));
    Serial.print(s.raw[i]);
    Serial.print(s.isBlack[i] ? F("B") : F("W"));
  }
  Serial.print(F(" dir="));
  Serial.print(s.blackDirState);
  Serial.print(F(" last="));
  Serial.println(s.lastBlackDirState);
}
