#include "sensors.h"

#include "pins.h"
#include "run_mode.h"
#include "sensor_leds.h"

extern int THRESHOLD;
extern int HYST;
extern float LINE_WHITE;
extern float LINE_BLACK;
extern float LINE_EPS;

extern RunMode runMode;

namespace {

SensorMode determineSensorMode() {
  return (runMode == RUNMODE_LOOP) ? SensorMode::Front5 : SensorMode::Front3Rear2;
}

bool applyHysteresis(int raw, bool lastState, int thH, int thL) {
  return lastState ? (raw > thL) : (raw > thH);
}

template <size_t N>
void clearArray(bool (&arr)[N]) {
  for (size_t i = 0; i < N; ++i) {
    arr[i] = false;
  }
}

template <size_t N>
void clearArray(int (&arr)[N]) {
  for (size_t i = 0; i < N; ++i) {
    arr[i] = 0;
  }
}

}  // namespace

Sense readSensors() {
  Sense s{};
  s.mode = determineSensorMode();
  s.frontCount = (s.mode == SensorMode::Front5) ? 5 : 3;
  s.rearCount = (s.mode == SensorMode::Front5) ? 0 : 2;

  static bool lastFrontBlack[Sense::MAX_FRONT_SENSORS] = {};
  static bool lastRearBlack[Sense::MAX_REAR_SENSORS] = {};
  static SensorMode lastMode = SensorMode::Front3Rear2;
  static int lastFrontDirState = 0;
  static int lastRearDirState = 0;

  if (lastMode != s.mode) {
    clearArray(lastFrontBlack);
    clearArray(lastRearBlack);
    lastFrontDirState = 0;
    lastRearDirState = 0;
    lastMode = s.mode;
  }

  const uint8_t* frontPins = (s.mode == SensorMode::Front5) ? FRONT5_SENSOR_PINS : FRONT3_SENSOR_PINS;
  for (size_t i = 0; i < s.frontCount; ++i) {
    s.rawFront[i] = analogRead(frontPins[i]);
  }
  for (size_t i = s.frontCount; i < Sense::MAX_FRONT_SENSORS; ++i) {
    s.rawFront[i] = 0;
  }

  const uint8_t* rearPins = REAR2_SENSOR_PINS;
  for (size_t i = 0; i < s.rearCount; ++i) {
    s.rawRear[i] = analogRead(rearPins[i]);
  }
  for (size_t i = s.rearCount; i < Sense::MAX_REAR_SENSORS; ++i) {
    s.rawRear[i] = 0;
  }

  int thH = THRESHOLD + HYST;
  int thL = THRESHOLD - HYST;

  for (size_t i = 0; i < s.frontCount; ++i) {
    s.isBlackFront[i] = applyHysteresis(s.rawFront[i], lastFrontBlack[i], thH, thL);
    lastFrontBlack[i] = s.isBlackFront[i];
  }
  for (size_t i = s.frontCount; i < Sense::MAX_FRONT_SENSORS; ++i) {
    s.isBlackFront[i] = false;
    lastFrontBlack[i] = false;
  }

  for (size_t i = 0; i < s.rearCount; ++i) {
    s.isBlackRear[i] = applyHysteresis(s.rawRear[i], lastRearBlack[i], thH, thL);
    lastRearBlack[i] = s.isBlackRear[i];
  }
  for (size_t i = s.rearCount; i < Sense::MAX_REAR_SENSORS; ++i) {
    s.isBlackRear[i] = false;
    lastRearBlack[i] = false;
  }

  s.anyBlackFront = false;
  s.allBlackFront = (s.frontCount > 0);
  for (size_t i = 0; i < s.frontCount; ++i) {
    if (s.isBlackFront[i]) {
      s.anyBlackFront = true;
    } else {
      s.allBlackFront = false;
    }
  }
  s.allWhiteFront = !s.anyBlackFront;

  if (s.rearCount == 0) {
    s.anyBlackRear = false;
    s.allBlackRear = false;
    s.allWhiteRear = true;
  } else {
    s.anyBlackRear = false;
    s.allBlackRear = true;
    for (size_t i = 0; i < s.rearCount; ++i) {
      if (s.isBlackRear[i]) {
        s.anyBlackRear = true;
      } else {
        s.allBlackRear = false;
      }
    }
    s.allWhiteRear = !s.anyBlackRear;
  }

  s.anyBlack = s.anyBlackFront || s.anyBlackRear;
  bool rearAllBlackForAll = (s.rearCount == 0) ? true : s.allBlackRear;
  bool rearAllWhiteForAll = (s.rearCount == 0) ? true : s.allWhiteRear;
  s.allBlack = s.allBlackFront && rearAllBlackForAll;
  s.allWhite = s.allWhiteFront && rearAllWhiteForAll;

  s.frontBlackDirState = computeFrontBlackDirState(s);
  s.rearBlackDirState = computeRearBlackDirState(s);
  if (s.anyBlackFront) {
    lastFrontDirState = s.frontBlackDirState;
  }
  if (s.rearCount > 0 && s.anyBlackRear) {
    lastRearDirState = s.rearBlackDirState;
  } else if (s.rearCount == 0) {
    lastRearDirState = 0;
  }
  s.lastBlackStateFront = lastFrontDirState;
  s.lastBlackDirStateRear = lastRearDirState;

  if (s.mode == SensorMode::Front5) {
    bool f1 = (s.frontCount > 0) ? s.isBlackFront[0] : false;
    bool f2 = (s.frontCount > 1) ? s.isBlackFront[1] : false;
    bool f3 = (s.frontCount > 2) ? s.isBlackFront[2] : false;
    bool f4 = (s.frontCount > 3) ? s.isBlackFront[3] : false;
    bool f5 = (s.frontCount > 4) ? s.isBlackFront[4] : false;
    displaySensorStates(f1, f3, f5, f2, f4);
  } else {
    bool frontL = (s.frontCount > 0) ? s.isBlackFront[0] : false;
    bool frontC = (s.frontCount > 1) ? s.isBlackFront[1] : false;
    bool frontR = (s.frontCount > 2) ? s.isBlackFront[2] : false;
    bool rearL = (s.rearCount > 0) ? s.isBlackRear[0] : false;
    bool rearR = (s.rearCount > 1) ? s.isBlackRear[1] : false;
    displaySensorStates(frontL, frontC, frontR, rearL, rearR);
  }

  return s;
}

void debugPrintSensors(const Sense& s) {
  Serial.print(F("["));
  Serial.print((s.mode == SensorMode::Front5) ? F("Front5") : F("Front3Rear2"));
  Serial.print(F("] front:"));

  for (size_t i = 0; i < s.frontCount; ++i) {
    Serial.print(F(" "));
    Serial.print(s.rawFront[i]);
    Serial.print(s.isBlackFront[i] ? F("B") : F("W"));
  }

  Serial.print(F(" rear:"));
  if (s.rearCount == 0) {
    Serial.print(F(" (none)"));
  } else {
    for (size_t i = 0; i < s.rearCount; ++i) {
      Serial.print(F(" "));
      Serial.print(s.rawRear[i]);
      Serial.print(s.isBlackRear[i] ? F("B") : F("W"));
    }
  }

  Serial.print(F(" any="));
  Serial.print(s.anyBlack ? F("B") : F("W"));
  Serial.print(F(" lastF="));
  Serial.print(s.lastBlackStateFront);
  Serial.print(F(" lastR="));
  Serial.println(s.lastBlackDirStateRear);
}

int computeFrontBlackDirState(const Sense& s) {
  if (s.frontCount == 0) {
    return 0;
  }

  bool left = false;
  bool right = false;
  bool center = false;
  size_t mid = s.frontCount / 2;

  for (size_t i = 0; i < s.frontCount; ++i) {
    if (!s.isBlackFront[i]) {
      continue;
    }
    if (i < mid) {
      left = true;
    } else if (i > mid) {
      right = true;
    } else {
      center = true;
    }
  }

  if (left && !right) {
    return -1;
  }
  if (right && !left) {
    return +1;
  }
  if (center || (left && right)) {
    return 0;
  }
  return 0;
}

int computeRearBlackDirState(const Sense& s) {
  if (s.rearCount == 0) {
    return 0;
  }

  bool left = (s.rearCount > 0) ? s.isBlackRear[0] : false;
  bool right = (s.rearCount > 1) ? s.isBlackRear[1] : false;

  if (left && !right) {
    return -1;
  }
  if (right && !left) {
    return +1;
  }
  if ((left && right) || (!left && !right)) {
    return 0;
  }
  return 0;
}

int getBlackDirState(const Sense& s, SensorPosition position) {
  switch (position) {
    case SensorPosition::Front:
      return s.frontBlackDirState;
    case SensorPosition::Rear:
      return s.rearBlackDirState;
  }
  return 0;
}

int getLastBlackDirState(const Sense& s, SensorPosition position) {
  switch (position) {
    case SensorPosition::Front:
      return s.lastBlackStateFront;
    case SensorPosition::Rear:
      return s.lastBlackDirStateRear;
  }
  return 0;
}

bool getAnyBlack(const Sense& s, SensorPosition position) {
  switch (position) {
    case SensorPosition::Front:
      for (size_t i = 0; i < s.frontCount; ++i) {
        if (s.isBlackFront[i]) {
          return true;
        }
      }
      return false;
    case SensorPosition::Rear:
      for (size_t i = 0; i < s.rearCount; ++i) {
        if (s.isBlackRear[i]) {
          return true;
        }
      }
      return false;
  }
  return false;
}

bool getAllBlack(const Sense& s, SensorPosition position) {
  switch (position) {
    case SensorPosition::Front:
      return s.allBlackFront;
    case SensorPosition::Rear:
      if (s.rearCount == 0) {
        return false;
      }
      return s.allBlackRear;
  }
  return false;
}

bool getAllWhite(const Sense& s, SensorPosition position) {
  switch (position) {
    case SensorPosition::Front:
      return s.allWhiteFront;
    case SensorPosition::Rear:
      if (s.rearCount == 0) {
        return true;
      }
      return s.allWhiteRear;
  }
  return false;
}

float computeError(const Sense& s, SensorPosition position) {
  auto norm = [&](int v) -> float {
    float x = (v - LINE_WHITE) / (LINE_BLACK - LINE_WHITE);
    if (x < 0.0f) x = 0.0f;
    if (x > 1.0f) x = 1.0f;
    return x;
  };

  static float lastFrontErr = 0.0f;
  static float lastRearErr = 0.0f;

  if (position == SensorPosition::Front) {
    if (s.frontCount == 0) {
      return lastFrontErr;
    }

    float weights3[3] = {-1.0f, 0.0f, 1.0f};
    float weights5[5] = {-2.0f, -1.0f, 0.0f, 1.0f, 2.0f};

    const float* weights = (s.frontCount == 5) ? weights5 : weights3;
    float sum = 0.0f;
    float weighted = 0.0f;
    for (size_t i = 0; i < s.frontCount; ++i) {
      float b = norm(s.rawFront[i]);
      sum += b;
      weighted += b * weights[i];
    }
    if (sum < LINE_EPS) {
      return lastFrontErr;
    }
    float err = weighted / sum;
    lastFrontErr = err;
    return err;
  }

  if (s.rearCount == 0) {
    lastRearErr = 0.0f;
    return lastRearErr;
  }

  float sum = 0.0f;
  float weighted = 0.0f;
  static const float weightsRear[Sense::MAX_REAR_SENSORS] = {-1.0f, 1.0f};
  for (size_t i = 0; i < s.rearCount; ++i) {
    float b = norm(s.rawRear[i]);
    sum += b;
    weighted += b * weightsRear[i];
  }
  if (sum < LINE_EPS) {
    return lastRearErr;
  }
  float err = weighted / sum;
  lastRearErr = err;
  return err;
}

SensorPosition directionToSensorPosition(int direction, SensorMode mode) {
  if (direction >= 0) {
    return SensorPosition::Front;
  }
  if (mode == SensorMode::Front5) {
    return SensorPosition::Front;
  }
  return SensorPosition::Rear;
}
