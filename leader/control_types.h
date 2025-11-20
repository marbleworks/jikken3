#pragma once

#include "run_mode.h"

struct PIDState {
  float integral;
  float lastError;
  unsigned long lastTimeMs;
};

struct TravelProfile {
  PIDState& pid;
  float& baseFiltered;
  float kp;
  float ki;
  float kd;
  int baseNominal;
  int baseMin;
  bool disableSteering;
};

struct FollowResult {
  bool lineLost;
};

struct BrakeZone {
  float start;
  float end;
  CurveLevel level;
};
