#pragma once

#include <Arduino.h>

enum RunMode {
  RUNMODE_RECIP,
  RUNMODE_UTURN,
  RUNMODE_LOOP
};

enum CurveLevel : uint8_t {
  CURVE_NONE = 0,
  CURVE_GENTLE = 1,
  CURVE_SHARP = 2
};

inline constexpr RunMode COMPILE_TIME_RUNMODE = RUNMODE_RECIP; // 直線コース向け既定値

extern RunMode runMode;

const char* runModeLabel(RunMode mode);

void applyPotRunMode();
