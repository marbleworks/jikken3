#pragma once

enum RunMode {
  RUNMODE_RECIP,
  RUNMODE_UTURN,
  RUNMODE_LOOP
};

inline constexpr RunMode COMPILE_TIME_RUNMODE = RUNMODE_RECIP; // 直線コース向け既定値

extern RunMode runMode;

const char* runModeLabel(RunMode mode);

void applyPotRunMode();

void applyRunModeParameters();

