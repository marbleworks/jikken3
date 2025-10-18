#include "run_mode.h"

RunMode runMode = COMPILE_TIME_RUNMODE;

const char* runModeLabel(RunMode mode) {
  switch (mode) {
    case RUNMODE_RECIP: return "Reciprocal";
    case RUNMODE_UTURN: return "UTurn";
    case RUNMODE_LOOP:  return "Loop";
    default:            return "Unknown";
  }
}

