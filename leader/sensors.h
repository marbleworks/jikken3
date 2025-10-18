#pragma once

#include <Arduino.h>

struct Sense {
  int rawL;
  int rawC;
  int rawR;
  bool isBlackL;
  bool isBlackC;
  bool isBlackR;
  bool anyBlack;
  bool allBlack;
  bool allWhite;
};

Sense readSensors();
float computeError(int rawL, int rawC, int rawR);
int getLastBlackDir();

