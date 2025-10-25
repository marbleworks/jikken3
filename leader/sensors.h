#pragma once

#include <Arduino.h>

struct Sense {
  int rawL;
  int rawC;
  int rawR;
  int rawRL;
  int rawRR;
  bool isBlackL;
  bool isBlackC;
  bool isBlackR;
  bool isBlackRL;
  bool isBlackRR;
  bool anyBlack;
  bool allBlack;
  bool allWhite;
};

Sense readSensors();
int getBlackDirState(const Sense& s);
float computeError(int rawL, int rawC, int rawR);

