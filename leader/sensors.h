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
  bool anyBlackFront;
  bool allBlackFront;
  bool allWhiteFront;
  bool anyBlackRear;
  bool allBlackRear;
  bool allWhiteRear;
  bool anyBlack;
  bool allBlack;
  bool allWhite;
};

Sense readSensors();
int getBlackDirState(const Sense& s);
int getRearBlackDirState(const Sense& s);
float computeError(int rawL, int rawC, int rawR);

