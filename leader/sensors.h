#pragma once

#include <Arduino.h>

enum class SensorOrientation {
  Front,
  Back
};

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
  bool hasCenter;
  SensorOrientation orientation;
};

Sense readSensors(SensorOrientation orientation);
int getBlackDirState(const Sense& s);
float computeError(const Sense& s);

