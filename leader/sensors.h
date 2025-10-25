#pragma once

#include <Arduino.h>

enum class SensorPosition {
  Front,
  Rear,
};

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
  int frontBlackDirState;
  int rearBlackDirState;
  SensorPosition lastBlackSensorPosition;
};

Sense readSensors();
int getFrontBlackDirState(const Sense& s);
int getRearBlackDirState(const Sense& s);
float computeError(int rawL, int rawC, int rawR);

