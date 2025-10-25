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
int computeFrontBlackDirState(const Sense& s);
int computeRearBlackDirState(const Sense& s);
int getBlackDirState(const Sense& s, SensorPosition position);
float computeError(int rawL, int rawC, int rawR);

