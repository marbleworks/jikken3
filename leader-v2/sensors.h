#pragma once
#include <Arduino.h>

inline constexpr size_t SENSOR_COUNT = 5;

struct Sense {
  int raw[SENSOR_COUNT];
  bool isBlack[SENSOR_COUNT];

  bool anyBlack;
  bool allBlack;
  bool allWhite;
  int blackDirState;      // -1=左寄り, 0=中央, +1=右寄り
  int lastBlackDirState;  // 最後に黒を検出したときのdirState
};

Sense readSensors();
float computeError(const Sense& s);
void debugPrintSensors(const Sense& s);
