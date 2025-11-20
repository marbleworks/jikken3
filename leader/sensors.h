#pragma once

#include <Arduino.h>

enum class SensorMode {
  Front5,
  Front3Rear2,
};

enum class SensorPosition {
  Front,
  Rear,
};

struct CrossLineParams {
  unsigned long windowMs;
  unsigned long cooldownMs;
  unsigned long pairTimeoutMs;
  int thresholdOffset;
  float maxError;
};

struct Sense {
  static constexpr size_t MAX_FRONT_SENSORS = 5;
  static constexpr size_t MAX_REAR_SENSORS = 2;

  SensorMode mode;
  size_t frontCount;
  size_t rearCount;

  int rawFront[MAX_FRONT_SENSORS];
  bool isBlackFront[MAX_FRONT_SENSORS];
  int rawRear[MAX_REAR_SENSORS];
  bool isBlackRear[MAX_REAR_SENSORS];

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
  int lastBlackStateFront;
  int lastBlackDirStateRear;
};

Sense readSensors();
int computeFrontBlackDirState(const Sense& s);
int computeRearBlackDirState(const Sense& s);
int getBlackDirState(const Sense& s, SensorPosition position);
int getLastBlackDirState(const Sense& s, SensorPosition position);
bool getAnyBlack(const Sense& s, SensorPosition position);
bool getAllBlack(const Sense& s, SensorPosition position);
bool getAllWhite(const Sense& s, SensorPosition position);
float computeError(const Sense& s, SensorPosition position);
SensorPosition directionToSensorPosition(int direction, SensorMode mode);
void debugPrintSensors(const Sense& s);
void resetCrossLineDetector();
// クロスライン（ゴールライン）検出
// 2本のラインをペアとして検出する
// outDurationMs: 2本のライン間の通過時間(ms)を格納するポインタ（省略可）
bool detectCrossLinePair(const Sense& s,
                         unsigned long now,
                         float currentError,
                         bool errorValid,
                         const CrossLineParams& params,
                         unsigned long* outDurationMs = nullptr);
