#include "pins.h"
#include "sensor_leds.h"
#include "sensors.h"
#include "wheel_control.h"
#include "timer.h"
#include "run_mode.h"

#include <math.h>

// センサーデバッグ出力を有効化する場合は 1 に設定する。
#ifndef SENSOR_DEBUG_PRINT
#define SENSOR_DEBUG_PRINT 0
#endif

// PID制御デバッグ出力を有効化する場合は 1 に設定する。
#ifndef PID_DEBUG_PRINT
#define PID_DEBUG_PRINT 0
#endif

// ------------------ チューニング用パラメータ ------------------
int   THRESHOLD      = 500;   // 白40 / 黒1000想定の中間。環境で調整
int   HYST           = 40;    // ヒステリシス
int   BASE_FWD       = 255;   // 前進の基準PWM
int   BASE_BACK      = 200;   // 後退の基準PWM
int   BASE_FWD_MIN   = 100;   // カーブ時に減速してもこの値以下にはしない
int   BASE_BACK_MIN  = 100;   // 後退時の最低PWM
float BASE_SPEED_ALPHA_ACCEL = 0.15f; // 直線復帰時の加速レスポンス
float BASE_SPEED_ALPHA_DECEL = 1.0f;  // カーブ時の減速レスポンス
float KP_FWD         = 0.35f;  // 前進Pゲイン
float KP_BACK        = 0.3f;  // 後退Pゲイン
float KI_FWD         = 0.005f;  // 前進Iゲイン
float KI_BACK        = 0.005f;  // 後退Iゲイン
float KD_FWD         = 0.031f;  // 前進Dゲイン
float KD_BACK        = 0.04f;  // 後退Dゲイン
float CURVE_E_GAIN   = 0.058f;   // 誤差に対する減速係数
float CURVE_E_EXP    = 1.5f;   // 誤差に対する減速の非線形指数（1で線形）
float CURVE_D_GAIN   = 1.0f;   // 変化量に対する減速係数
float CORR_EXP       = 1.5f;   // 補正量の非線形指数（1で線形）
float PID_I_LIMIT    = 1.0f;  // I項アンチワインドアップ上限
float LINE_WHITE     = 40.0f;   // センサ白レベル
float LINE_BLACK     = 900.0f;  // センサ黒レベル
float LINE_EPS       = 1e-3f;   // 全白判定のしきい値
int   MAX_PWM        = 255;   // PWM上限
int   MIN_PWM        = -1;     // PWM下限
int   SEEK_SPEED     = 120;   // ライン探索速度（端点から黒を掴むまで）
unsigned long SEEK_LINE_BACK_MIN_DURATION_MS = 500; // SEEK_LINE_BACKの最低継続時間
unsigned long LOST_MS_RECIP      = 100; // Reciprocalモードの見失い判定時間
unsigned long LOST_MS_UTURN      = 100; // UTurnモードの見失い判定時間
unsigned long LOST_MS_LOOP       = 0; // Loopモードの見失い判定時間
unsigned int ENDPOINT_DONE_COUNT = 2; // 端点遭遇回数の上限 (0 で無効)
int   REC_STEER      = 240;    // リカバリ時の曲げ量（左右差）
int   UTURN_SPEED_LEFT  = 70;   // Uターン時の左輪PWM（正で前進）
int   UTURN_SPEED_RIGHT = -150;  // Uターン時の右輪PWM（正で前進）
unsigned long PRE_DONE_DURATION_MS = 100; // PRE_DONE時間（DONEの前に前進or後退）

// 疑似距離と学習ラップ管理
const float LOOP_DISTANCE_MARGIN_BEFORE = 200.0f;   // カーブ検出からどれだけ手前を減速開始とみなすか
const float LOOP_DISTANCE_MARGIN_AFTER  = 120.0f;   // カーブ出口後に余韻として減速区間へ含める距離
const float LOOP_DISTANCE_MERGE_GAP     = 120.0f;   // 近接した減速区間を一つに結合する距離しきい値
const float LOOP_CURVE_ENTER_ERROR      = 0.65f;    // フロント偏差の絶対値がこの値以上で急カーブとして記録開始
const float LOOP_CURVE_EXIT_ERROR       = 0.35f;    // 偏差がこの値を下回ったら記録中のカーブを終了
const int   LOOP_CURVE_PWM_LIMIT        = 150;      // 先読み減速で制限する基準PWMの上限
const size_t LOOP_MAX_BRAKE_ZONES       = 8;        // 記録できる急カーブ区間の最大数
const unsigned long CROSS_WINDOW_MS     = 100;      // 十字線検出の観測ウィンドウ（全センサが黒判定されるまでの許容時間）
const unsigned long CROSS_COOLDOWN_MS   = 250;      // 十字線検出後に再検出を抑制するクールタイム
const unsigned long CROSS_PAIR_TIMEOUT_MS = 900;    // 1本目と2本目の十字線の最大間隔（ms）。超えるとノイズ扱い
const int   CROSS_THRESHOLD_OFFSET      = 70;       // 通常しきい値との差分。十字線検出用に黒判定を緩める量
const float CROSS_MAX_ERROR             = 0.55f;    // 走行偏差がこの値を超えているときは十字線とみなさない
// ----------------------------------------------------------------

// ====== struct をグローバルで定義 ======
struct FollowResult {
  bool lineLost;
};
// =================================================================

enum State {
  SEEK_LINE_FWD,     // 端点スタート（白）→黒ラインを探しながら前進
  FOLLOW_FWD,        // 前進でライン追従
  RECOVER_FWD,       // 前進中にライン見失い→自動復帰
  SEEK_LINE_BACK,    // 折り返し（後退で黒ライン再捕捉）
  FOLLOW_BACK,       // 後退でライン追従（復路）
  RECOVER_BACK,      // 後退中に見失い→自動復帰
  UTURN,             // Uターン中
  PRE_DONE,          // 完全停止前の慣性動作
  DONE               // 完了（停止）
};
State state = SEEK_LINE_FWD;
State prevState = SEEK_LINE_FWD;

struct PIDState {
  float integral;
  float lastError;
  unsigned long lastTimeMs;
};

PIDState pidForward{};
PIDState pidBackward{};

float baseForwardFiltered = BASE_FWD;
float baseBackwardFiltered = BASE_BACK;

struct TravelProfile {
  PIDState& pid;
  float& baseFiltered;
  float kp;
  float ki;
  float kd;
  int baseNominal;
  int baseMin;
  bool disableSteering;
};

TravelProfile makeTravelProfile(int travelDir, SensorMode mode) {
  if (travelDir > 0) {
    return { pidForward,
             baseForwardFiltered,
             KP_FWD,
             KI_FWD,
             KD_FWD,
             BASE_FWD,
             BASE_FWD_MIN,
             false };
  }

  bool disableSteering = (mode == SensorMode::Front5);
  return { pidBackward,
           baseBackwardFiltered,
           KP_BACK,
           KI_BACK,
           KD_BACK,
           BASE_BACK,
           BASE_BACK_MIN,
           disableSteering };
}

unsigned int endpointCount = 0;

struct BrakeZone {
  float start;
  float end;
};

BrakeZone brakeZones[LOOP_MAX_BRAKE_ZONES];
size_t brakeZoneCount = 0;
float pseudoDistance = 0.0f;
bool lapInitialized = false;
bool learningLapActive = false;
bool racingLapReady = false;
int lapCount = 0;
bool curveLearningActive = false;
float curveLearningStartDist = 0.0f;
unsigned long lastLooseBlackMs[Sense::MAX_FRONT_SENSORS] = {};
unsigned long lastCrossLineMs = 0;
bool waitingForSecondCross = false;
unsigned long firstCrossLineMs = 0;
float currentFrontError = 0.0f;
bool currentFrontErrorValid = false;

// 見失い管理
Timer lineLostTimer;
Timer preDoneTimer;
Timer seekLineBackTimer;

bool uturnReadyForBlack = false;

void resetBrakeZones() {
  brakeZoneCount = 0;
  curveLearningActive = false;
  curveLearningStartDist = 0.0f;
}

void addOrExtendBrakeZone(float start, float end) {
  if (start < 0.0f) {
    start = 0.0f;
  }
  if (end < start) {
    end = start;
  }

  if (brakeZoneCount > 0) {
    BrakeZone& last = brakeZones[brakeZoneCount - 1];
    if ((start - last.end) < LOOP_DISTANCE_MERGE_GAP) {
      last.start = min(last.start, start);
      last.end = max(last.end, end);
      return;
    }
  }

  if (brakeZoneCount < LOOP_MAX_BRAKE_ZONES) {
    brakeZones[brakeZoneCount].start = start;
    brakeZones[brakeZoneCount].end = end;
    ++brakeZoneCount;
  }
}

bool isInBrakeZone(float position) {
  for (size_t i = 0; i < brakeZoneCount; ++i) {
    if (position >= brakeZones[i].start && position <= brakeZones[i].end) {
      return true;
    }
  }
  return false;
}

int applyLocationBaseLimit(int baseNominal) {
  if (!lapInitialized || learningLapActive || !racingLapReady || brakeZoneCount == 0) {
    return baseNominal;
  }
  if (isInBrakeZone(pseudoDistance)) {
    return min(baseNominal, LOOP_CURVE_PWM_LIMIT);
  }
  return baseNominal;
}

void finalizeOpenCurveZone() {
  if (!curveLearningActive) {
    return;
  }
  float start = curveLearningStartDist;
  float end = pseudoDistance + LOOP_DISTANCE_MARGIN_AFTER;
  curveLearningActive = false;
  addOrExtendBrakeZone(start, end);
}

void updateCurveLearning(float error) {
  if (!lapInitialized || !learningLapActive) {
    return;
  }

  float ae = fabsf(error);
  if (!curveLearningActive) {
    if (ae >= LOOP_CURVE_ENTER_ERROR) {
      curveLearningActive = true;
      curveLearningStartDist = max(0.0f, pseudoDistance - LOOP_DISTANCE_MARGIN_BEFORE);
    }
    return;
  }

  if (ae < LOOP_CURVE_EXIT_ERROR) {
    finalizeOpenCurveZone();
  }
}

void updatePseudoDistance() {
  if (!lapInitialized || runMode != RUNMODE_LOOP) {
    if (!lapInitialized) {
      pseudoDistance = 0.0f;
    }
    return;
  }

  int left = getLastLeftCommand();
  int right = getLastRightCommand();
  if (left > 0 && right > 0) {
    float avg = (abs(left) + abs(right)) * 0.5f;
    pseudoDistance += avg;
  }
}

bool detectCrossLine(const Sense& s, unsigned long now) {
  if (s.frontCount == 0) {
    return false;
  }

  int looseThreshold = THRESHOLD - CROSS_THRESHOLD_OFFSET;
  if (looseThreshold < 0) {
    looseThreshold = 0;
  }

  for (size_t i = 0; i < s.frontCount; ++i) {
    if (s.rawFront[i] > looseThreshold) {
      lastLooseBlackMs[i] = now;
    }
  }
  for (size_t i = s.frontCount; i < Sense::MAX_FRONT_SENSORS; ++i) {
    lastLooseBlackMs[i] = now;
  }

  for (size_t i = 0; i < s.frontCount; ++i) {
    if (now - lastLooseBlackMs[i] > CROSS_WINDOW_MS) {
      return false;
    }
  }

  if (now - lastCrossLineMs < CROSS_COOLDOWN_MS) {
    return false;
  }
  if (!currentFrontErrorValid || fabsf(currentFrontError) > CROSS_MAX_ERROR) {
    return false;
  }

  lastCrossLineMs = now;
  return true;
}

void handleLapBoundary() {
  if (!lapInitialized) {
    lapInitialized = true;
    lapCount = 1;
    learningLapActive = true;
    racingLapReady = false;
    resetBrakeZones();
    pseudoDistance = 0.0f;
    Serial.println(F("Lap sync: learning lap start"));
    return;
  }

  if (learningLapActive) {
    finalizeOpenCurveZone();
    learningLapActive = false;
    racingLapReady = (brakeZoneCount > 0);
    ++lapCount;
    pseudoDistance = 0.0f;
    Serial.println(F("Lap sync: racing lap start"));
    return;
  }

  ++lapCount;
  pseudoDistance = 0.0f;
}

void updateLapDetection(const Sense& s) {
  if (runMode != RUNMODE_LOOP) {
    return;
  }

  if (state != FOLLOW_FWD) {
    waitingForSecondCross = false;
    return;
  }

  unsigned long now = millis();
  if (!detectCrossLine(s, now)) {
    if (waitingForSecondCross && (now - firstCrossLineMs > CROSS_PAIR_TIMEOUT_MS)) {
      waitingForSecondCross = false;
    }
    return;
  }

  if (!waitingForSecondCross) {
    waitingForSecondCross = true;
    firstCrossLineMs = now;
    return;
  }

  waitingForSecondCross = false;
  if (now - firstCrossLineMs <= CROSS_PAIR_TIMEOUT_MS) {
    handleLapBoundary();
  }
}

unsigned long getLostMsForMode(RunMode mode) {
  switch (mode) {
    case RUNMODE_RECIP: return LOST_MS_RECIP;
    case RUNMODE_UTURN: return LOST_MS_UTURN;
    case RUNMODE_LOOP:  return LOST_MS_LOOP;
    default:            return LOST_MS_RECIP;
  }
}

const __FlashStringHelper* stateLabel(State s) {
  switch (s) {
    case SEEK_LINE_FWD:    return F("SEEK_LINE_FWD");
    case FOLLOW_FWD:       return F("FOLLOW_FWD");
    case RECOVER_FWD:      return F("RECOVER_FWD");
    case SEEK_LINE_BACK:   return F("SEEK_LINE_BACK");
    case FOLLOW_BACK:      return F("FOLLOW_BACK");
    case RECOVER_BACK:     return F("RECOVER_BACK");
    case UTURN:            return F("UTURN");
    case PRE_DONE:         return F("PRE_DONE");
    case DONE:             return F("DONE");
    default:               return F("UNKNOWN");
  }
}

void resetPidState(PIDState& pid) {
  pid.integral = 0.0f;
  pid.lastError = 0.0f;
  pid.lastTimeMs = 0;
}

void resetPidForState(State followState) {
  if (followState == FOLLOW_FWD) {
    resetPidState(pidForward);
    baseForwardFiltered = BASE_FWD;
  } else if (followState == FOLLOW_BACK) {
    resetPidState(pidBackward);
    baseBackwardFiltered = BASE_BACK;
  }
}

void changeState(State newState,
                 const __FlashStringHelper* reason = nullptr) {
  if (state == newState) {
    return;
  }

  State oldState = state;
  prevState = state;
  state = newState;

  if (learningLapActive && oldState == FOLLOW_FWD && newState != FOLLOW_FWD) {
    finalizeOpenCurveZone();
  }

  if (oldState == SEEK_LINE_BACK && newState != SEEK_LINE_BACK) {
    seekLineBackTimer.reset();
  }
  if (state == UTURN) {
    uturnReadyForBlack = false;
  }
  resetPidForState(newState);
  updateSeekLineBackTimer(oldState, newState);

  if (state == SEEK_LINE_BACK) {
    seekLineBackTimer.reset();
    seekLineBackTimer.start();
  }

  if (reason) {
    Serial.print(reason);
    Serial.print(F(" -> "));
    Serial.println(stateLabel(newState));
  }
}

bool handleLineLostTimer(bool allWhite, unsigned long lostMs) {
  if (allWhite) {
    if (!lineLostTimer.running()) {
      lineLostTimer.start();
    }
    if (lineLostTimer.elapsed() > lostMs) {
      lineLostTimer.reset();
      return true;
    }
  } else if (lineLostTimer.running()) {
    lineLostTimer.reset();
  }

  return false;
}

bool handlePreDoneTimer() {
  if (!preDoneTimer.running()) {
    preDoneTimer.start();
  }

  if (preDoneTimer.elapsed() > PRE_DONE_DURATION_MS) {
    preDoneTimer.reset();
    return true;
  }

  return false;
}

void handleUTurn(const Sense& s) {
  bool anyBlackFront = getAnyBlack(s, SensorPosition::Front);

  if (!uturnReadyForBlack) {
    if (!anyBlackFront) {
      uturnReadyForBlack = true;
    }
  } else if (anyBlackFront) {
    changeState(SEEK_LINE_FWD, F("UTURN complete"));
    return;
  }

  setWheels(UTURN_SPEED_LEFT, UTURN_SPEED_RIGHT);
}

bool handleSeekLine(State followState, int speedSign, const Sense& s) {
  int speed = speedSign * SEEK_SPEED;
  setWheels(speed, speed);
  SensorPosition position = directionToSensorPosition(speedSign, s.mode);
  bool found = getAnyBlack(s, position);
  if (found) {
    bool canTransition = true;
    if (state == SEEK_LINE_BACK) {
      if (!seekLineBackTimer.running()) {
        seekLineBackTimer.start();
      }

      if (seekLineBackTimer.elapsed() < SEEK_LINE_BACK_MIN_DURATION_MS) {
        canTransition = false;
      }
    }

    if (canTransition) {
      const __FlashStringHelper* reason =
        (followState == FOLLOW_FWD) ? F("Line found (forward)") : F("Line found (backward)");
      changeState(followState, reason);
    }
  }

  return found;
}

FollowResult runLineTraceCommon(const Sense& s, int travelDir) {
  FollowResult res { false };

  SensorPosition position = directionToSensorPosition(travelDir, s.mode);
  bool allWhite = getAllWhite(s, position);

  TravelProfile profile = makeTravelProfile(travelDir, s.mode);

  if (handleLineLostTimer(allWhite, getLostMsForMode(runMode))) {
    res.lineLost = true;
    resetPidState(profile.pid);
    return res;
  }

  bool disableSteering = profile.disableSteering;

  float e = 0.0f;
  if (!disableSteering) {
    e = computeError(s, position);
    if (allWhite) {
      e = profile.pid.lastError;
    }
  }
  float kp = profile.kp;
  float ki = profile.ki;
  float kd = profile.kd;
  int baseNominal = profile.baseNominal;
  int baseMin = profile.baseMin;
  if (travelDir > 0) {
    baseNominal = applyLocationBaseLimit(baseNominal);
    if (baseNominal < baseMin) {
      baseNominal = baseMin;
    }
  }
  int base = baseNominal;
  float& baseFiltered = profile.baseFiltered;

  unsigned long now = millis();
  float dt = 0.0f;
  if (profile.pid.lastTimeMs != 0) {
    dt = (now - profile.pid.lastTimeMs) / 1000.0f;
  }
  profile.pid.lastTimeMs = now;

  float derivative = 0.0f;
  if (!disableSteering && !allWhite && dt > 0.0f) {
    profile.pid.integral += e * dt;
    profile.pid.integral = constrain(profile.pid.integral, -PID_I_LIMIT, PID_I_LIMIT);
    derivative = (e - profile.pid.lastError) / dt;
  } else if (disableSteering) {
    profile.pid.integral = 0.0f;
  }
  profile.pid.lastError = disableSteering ? 0.0f : e;

  bool errorValid = (!disableSteering && !allWhite);
  if (travelDir > 0) {
    if (errorValid) {
      currentFrontError = e;
      currentFrontErrorValid = true;
    } else {
      currentFrontErrorValid = false;
    }
    if (errorValid) {
      updateCurveLearning(e);
    }
  } else {
    currentFrontErrorValid = false;
  }

  if (!disableSteering) {
    float ae = fabsf(e);
    float ad = fabsf(derivative);
    float errorComponent = CURVE_E_GAIN * powf(ae * 100.0f, CURVE_E_EXP);
    int reduce = (int)(errorComponent + CURVE_D_GAIN * ad);
    base = constrain(baseNominal - reduce, baseMin, baseNominal);
  }

  float targetBase = base;
  float alpha = (targetBase < baseFiltered) ? BASE_SPEED_ALPHA_DECEL : BASE_SPEED_ALPHA_ACCEL;
  baseFiltered += alpha * (targetBase - baseFiltered);
  baseFiltered = constrain(baseFiltered, (float)baseMin, (float)baseNominal);
  base = (int)roundf(baseFiltered);

  float output = disableSteering ? 0.0f : (kp * e + ki * profile.pid.integral + kd * derivative);
  float corrNorm = constrain(output, -1.0f, 1.0f);
  float corrMagnitude = powf(fabsf(corrNorm), CORR_EXP);
  float corrScaled = copysignf(corrMagnitude, corrNorm);
  int corr = (int)(corrScaled * 255.0f);

  int dirSign    = (travelDir >= 0) ? 1 : -1;

  int left  = constrain(base + corr, MIN_PWM, MAX_PWM) * dirSign;
  int right = constrain(base - corr, MIN_PWM, MAX_PWM) * dirSign;
  setWheels(left, right);

#if PID_DEBUG_PRINT
  // PID制御関連のデバッグ出力
  Serial.print("BASE_FWD:");      Serial.print(BASE_FWD);      Serial.print("\t");
  Serial.print("BASE_FWD_MIN:");  Serial.print(BASE_FWD_MIN);  Serial.print("\t");
  Serial.print("targetBase:");   Serial.print(targetBase);   Serial.print("\t");
  Serial.print("baseFiltered:"); Serial.print(baseFiltered); Serial.print("\t");
  Serial.print("baseNominal:");  Serial.print(baseNominal);  Serial.print("\t");
  Serial.print("baseMin:");      Serial.print(baseMin);      Serial.print("\t");
  Serial.print("error:");        Serial.print(e);            Serial.print("\t");
  Serial.print("integral:");     Serial.print(profile.pid.integral); Serial.print("\t");
  Serial.print("derivative:");   Serial.print(derivative);   Serial.print("\t");
  Serial.print("corr:");         Serial.print(corr);         Serial.print("\t");
  Serial.print("left:");         Serial.print(left);         Serial.print("\t");
  Serial.print("right:");        Serial.print(right);
  Serial.println();
#endif

  return res;
}

void recoverLine(const Sense& s, int basePwm, int travelDir) {
  int dirSign = (travelDir >= 0) ? 1 : -1;

  if (travelDir < 0 && s.mode == SensorMode::Front5) {
    int speed = constrain(basePwm, MIN_PWM, MAX_PWM) * dirSign;
    setWheels(speed, speed);
    return;
  }

  int steerOffset;
  int lastDir = getLastBlackDirState(s, directionToSensorPosition(travelDir, s.mode));
  if (lastDir > 0) {
    steerOffset = REC_STEER;
  } else if (lastDir < 0) {
    steerOffset = -REC_STEER;
  } else {
    bool rightBias = (millis() / 300) % 2;
    steerOffset = rightBias ? REC_STEER : -REC_STEER;
  }

  int left = constrain(basePwm + steerOffset, MIN_PWM, MAX_PWM) * dirSign;
  int right = constrain(basePwm - steerOffset, MIN_PWM, MAX_PWM) * dirSign;

  setWheels(left, right);
}

bool handleRecover(const Sense& s, State followState, int basePwm, int travelDir) {
  SensorPosition position = directionToSensorPosition(travelDir, s.mode);
  bool recovered = getAnyBlack(s, position);
  if (recovered) {
    const __FlashStringHelper* reason =
      (followState == FOLLOW_FWD) ? F("Recovered (forward)") : F("Recovered (back)");
    changeState(followState, reason);
  }
  else {
    recoverLine(s, basePwm, travelDir);
  }
  
  return recovered;
}

bool onEndpointEncountered() {
  ++endpointCount;
  if (ENDPOINT_DONE_COUNT > 0 && endpointCount >= ENDPOINT_DONE_COUNT) {
    if (state == FOLLOW_FWD) {
      changeState(PRE_DONE, F("Endpoint limit reached"));
    }
    else {
      changeState(DONE, F("Endpoint limit reached"));
    }

    return true;
  }

  return false;
}

void handleForwardEndpoint() {
  if (onEndpointEncountered()) {
    return;
  }

  if (runMode == RUNMODE_UTURN) {
    changeState(UTURN, F("Endpoint (forward)"));
  } else {
    changeState(SEEK_LINE_BACK, F("Endpoint (forward)"));
  }
}

void handleBackwardEndpoint() {
  if (onEndpointEncountered()) {
    return;
  }

  changeState(SEEK_LINE_FWD, F("Endpoint (backward)"));
}

void handleForwardLineLost() {
  if (runMode == RUNMODE_LOOP) {
    changeState(RECOVER_FWD, F("Line lost (forward)"));
    return;
  }

  handleForwardEndpoint();
}

void handleBackwardLineLost() {
  if (runMode == RUNMODE_LOOP) {
    changeState(RECOVER_BACK, F("Line lost (backward)"));
    return;
  }

  handleBackwardEndpoint();
}

void handlePreDone() {
  if (!handlePreDoneTimer()) {
    if (prevState == FOLLOW_FWD) {
      setWheels(BASE_FWD, BASE_FWD);
    }

    return;
  }

  changeState(DONE, F("Pre-done complete"));
}

// ------------------ setup / loop ------------------
void setup() {
  Serial.begin(115200);
  runMode = COMPILE_TIME_RUNMODE;
  applyPotRunMode();
  setupWheelPins();
  pinMode(LED_WARN, OUTPUT);
  setupSensorLeds();
  Serial.print("Power-on (run mode: ");
  Serial.print(runModeLabel(runMode));
  Serial.println(") -> SEEK_LINE_FWD");
}

void loop() {
  Sense s = readSensors();
#if SENSOR_DEBUG_PRINT
  debugPrintSensors(s);
#endif
  currentFrontErrorValid = false;
  switch (state) {
    // 端点(全白)から前進して黒ラインを掴む
    case SEEK_LINE_FWD: {
      handleSeekLine(FOLLOW_FWD, +1, s);
      break;
    }

    // 前進でライントレース（P制御）
    case FOLLOW_FWD: {
      FollowResult r = runLineTraceCommon(s, +1);
      if (r.lineLost) {
        handleForwardLineLost();
        break;
      }
      break;
    }

    // 前進のリカバリ：最後に黒を見た側へ強めに切りながら再捕捉
    case RECOVER_FWD: {
      handleRecover(s, FOLLOW_FWD, BASE_FWD, +1);
      break;
    }

    // 折り返し：後退で黒ライン再捕捉
    case SEEK_LINE_BACK: {
      handleSeekLine(FOLLOW_BACK, -1, s);
      break;
    }

    // 後退でライントレース（P制御：進行方向が逆なので注意）
    case FOLLOW_BACK: {
      FollowResult r = runLineTraceCommon(s, -1);
      if (r.lineLost) {
        handleBackwardLineLost();
        break;
      }
      break;
    }

    // 後退のリカバリ：前進時と同じ“寄せ方向”を得るため，後進では前進と同じように左右の速度を計算した後、それぞれに -1 を乗算
    case RECOVER_BACK: {
      handleRecover(s, FOLLOW_BACK, BASE_BACK, -1);
      break;
    }

    case UTURN: {
      handleUTurn(s);
      break;
    }

    case PRE_DONE: {
      handlePreDone();
      break;
    }

    case DONE: {
      setWheels(0, 0); // 完全停止
      // 必要ならスリープやLED表示など
      break;
    }
  }

  updatePseudoDistance();
  updateLapDetection(s);

  delay(5); // ループ安定化
}
