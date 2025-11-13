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
float CURVE_E_GAIN   = 0.059f;   // 誤差に対する減速係数
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
unsigned long UTURN_HOLD_DURATION_MS = 0; // Uターン後も継続させる時間
unsigned long PRE_DONE_DURATION_MS = 100; // PRE_DONE時間（DONEの前に前進or後退）
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

// ====== 周回コース記憶関連 ======
struct CourseMemoryEntry {
  int leftPwm;
  int rightPwm;
};

const size_t COURSE_MEMORY_CAPACITY = 3000;
const size_t MIN_LOOP_MEMORY_SAMPLES = 200;

CourseMemoryEntry courseMemory[COURSE_MEMORY_CAPACITY];
size_t courseMemoryLength = 0;
size_t courseMemoryPlaybackIndex = 0;
bool loopMemoryRecording = false;
bool loopMemoryReady = false;
bool loopMarkerLatched = false;
bool loopMarkerInitialized = false;
unsigned int loopLapCount = 0;
// ==================================

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

// 見失い管理
Timer lineLostTimer;
Timer preDoneTimer;
Timer seekLineBackTimer;
Timer uturnHoldTimer;

bool uturnReadyForBlack = false;

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

  if (oldState == SEEK_LINE_BACK && newState != SEEK_LINE_BACK) {
    seekLineBackTimer.reset();
  }
  if (state == UTURN) {
    uturnReadyForBlack = false;
    uturnHoldTimer.reset();
  }
  resetPidForState(newState);

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

bool shouldRecordCourseMemory(int travelDir) {
  return (runMode == RUNMODE_LOOP) && (travelDir > 0) &&
         (state == FOLLOW_FWD) && loopMemoryRecording &&
         (courseMemoryLength < COURSE_MEMORY_CAPACITY);
}

bool shouldPlaybackCourseMemory(int travelDir) {
  return (runMode == RUNMODE_LOOP) && (travelDir > 0) &&
         (state == FOLLOW_FWD) && loopMemoryReady &&
         (courseMemoryLength > 0);
}

void recordCourseMemorySample(int left, int right) {
  if (!loopMemoryRecording) {
    return;
  }
  if (courseMemoryLength >= COURSE_MEMORY_CAPACITY) {
    loopMemoryRecording = false;
    loopMemoryReady = (courseMemoryLength > 0);
    return;
  }
  courseMemory[courseMemoryLength++] = { left, right };
}

bool fetchCourseMemoryPlayback(int travelDir, CourseMemoryEntry& entry) {
  if (!shouldPlaybackCourseMemory(travelDir)) {
    return false;
  }
  entry = courseMemory[courseMemoryPlaybackIndex];
  courseMemoryPlaybackIndex = (courseMemoryPlaybackIndex + 1) % courseMemoryLength;
  return true;
}

void updateLoopCourseMemoryState(const Sense& s) {
  if (runMode != RUNMODE_LOOP || s.mode != SensorMode::Front5) {
    return;
  }

  bool onMarker = s.allBlackFront;
  if (onMarker) {
    if (!loopMarkerLatched) {
      loopMarkerLatched = true;
      if (!loopMemoryReady) {
        if (!loopMarkerInitialized) {
          loopMarkerInitialized = true;
          loopMemoryRecording = true;
          courseMemoryLength = 0;
          courseMemoryPlaybackIndex = 0;
          Serial.println(F("Loop memory: start recording first lap"));
        } else if (loopMemoryRecording && courseMemoryLength >= MIN_LOOP_MEMORY_SAMPLES) {
          loopMemoryRecording = false;
          loopMemoryReady = true;
          loopLapCount = 1;
          courseMemoryPlaybackIndex = 0;
          Serial.print(F("Loop memory recorded samples: "));
          Serial.println(courseMemoryLength);
        }
      } else {
        ++loopLapCount;
        courseMemoryPlaybackIndex = 0;
        Serial.print(F("Loop memory playback lap "));
        Serial.println(loopLapCount + 1);
      }
    }
  } else {
    loopMarkerLatched = false;
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
  if (uturnHoldTimer.running()) {
    if (uturnHoldTimer.elapsed() >= UTURN_HOLD_DURATION_MS) {
      uturnHoldTimer.reset();
      changeState(SEEK_LINE_FWD, F("UTURN complete"));
      return;
    }
    setWheels(UTURN_SPEED_LEFT, UTURN_SPEED_RIGHT);
    return;
  }

  bool anyBlackFront = getAnyBlack(s, SensorPosition::Front);

  if (!uturnReadyForBlack) {
    if (!anyBlackFront) {
      uturnReadyForBlack = true;
    }
  } else if (anyBlackFront) {
    uturnHoldTimer.start();
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
  bool playbackActive = shouldPlaybackCourseMemory(travelDir);

  if (handleLineLostTimer(allWhite, getLostMsForMode(runMode))) {
    res.lineLost = true;
    resetPidState(profile.pid);
    return res;
  }

  if (playbackActive) {
    CourseMemoryEntry playbackEntry{};
    if (fetchCourseMemoryPlayback(travelDir, playbackEntry)) {
      setWheels(playbackEntry.leftPwm, playbackEntry.rightPwm);
      return res;
    }
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

  if (shouldRecordCourseMemory(travelDir)) {
    recordCourseMemorySample(left, right);
  }

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
  updateLoopCourseMemoryState(s);
#if SENSOR_DEBUG_PRINT
  debugPrintSensors(s);
#endif
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

  delay(5); // ループ安定化
}
