#include "pins.h"
#include "sensor_leds.h"
#include "sensors.h"
#include "wheel_control.h"
#include "timer.h"
#include "run_mode.h"

// ------------------ チューニング用パラメータ ------------------
// センサ・制御共通
int   THRESHOLD      = 500;   // 白40 / 黒1000想定の中間。環境で調整
int   HYST           = 40;    // ヒステリシス
float LINE_WHITE     = 40.0f;   // センサ白レベル
float LINE_BLACK     = 900.0f;  // センサ黒レベル
float LINE_EPS       = 1e-3f;   // 全白判定のしきい値

// PID / 走行パラメータ
int   BASE_FWD       = 100;   // 前進の基準PWM
int   BASE_BACK      = 100;   // 後退の基準PWM
float KP_FWD         = 0.2f;  // 前進Pゲイン
float KP_BACK        = 0.2f;  // 後退Pゲイン
float KI_FWD         = 0.0f;  // 前進Iゲイン
float KI_BACK        = 0.0f;  // 後退Iゲイン
float KD_FWD         = 0.0f;  // 前進Dゲイン
float KD_BACK        = 0.0f;  // 後退Dゲイン
float PID_I_LIMIT    = 1.0f;  // I項アンチワインドアップ上限
int   MAX_PWM        = 255;   // PWM上限
int   MIN_PWM        = 0;     // PWM下限
int   SEEK_SPEED     = 120;   // ライン探索速度（端点から黒を掴むまで）

// 端点検出（全白判定）
unsigned long ENDPOINT_BLACK_HOLD_MS = 200;   // 直前に連続して黒を検出していた最小時間
unsigned long ENDPOINT_BLACK_ACC_MIN = 10000; // 直前の黒強度の累積量（max(raw) - THRESHOLD の積算）
unsigned long ENDPOINT_WHITE_HOLD_MS = 800;   // 端点として確定するために全白が続く時間

// ラインロスト検出
unsigned long LOST_RECENT_BLACK_MS = 250; // 直前の黒検出を有効とみなす最大経過時間
unsigned long LOST_WHITE_HOLD_MS   = 300; // ラインロスト確定までの全白継続時間
int   LOST_DROP_THRESHOLD          = 250; // 直前の黒ピークとの差分がこの値以上で急減と判定
int   LOST_MIN_BLACK_LEVEL         = 650; // ドロップ判定に使う直前黒ピークの下限

// リカバリ / Uターン
int   REC_STEER      = 128;    // リカバリ時の曲げ量（左右差）
int   UTURN_SPEED    = 150;   // 片輪前進・片輪後退のPWM
unsigned long UTURN_TIME_MS = 3000; // 180度回頭に掛ける時間（要調整）
// ----------------------------------------------------------------

// ====== struct をグローバルで定義 ======
struct FollowResult {
  bool lineLost;
  bool endpoint;
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
  DONE               // 完了（停止）
};
State state = SEEK_LINE_FWD;

struct PIDState {
  float integral;
  float lastError;
  unsigned long lastTimeMs;
};

PIDState pidForward{};
PIDState pidBackward{};

unsigned long lapCount = 0; // RUNMODE_LOOP で端点を通過した回数

// 端点検出・見失い管理
Timer endpointTimer;                 // 全白継続時間（端点専用）
Timer lostWhiteTimer;                // ラインロスト用の全白継続時間
int lastBlackDirState = 0;           // -1=左, +1=右, 0=中央/不明
Timer uturnTimer;

struct EndpointDetectionState {
  Timer blackTimer;
  unsigned long blackAccum;
  unsigned long lastBlackDuration;
  unsigned long lastBlackAccum;
  bool blackActive;
  bool candidateArmed;
} endpointState;

struct LineLossDetectionState {
  int lastDarkPeak;
  unsigned long lastBlackMs;
  bool dropArmed;
  int dropDir;
  int lastDropDelta;
} lineLossState;

typedef void (*EndpointHandler)(const char* context);

void handleUTurn() {
  if (!uturnTimer.running()) {
    uturnTimer.start();
  }

  unsigned long elapsed = uturnTimer.elapsed();
  if (elapsed < UTURN_TIME_MS) {
    setWheels(UTURN_SPEED, -UTURN_SPEED);
    return;
  }

  setWheels(0, 0);
  state = SEEK_LINE_FWD;
  resetLineLossDetection();
  resetEndpointDetection();
  uturnTimer.reset();
  Serial.println(F("UTURN complete -> SEEK_LINE_FWD"));
}

void updateLastBlackDirState(const Sense& s) {
  if (s.anyBlack) {
    lastBlackDirState = getBlackDirState(s);
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
  } else if (followState == FOLLOW_BACK) {
    resetPidState(pidBackward);
  }
}

void resetPidForDir(int travelDir) {
  if (travelDir >= 0) {
    resetPidState(pidForward);
  } else {
    resetPidState(pidBackward);
  }
}

void resetEndpointDetection() {
  endpointState.blackTimer.reset();
  endpointState.blackAccum = 0;
  endpointState.lastBlackDuration = 0;
  endpointState.lastBlackAccum = 0;
  endpointState.blackActive = false;
  endpointState.candidateArmed = false;
  endpointTimer.reset();
}

void resetLineLossDetection() {
  lostWhiteTimer.reset();
  lineLossState.lastDarkPeak = 0;
  lineLossState.lastBlackMs = 0;
  lineLossState.dropArmed = false;
  lineLossState.dropDir = 0;
  lineLossState.lastDropDelta = 0;
}

int maxSenseRaw(const Sense& s) {
  return max(s.rawL, max(s.rawC, s.rawR));
}

void handleSeekLine(State followState, int speedSign, const __FlashStringHelper* logMsg, const Sense& s) {
  int speed = speedSign * SEEK_SPEED;
  setWheels(speed, speed);
  if (s.anyBlack) {
    state = followState;
    resetPidForState(followState);
    resetLineLossDetection();
    resetEndpointDetection();
    Serial.println(logMsg);
  }
}

bool checkEndpointReached(const Sense& s, const __FlashStringHelper* phaseLabel) {
  if (s.anyBlack) {
    if (!endpointState.blackActive) {
      endpointState.blackActive = true;
      endpointState.blackTimer.start();
      endpointState.blackAccum = 0;
    }
    int m = maxSenseRaw(s);
    if (m > THRESHOLD) {
      endpointState.blackAccum += (unsigned long)(m - THRESHOLD);
    }
    endpointState.candidateArmed = false;
    endpointTimer.reset();
    return false;
  }

  if (endpointState.blackActive) {
    endpointState.lastBlackDuration = endpointState.blackTimer.elapsed();
    endpointState.lastBlackAccum = endpointState.blackAccum;
    endpointState.blackTimer.reset();
    endpointState.blackActive = false;
    endpointState.candidateArmed =
      endpointState.lastBlackDuration >= ENDPOINT_BLACK_HOLD_MS &&
      endpointState.lastBlackAccum   >= ENDPOINT_BLACK_ACC_MIN;
    if (!endpointState.candidateArmed) {
      if (endpointState.lastBlackDuration >= ENDPOINT_BLACK_HOLD_MS / 2 ||
          endpointState.lastBlackAccum >= ENDPOINT_BLACK_ACC_MIN / 2) {
        Serial.print(F("Endpoint guard reset ("));
        Serial.print(phaseLabel);
        Serial.print(F(") duration="));
        Serial.print(endpointState.lastBlackDuration);
        Serial.print(F("ms, accum="));
        Serial.print(endpointState.lastBlackAccum);
        Serial.println(F(")"));
      }
    }
  }

  if (!endpointState.candidateArmed) {
    endpointTimer.reset();
    return false;
  }

  if (!endpointTimer.running()) {
    endpointTimer.start();
  }

  if (endpointTimer.elapsed() >= ENDPOINT_WHITE_HOLD_MS) {
    Serial.print(F("Endpoint detected ("));
    Serial.print(phaseLabel);
    Serial.print(F(") white="));
    Serial.print(endpointTimer.elapsed());
    Serial.print(F("ms, pre-black="));
    Serial.print(endpointState.lastBlackDuration);
    Serial.print(F("ms, accum="));
    Serial.print(endpointState.lastBlackAccum);
    Serial.println(F(")"));
    resetEndpointDetection();
    return true;
  }

  return false;
}

bool checkLineLost(const Sense& s,
                   unsigned long now,
                   const __FlashStringHelper* phaseLabel,
                   bool allowTrigger) {
  int maxRaw = maxSenseRaw(s);

  if (s.anyBlack) {
    lineLossState.lastDarkPeak = maxRaw;
    lineLossState.lastBlackMs = now;
    lineLossState.dropArmed = false;
    lineLossState.lastDropDelta = 0;
    lostWhiteTimer.reset();
    return false;
  }

  if (lineLossState.lastBlackMs == 0 || now - lineLossState.lastBlackMs > LOST_RECENT_BLACK_MS) {
    lineLossState.dropArmed = false;
    lineLossState.lastDarkPeak = 0;
    lostWhiteTimer.reset();
    return false;
  }

  int dropDelta = lineLossState.lastDarkPeak - maxRaw;
  if (!lineLossState.dropArmed) {
    if (lineLossState.lastDarkPeak >= LOST_MIN_BLACK_LEVEL && dropDelta >= LOST_DROP_THRESHOLD) {
      lineLossState.dropArmed = true;
      lineLossState.dropDir = lastBlackDirState;
      lineLossState.lastDropDelta = dropDelta;
      Serial.print(F("Line drop ("));
      Serial.print(phaseLabel);
      Serial.print(F(") dir="));
      Serial.print(lineLossState.dropDir);
      Serial.print(F(", peak="));
      Serial.print(lineLossState.lastDarkPeak);
      Serial.print(F(", delta="));
      Serial.print(lineLossState.lastDropDelta);
      Serial.println(F(")"));
    } else {
      lostWhiteTimer.reset();
      return false;
    }
  }

  if (!lostWhiteTimer.running()) {
    lostWhiteTimer.start();
  }

  if (lostWhiteTimer.elapsed() >= LOST_WHITE_HOLD_MS) {
    Serial.print(F("Line lost via white hold ("));
    Serial.print(phaseLabel);
    Serial.print(F(") dir="));
    Serial.print(lineLossState.dropDir);
    Serial.print(F(", white="));
    Serial.print(lostWhiteTimer.elapsed());
    Serial.print(F("ms, delta="));
    Serial.print(lineLossState.lastDropDelta);
    Serial.println(F(")"));
    resetLineLossDetection();
    return allowTrigger;
  }

  return false;
}

FollowResult runLineTraceCommon(const Sense& s, int travelDir) {
  FollowResult res { false, false };

  unsigned long now = millis();
  if (checkLineLost(s, now, F("FOLLOW"), true)) {
    res.lineLost = true;
    resetPidForDir(travelDir);
    return res;
  }

  float e = computeError(s.rawL, s.rawC, s.rawR);
  PIDState& pid = (travelDir > 0) ? pidForward : pidBackward;
  float kp = (travelDir > 0) ? KP_FWD : KP_BACK;
  float ki = (travelDir > 0) ? KI_FWD : KI_BACK;
  float kd = (travelDir > 0) ? KD_FWD : KD_BACK;
  int base = (travelDir > 0) ? BASE_FWD : BASE_BACK;

  float dt = 0.0f;
  if (pid.lastTimeMs != 0) {
    dt = (now - pid.lastTimeMs) / 1000.0f;
  }
  pid.lastTimeMs = now;

  float derivative = 0.0f;
  if (!s.allWhite && dt > 0.0f) {
    pid.integral += e * dt;
    pid.integral = constrain(pid.integral, -PID_I_LIMIT, PID_I_LIMIT);
    derivative = (e - pid.lastError) / dt;
  }
  pid.lastError = e;

  float output = kp * e + ki * pid.integral + kd * derivative;
  int corr = (int)(output * 255.0f);

  int dirSign    = (travelDir >= 0) ? 1 : -1;

  int left  = constrain(base + corr, MIN_PWM, MAX_PWM) * dirSign;
  int right = constrain(base - corr, MIN_PWM, MAX_PWM) * dirSign;
  setWheels(left, right);

  res.endpoint = checkEndpointReached(s, F("FOLLOW"));
  return res;
}

bool recoverLine(const Sense& s, int basePwm, int travelDir) {
  int dirSign = (travelDir >= 0) ? 1 : -1;

  int steerOffset;
  int lastDir = lastBlackDirState;
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

  return s.anyBlack;
}

void handleRecover(const Sense& s,
                   State followState,
                   int basePwm,
                   int travelDir,
                   const __FlashStringHelper* logMsg,
                   EndpointHandler endpointHandler,
                   const char* endpointContext,
                   bool enableEndpointHandling) {
  unsigned long now = millis();
  checkLineLost(s, now, F("RECOVER"), false);

  bool recovered = recoverLine(s, basePwm, travelDir);
  if (recovered) {
    resetLineLossDetection();
    state = followState;
    resetPidForState(followState);
    Serial.println(logMsg);
  }

  if (enableEndpointHandling && endpointHandler != nullptr && checkEndpointReached(s, F("RECOVER"))) {
    endpointHandler(endpointContext);
  }
}

void handleForwardEndpoint(const char* context) {
  setWheels(0, 0);
  delay(150);
  resetLineLossDetection();
  resetEndpointDetection();

  if (runMode == RUNMODE_RECIP) {
    state = SEEK_LINE_BACK;
    Serial.print("Endpoint ");
    Serial.print(context);
    Serial.println(" -> SEEK_LINE_BACK");
  } else if (runMode == RUNMODE_LOOP) {
    ++lapCount;
    state = SEEK_LINE_FWD;
    Serial.print("Endpoint ");
    Serial.print(context);
    Serial.print(" -> continuing loop (lap ");
    Serial.print(lapCount);
    Serial.println(")");
  } else if (runMode == RUNMODE_UTURN) {
    Serial.print("Endpoint ");
    Serial.print(context);
    Serial.println(" -> UTURN");

    uturnTimer.start();
    state = UTURN;
  }
}

void finishReciprocalReturn(const char* context) {
  setWheels(0, 0);
  Serial.print(context);
  Serial.println(" DONE.");
  resetLineLossDetection();
  resetEndpointDetection();
  state = DONE;
}

// ------------------ setup / loop ------------------
void setup() {
  Serial.begin(115200);
  runMode = COMPILE_TIME_RUNMODE;
  applyPotRunMode();
  setupWheelPins();
  setupSensorLeds();
  resetLineLossDetection();
  resetEndpointDetection();
  Serial.print("Power-on (run mode: ");
  Serial.print(runModeLabel(runMode));
  Serial.println(") -> SEEK_LINE_FWD");
}

void loop() {
  Sense s = readSensors();
  updateLastBlackDirState(s);
  Serial.println(state);
  // setWheels(128,0);

  switch (state) {
    // 端点(全白)から前進して黒ラインを掴む
    case SEEK_LINE_FWD: {
      handleSeekLine(FOLLOW_FWD, +1, F("-> FOLLOW_FWD"), s);
      break;
    }

    // 前進でライントレース（P制御）
    case FOLLOW_FWD: {
      FollowResult r = runLineTraceCommon(s, +1);
      if (r.lineLost) {
        state = RECOVER_FWD;
        break;
      }
      if (r.endpoint) {
        handleForwardEndpoint("reached (forward)");
      }
      break;
    }

    // 前進のリカバリ：最後に黒を見た側へ強めに切りながら再捕捉
    case RECOVER_FWD: {
      handleRecover(s,
                    FOLLOW_FWD,
                    BASE_FWD,
                    +1,
                    F("Recovered (forward) -> FOLLOW_FWD"),
                    handleForwardEndpoint,
                    "(recover fwd)",
                    true);
      break;
    }

    // 折り返し：後退で黒ライン再捕捉
    case SEEK_LINE_BACK: {
      handleSeekLine(FOLLOW_BACK, -1, F("-> FOLLOW_BACK"), s);
      break;
    }

    // 後退でライントレース（P制御：進行方向が逆なので注意）
    case FOLLOW_BACK: {
      FollowResult r = runLineTraceCommon(s, -1);
      if (r.lineLost) {
        state = RECOVER_BACK;
        break;
      }
      if (r.endpoint && runMode == RUNMODE_RECIP) {
        finishReciprocalReturn("Back to start.");
      }
      break;
    }

    // 後退のリカバリ：前進時と同じ“寄せ方向”を得るため，後進では前進と同じように左右の速度を計算した後、それぞれに -1 を乗算
    case RECOVER_BACK: {
      handleRecover(s,
                    FOLLOW_BACK,
                    BASE_BACK,
                    -1,
                    F("Recovered (back) -> FOLLOW_BACK"),
                    finishReciprocalReturn,
                    "Back to start.",
                    runMode == RUNMODE_RECIP);
      break;
    }

    case UTURN: {
      handleUTurn();
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
