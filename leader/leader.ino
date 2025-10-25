#include "pins.h"
#include "sensor_leds.h"
#include "sensors.h"
#include "wheel_control.h"
#include "timer.h"
#include "run_mode.h"

// ------------------ チューニング用パラメータ ------------------
int   THRESHOLD      = 500;   // 白40 / 黒1000想定の中間。環境で調整
int   HYST           = 40;    // ヒステリシス
int   BASE_FWD       = 100;   // 前進の基準PWM
int   BASE_BACK      = 100;   // 後退の基準PWM
float KP_FWD         = 0.2f;  // 前進Pゲイン
float KP_BACK        = 0.1f;  // 後退Pゲイン
float KI_FWD         = 0.05f;  // 前進Iゲイン
float KI_BACK        = 0.05f;  // 後退Iゲイン
float KD_FWD         = 0.05f;  // 前進Dゲイン
float KD_BACK        = 0.05f;  // 後退Dゲイン
float PID_I_LIMIT    = 1.0f;  // I項アンチワインドアップ上限
float LINE_WHITE     = 40.0f;   // センサ白レベル
float LINE_BLACK     = 900.0f;  // センサ黒レベル
float LINE_EPS       = 1e-3f;   // 全白判定のしきい値
int   MAX_PWM        = 255;   // PWM上限
int   MIN_PWM        = 0;     // PWM下限
int   SEEK_SPEED     = 120;   // ライン探索速度（端点から黒を掴むまで）
unsigned long LOST_MS      = 100; // 見失い判定（FOLLOW中に全白がこの時間続いたらリカバリ）
unsigned int ENDPOINT_DONE_COUNT = 2; // 端点遭遇回数の上限 (0 で無効)
int   REC_STEER      = 128;    // リカバリ時の曲げ量（左右差）
int   UTURN_SPEED_LEFT  = 90;   // Uターン時の左輪PWM（正で前進）
int   UTURN_SPEED_RIGHT = -130;  // Uターン時の右輪PWM（正で前進）
unsigned long UTURN_TIME_MS = 900; // 180度回頭に掛ける時間（要調整）
unsigned long PRE_DONE_DURATION_MS = 200; // PRE_DONE時間（DONEの前に前進or後退）
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

unsigned int endpointCount = 0;

// 見失い管理
Timer lineLostTimer;
int lastBlackDirState = 0;           // -1=左, +1=右, 0=中央/不明
Timer uturnTimer;
Timer preDoneTimer;

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
  } else if (followState == FOLLOW_BACK) {
    resetPidState(pidBackward);
  }
}

void changeState(State newState,
                 const __FlashStringHelper* reason = nullptr) {
  if (state == newState) {
    return;
  }

  prevState = state;
  state = newState;
  resetPidForState(newState);

  if (reason) {
    Serial.print(reason);
    Serial.print(F(" -> "));
    Serial.println(stateLabel(newState));
  }
}

bool handleLineLostTimer(bool allWhite) {
  if (allWhite) {
    if (!lineLostTimer.running()) {
      lineLostTimer.start();
    }
    if (lineLostTimer.elapsed() > LOST_MS) {
      lineLostTimer.reset();
      return true;
    }
  } else if (lineLostTimer.running()) {
    lineLostTimer.reset();
  }

  return false;
}

bool handleUTurnTimer() {
  if (!uturnTimer.running()) {
    uturnTimer.start();
  }

  if (uturnTimer.elapsed() > UTURN_TIME_MS) {
    uturnTimer.reset();
    return true;
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

void handleUTurn() {
  if (!handleUTurnTimer()) {
    setWheels(UTURN_SPEED_LEFT, UTURN_SPEED_RIGHT);
    return;
  }

  changeState(SEEK_LINE_FWD, F("UTURN complete"));
}

void updateLastBlackDirState(const Sense& s) {
  if (s.anyBlack) {
    lastBlackDirState = getBlackDirState(s);
  }
}

bool handleSeekLine(State followState, int speedSign, const Sense& s) {
  int speed = speedSign * SEEK_SPEED;
  setWheels(speed, speed);
  bool found = s.anyBlack;
  if (found) {
    const __FlashStringHelper* reason =
      (followState == FOLLOW_FWD) ? F("Line found (forward)") : F("Line found (backward)");
    changeState(followState, reason);
  }

  return found;
}

FollowResult runLineTraceCommon(const Sense& s, PIDState& pid, int travelDir) {
  FollowResult res { false };

  if (handleLineLostTimer(s.allWhite)) {
    res.lineLost = true;
    resetPidState(pid);
    return res;
  }

  float e = computeError(s.rawL, s.rawC, s.rawR);
  float kp = (travelDir > 0) ? KP_FWD : KP_BACK;
  float ki = (travelDir > 0) ? KI_FWD : KI_BACK;
  float kd = (travelDir > 0) ? KD_FWD : KD_BACK;
  int base = (travelDir > 0) ? BASE_FWD : BASE_BACK;

  unsigned long now = millis();
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

bool handleRecover(const Sense& s, State followState, int basePwm, int travelDir) {
  bool recovered = recoverLine(s, basePwm, travelDir);
  if (recovered) {
    const __FlashStringHelper* reason =
      (followState == FOLLOW_FWD) ? F("Recovered (forward)") : F("Recovered (back)");
    changeState(followState, reason);
  }
  return recovered;
}

bool onEndpointEncountered() {
  ++endpointCount;
  if (ENDPOINT_DONE_COUNT > 0 && endpointCount >= ENDPOINT_DONE_COUNT) {
    if (prevState == FOLLOW_FWD) {
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
  updateLastBlackDirState(s);

  switch (state) {
    // 端点(全白)から前進して黒ラインを掴む
    case SEEK_LINE_FWD: {
      handleSeekLine(FOLLOW_FWD, +1, s);
      break;
    }

    // 前進でライントレース（P制御）
    case FOLLOW_FWD: {
      FollowResult r = runLineTraceCommon(s, pidForward, +1);
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
      FollowResult r = runLineTraceCommon(s, pidBackward, -1);
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
      handleUTurn();
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
