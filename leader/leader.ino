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
unsigned int ENDPOINT_DONE_COUNT = 0; // 端点遭遇回数の上限 (0 で無効)
int   REC_STEER      = 128;    // リカバリ時の曲げ量（左右差）
int   UTURN_SPEED    = 150;   // 片輪前進・片輪後退のPWM
unsigned long UTURN_TIME_MS = 3000; // 180度回頭に掛ける時間（要調整）
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

unsigned int endpointCount = 0;

// 見失い管理
Timer lostTimer;
int lastBlackDirState = 0;           // -1=左, +1=右, 0=中央/不明
Timer uturnTimer;

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
  lostTimer.reset();
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

bool handleSeekLine(State followState, int speedSign, const Sense& s) {
  int speed = speedSign * SEEK_SPEED;
  setWheels(speed, speed);
  if (s.anyBlack) {
    state = followState;
    resetPidForState(followState);
    lostTimer.reset();
    return true;
  }
  return false;
}
FollowResult runLineTraceCommon(const Sense& s, int travelDir) {
  FollowResult res { false };

  if (s.allWhite) {
    if (!lostTimer.running()) {
      lostTimer.start();
    }
    if (lostTimer.elapsed() > LOST_MS) {
      res.lineLost = true;
      resetPidForDir(travelDir);
      return res;
    }
  } else {
    lostTimer.reset();
  }

  float e = computeError(s.rawL, s.rawC, s.rawR);
  PIDState& pid = (travelDir > 0) ? pidForward : pidBackward;
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

bool handleRecover(const Sense& s,
                   State followState,
                   int basePwm,
                   int travelDir) {
  bool recovered = recoverLine(s, basePwm, travelDir);
  if (recovered) {
    lostTimer.reset();
    state = followState;
    resetPidForState(followState);
  }
  return recovered;
}

void handleEndpointLimitReached() {
  Serial.println(F("Endpoint limit reached -> DONE"));
  state = DONE;
  uturnTimer.reset();
}

void onEndpointEncountered() {
  ++endpointCount;
  if (ENDPOINT_DONE_COUNT > 0 && endpointCount >= ENDPOINT_DONE_COUNT) {
    handleEndpointLimitReached();
  }
}

void handleForwardEndpoint() {
  setWheels(0, 0);
  lostTimer.reset();

  onEndpointEncountered();
  if (state == DONE) {
    return;
  }

  if (runMode == RUNMODE_UTURN) {
    Serial.println(F("Endpoint (forward) -> UTURN"));
    uturnTimer.start();
    state = UTURN;
  } else {
    state = SEEK_LINE_BACK;
    Serial.println(F("Endpoint (forward) -> SEEK_LINE_BACK"));
  }
}

void handleBackwardEndpoint() {
  setWheels(0, 0);
  lostTimer.reset();

  onEndpointEncountered();
  if (state == DONE) {
    return;
  }

  state = SEEK_LINE_FWD;
  Serial.println(F("Endpoint (backward) -> SEEK_LINE_FWD"));
}

void handleForwardLineLost() {
  if (runMode == RUNMODE_LOOP) {
    state = RECOVER_FWD;
    Serial.println(F("Line lost (forward) -> RECOVER_FWD"));
    return;
  }

  handleForwardEndpoint();
}

void handleBackwardLineLost() {
  if (runMode == RUNMODE_LOOP) {
    state = RECOVER_BACK;
    Serial.println(F("Line lost (backward) -> RECOVER_BACK"));
    return;
  }

  handleBackwardEndpoint();
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
  Serial.println(state);

  switch (state) {
    // 端点(全白)から前進して黒ラインを掴む
    case SEEK_LINE_FWD: {
      bool found = handleSeekLine(FOLLOW_FWD, +1, s);
      if (found) {
        Serial.println(F("-> FOLLOW_FWD"));
      }
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
      bool recovered = handleRecover(s,
                                     FOLLOW_FWD,
                                     BASE_FWD,
                                     +1);
      if (recovered) {
        Serial.println(F("Recovered (forward) -> FOLLOW_FWD"));
      }
      break;
    }

    // 折り返し：後退で黒ライン再捕捉
    case SEEK_LINE_BACK: {
      bool found = handleSeekLine(FOLLOW_BACK, -1, s);
      if (found) {
        Serial.println(F("-> FOLLOW_BACK"));
      }
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
      bool recovered = handleRecover(s,
                                     FOLLOW_BACK,
                                     BASE_BACK,
                                     -1);
      if (recovered) {
        Serial.println(F("Recovered (back) -> FOLLOW_BACK"));
      }
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
