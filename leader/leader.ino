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
float KP_BACK        = 0.2f;  // 後退Pゲイン
float LINE_WHITE     = 40.0f;   // センサ白レベル
float LINE_BLACK     = 900.0f;  // センサ黒レベル
float LINE_EPS       = 1e-3f;   // 全白判定のしきい値
int   MAX_PWM        = 255;   // PWM上限
int   MIN_PWM        = 0;     // PWM下限
int   SEEK_SPEED     = 120;   // ライン探索速度（端点から黒を掴むまで）
unsigned long END_WHITE_MS = 800; // 端点判定（全白がこの時間以上続く）
unsigned long LOST_MS      = 300; // 見失い判定（FOLLOW中に全白がこの時間続いたらリカバリ）
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

unsigned long lapCount = 0; // RUNMODE_LOOP で端点を通過した回数

// 端点検出・見失い管理
Timer endpointTimer;
Timer lostTimer;
int lastBlackDirState = 0;           // -1=左, +1=右, 0=中央/不明
Timer uturnTimer;

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
  lostTimer.reset();
  uturnTimer.reset();
  Serial.println(F("UTURN complete -> SEEK_LINE_FWD"));
}

void updateLastBlackDirState(const Sense& s) {
  if (s.anyBlack) {
    lastBlackDirState = getBlackDirState(s);
  }
}

void handleSeekLine(State followState, int speedSign, const __FlashStringHelper* logMsg, const Sense& s) {
  int speed = speedSign * SEEK_SPEED;
  setWheels(speed, speed);
  if (s.anyBlack) {
    state = followState;
    lostTimer.reset();
    Serial.println(logMsg);
  }
}
// ------------------ 端点（全白）検出のデバウンス ------------------
bool endpointSeen(bool allWhiteNow) {
  if (allWhiteNow) {
    if (!endpointTimer.running()) {
      endpointTimer.start();
    }
    return endpointTimer.elapsed() >= END_WHITE_MS;
  } else {
    endpointTimer.reset();
    return false;
  }
}

FollowResult runLineTraceCommon(const Sense& s, int travelDir) {
  FollowResult res { false, false };

  if (s.allWhite) {
    if (!lostTimer.running()) {
      lostTimer.start();
    }
    if (lostTimer.elapsed() > LOST_MS) {
      res.lineLost = true;
      return res;
    }
  } else {
    lostTimer.reset();
  }

  float e = computeError(s.rawL, s.rawC, s.rawR);
  float kp = (travelDir > 0) ? KP_FWD : KP_BACK;
  int base = (travelDir > 0) ? BASE_FWD : BASE_BACK;
  int corr = (int)(kp * e * 255.0f);

  int dirSign    = (travelDir >= 0) ? 1 : -1;

  int left  = constrain(base + corr, MIN_PWM, MAX_PWM) * dirSign;
  int right = constrain(base - corr, MIN_PWM, MAX_PWM) * dirSign;
  setWheels(left, right);

  res.endpoint = endpointSeen(s.allWhite);
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
  bool recovered = recoverLine(s, basePwm, travelDir);
  if (recovered) {
    lostTimer.reset();
    state = followState;
    Serial.println(logMsg);
  }

  if (enableEndpointHandling && endpointHandler != nullptr && endpointSeen(s.allWhite)) {
    endpointHandler(endpointContext);
  }
}

void handleForwardEndpoint(const char* context) {
  setWheels(0, 0);
  delay(150);
  lostTimer.reset();

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
  state = DONE;
}

// ------------------ setup / loop ------------------
void setup() {
  Serial.begin(115200);
  runMode = COMPILE_TIME_RUNMODE;
  applyPotRunMode();
  setupWheelPins();
  setupSensorLeds();
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
