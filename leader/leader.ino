#include "pins.h"
#include "sensor_leds.h"
#include "sensors.h"
#include "wheel_control.h"

// ------------------ チューニング用パラメータ ------------------
int   THRESHOLD      = 500;   // 白40 / 黒1000想定の中間。環境で調整
int   HYST           = 40;    // ヒステリシス
int   BASE_FWD       = 160;   // 前進の基準PWM
int   BASE_BACK      = 150;   // 後退の基準PWM
float KP_FWD         = 1;  // 前進Pゲイン
float KP_BACK        = 1;  // 後退Pゲイン
int   MAX_PWM        = 96;   // PWM上限
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

// 状態機械
enum RunMode {
  RUNMODE_RECIP,
  RUNMODE_UTURN,
  RUNMODE_LOOP
};

inline constexpr RunMode COMPILE_TIME_RUNMODE = RUNMODE_RECIP; // 直線コース向け既定値

enum State {
  SEEK_LINE_FWD,     // 端点スタート（白）→黒ラインを探しながら前進
  FOLLOW_FWD,        // 前進でライン追従
  RECOVER_FWD,       // 前進中にライン見失い→自動復帰
  SEEK_LINE_BACK,    // 折り返し（後退で黒ライン再捕捉）
  FOLLOW_BACK,       // 後退でライン追従（復路）
  RECOVER_BACK,      // 後退中に見失い→自動復帰
  DONE               // 完了（停止）
};
State state = SEEK_LINE_FWD;
RunMode runMode = COMPILE_TIME_RUNMODE;

unsigned long lapCount = 0; // RUNMODE_LOOP で端点を通過した回数

// 端点検出・見失い管理
unsigned long allWhiteSince = 0; // 全白が続いている開始時刻（端点判定用）
bool lastAllWhite = false;

unsigned long whiteSinceFollow = 0;  // FOLLOW中の「全白開始時刻」（見失い判定用）
int lastBlackDirState = 0;           // -1=左, +1=右, 0=中央/不明

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
    whiteSinceFollow = 0;
    Serial.println(logMsg);
  }
}
// ------------------ 端点（全白）検出のデバウンス ------------------
bool endpointSeen(bool allWhiteNow) {
  unsigned long t = millis();
  if (allWhiteNow) {
    if (!lastAllWhite) allWhiteSince = t; // 立ち上がり
    lastAllWhite = true;
    return (t - allWhiteSince) >= END_WHITE_MS;
  } else {
    lastAllWhite = false;
    return false;
  }
}

const char* runModeLabel(RunMode mode) {
  switch (mode) {
    case RUNMODE_RECIP: return "Reciprocal";
    case RUNMODE_UTURN: return "UTurn";
    case RUNMODE_LOOP:  return "Loop";
    default:            return "Unknown";
  }
}

FollowResult runLineTraceCommon(const Sense& s, int travelDir) {
  FollowResult res { false, false };

  if (s.allWhite) {
    if (whiteSinceFollow == 0) whiteSinceFollow = millis();
    if (millis() - whiteSinceFollow > LOST_MS) {
      res.lineLost = true;
      return res;
    }
  } else {
    whiteSinceFollow = 0;
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

void handleForwardEndpoint(const char* context) {
  setWheels(0, 0);
  delay(150);
  whiteSinceFollow = 0;

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

    unsigned long start = millis();
    while (millis() - start < UTURN_TIME_MS) {
      setWheels(UTURN_SPEED, -UTURN_SPEED);
      delay(5);
    }

    setWheels(0, 0);
    state = SEEK_LINE_FWD;
    Serial.println("UTURN complete -> SEEK_LINE_FWD");
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
      bool recovered = recoverLine(s, BASE_FWD, +1);
      if (recovered) {
        whiteSinceFollow = 0;
        state = FOLLOW_FWD;
        Serial.println("Recovered (forward) -> FOLLOW_FWD");
      }

      // リカバリ中でも端点ならモードに応じて処理
      if (endpointSeen(s.allWhite)) {
        handleForwardEndpoint("(recover fwd)");
      }
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
      bool recovered = recoverLine(s, BASE_BACK, -1);
      if (recovered) {
        whiteSinceFollow = 0;
        state = FOLLOW_BACK;
        Serial.println("Recovered (back) -> FOLLOW_BACK");
      }

      if (endpointSeen(s.allWhite) && runMode == RUNMODE_RECIP) {
        finishReciprocalReturn("Back to start.");
      }
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
