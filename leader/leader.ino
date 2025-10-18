#include "pins.h"

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
struct Sense {
  int  rawL, rawC, rawR; // センサ生値
  bool isBlackL;         // 左が黒か
  bool isBlackC;         // 中央が黒か
  bool isBlackR;         // 右が黒か
  bool anyBlack;         // いずれかが黒か
  bool allBlack;         // 全て黒か
  bool allWhite;         // 全て白か
};
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
int lastBlackDir = 0;                // -1=左が黒, +1=右が黒, 0=両黒/不明（最後に黒を見た側の記録）

// ------------------ 低レベル：モータ制御 ------------------
void drivePins(int IN1, int IN2, bool cw) {
  digitalWrite(IN1, cw ? HIGH : LOW);
  digitalWrite(IN2, cw ? LOW  : HIGH);
}

void setWheel(int speedSigned, uint8_t in1, uint8_t in2, uint8_t pwmPin) {
  int spd = constrain(abs(speedSigned), MIN_PWM, MAX_PWM);
  if (speedSigned > 0)      drivePins(in1, in2, true);
  else if (speedSigned < 0) drivePins(in1, in2, false);
  else { digitalWrite(in1, LOW); digitalWrite(in2, LOW); }
  analogWrite(pwmPin, spd);
}

// 速度符号付き指定：正=前進、負=後退、0=停止（惰性寄り）
void setWheelA(int speedSigned) {
  setWheel(speedSigned, A_IN1, A_IN2, A_PWM);
}

void setWheelB(int speedSigned) {
  setWheel(speedSigned, B_IN1, B_IN2, B_PWM);
}

// 左右同時設定（LEFT_IS_Aに合わせて割当）
// left>right → 右旋回（ω<0）、right>left → 左旋回（ω>0）
void setWheels(int leftSpeed, int rightSpeed) {
  if (LEFT_IS_A) { setWheelA(leftSpeed); setWheelB(rightSpeed); }
  else           { setWheelA(rightSpeed); setWheelB(leftSpeed); }
}

// ------------------ センサ関連（struct 定義後に関数を定義） ------------------
Sense readSensors() {
  Sense s;
  s.rawL = analogRead(pinL);
  s.rawC = analogRead(pinC);
  s.rawR = analogRead(pinR);

  // ヒステリシス付き判定
  static bool lastLBlack=false, lastCBlack=false, lastRBlack=false;
  int thH = THRESHOLD + HYST;
  int thL = THRESHOLD - HYST;

  if (lastLBlack) s.isBlackL = (s.rawL > thL);
  else            s.isBlackL = (s.rawL > thH);

  if (lastCBlack) s.isBlackC = (s.rawC > thL);
  else            s.isBlackC = (s.rawC > thH);

  if (lastRBlack) s.isBlackR = (s.rawR > thL);
  else            s.isBlackR = (s.rawR > thH);

  lastLBlack = s.isBlackL;
  lastCBlack = s.isBlackC;
  lastRBlack = s.isBlackR;

  s.anyBlack = s.isBlackL || s.isBlackC || s.isBlackR;
  s.allBlack = s.isBlackL && s.isBlackC && s.isBlackR;
  s.allWhite = !s.anyBlack;

  // 最後に黒を見た側を更新
  if (s.isBlackL && !s.isBlackR)      lastBlackDir = -1;
  else if (s.isBlackR && !s.isBlackL) lastBlackDir = +1;
  else if (s.isBlackC)                lastBlackDir = 0;
  else if (s.allBlack)                lastBlackDir = 0;

  digitalWrite(LED_LEFT_SENSOR,  s.isBlackL ? HIGH : LOW);
  digitalWrite(LED_CENTER_SENSOR,s.isBlackC ? HIGH : LOW);
  digitalWrite(LED_RIGHT_SENSOR, s.isBlackR ? HIGH : LOW);

  return s;
}

// 誤差（右正）：正→右寄り、負→左寄り
float computeError(int rawL, int rawC, int rawR) {
  const float span = 1000.0f - 40.0f; // 想定レンジ
  float weightL = max(0.0f, (float)(rawL - THRESHOLD)) / span;
  float weightC = max(0.0f, (float)(rawC - THRESHOLD)) / span;
  float weightR = max(0.0f, (float)(rawR - THRESHOLD)) / span;
  float total = weightL + weightC + weightR;
  if (total < 0.001f) {
    return 0.0f;
  }
  float position = (-1.0f * weightL + 1.0f * weightR) / total;
  return position;
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
  if (lastBlackDir > 0) {
    steerOffset = REC_STEER;
  } else if (lastBlackDir < 0) {
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
  pinMode(A_IN1, OUTPUT); pinMode(A_IN2, OUTPUT);
  pinMode(B_IN1, OUTPUT); pinMode(B_IN2, OUTPUT);
  pinMode(LED_LEFT_SENSOR, OUTPUT);
  pinMode(LED_CENTER_SENSOR, OUTPUT);
  pinMode(LED_RIGHT_SENSOR, OUTPUT);
  digitalWrite(LED_LEFT_SENSOR, LOW);
  digitalWrite(LED_CENTER_SENSOR, LOW);
  digitalWrite(LED_RIGHT_SENSOR, LOW);
  setWheels(0, 0);
  Serial.print("Power-on (run mode: ");
  Serial.print(runModeLabel(runMode));
  Serial.println(") -> SEEK_LINE_FWD");
}

void loop() {
  Sense s = readSensors();
  Serial.println(state);
  // setWheels(128,0);

  switch (state) {
    // 端点(全白)から前進して黒ラインを掴む
    case SEEK_LINE_FWD: {
      setWheels(SEEK_SPEED, SEEK_SPEED);
      if (s.anyBlack) {
        state = FOLLOW_FWD;
        whiteSinceFollow = 0;
        Serial.println("-> FOLLOW_FWD");
      }
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
      setWheels(-SEEK_SPEED, -SEEK_SPEED);
      if (s.anyBlack) {
        state = FOLLOW_BACK;
        whiteSinceFollow = 0;
        Serial.println("-> FOLLOW_BACK");
      }
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
