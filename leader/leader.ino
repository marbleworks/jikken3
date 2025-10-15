#include "pins.h"

// ------------------ チューニング用パラメータ ------------------
int   THRESHOLD      = 500;   // 白40 / 黒1000想定の中間。環境で調整
int   HYST           = 40;    // ヒステリシス
int   BASE_FWD       = 160;   // 前進の基準PWM
int   BASE_BACK      = 150;   // 後退の基準PWM
float KP_FWD         = 0.20;  // 前進Pゲイン
float KP_BACK        = 0.20;  // 後退Pゲイン
int   MAX_PWM        = 255;   // PWM上限
int   MIN_PWM        = 0;     // PWM下限
int   SEEK_SPEED     = 120;   // ライン探索速度（端点から黒を掴むまで）
unsigned long END_WHITE_MS = 600; // 端点判定（両白がこの時間以上続く）
unsigned long LOST_MS      = 120; // 見失い判定（FOLLOW中に両白がこの時間続いたらリカバリ）
int   REC_STEER      = 80;    // リカバリ時の曲げ量（左右差）
// ----------------------------------------------------------------

// ====== struct をグローバルで定義（末尾セミコロン必須）======
struct Sense {
  int  rawL, rawR;     // センサ生値
  bool isBlackL;       // 左が黒か
  bool isBlackR;       // 右が黒か
  bool bothBlack;
  bool bothWhite;
}; // ← セミコロン必須
struct FollowResult {
  bool lineLost;
  bool endpoint;
};
// =================================================================

// 状態機械
enum RunMode {
  RUNMODE_RECIP,
  RUNMODE_LOOP,
  RUNMODE_UTURN
};

inline constexpr RunMode COMPILE_TIME_RUNMODE = RUNMODE_RECIP; // 直線コース向け既定値

// 端点でその場Uターンする際のパラメータ
inline constexpr int UTURN_SPEED = 150;        // 片輪前進・片輪後退のPWM
inline constexpr unsigned long UTURN_TIME_MS = 600; // 180度回頭に掛ける時間（要調整）

// DIP スイッチでモードを切り替える場合はピン番号を指定（未使用なら -1 のまま）
inline constexpr int RUNMODE_DIP_PIN = -1;
inline constexpr bool RUNMODE_DIP_USE_PULLUP = true;
inline constexpr int RUNMODE_DIP_ACTIVE_LEVEL = LOW; // DIP ON（GNDに落とす）を LOW と想定
inline constexpr RunMode RUNMODE_DIP_ACTIVE_MODE = RUNMODE_LOOP;
inline constexpr RunMode RUNMODE_DIP_INACTIVE_MODE = RUNMODE_RECIP;

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
unsigned long bothWhiteSince = 0; // 両白が続いている開始時刻（端点判定用）
bool lastBothWhite = false;

unsigned long whiteSinceFollow = 0;  // FOLLOW中の「両白開始時刻」（見失い判定用）
int lastBlackDir = 0;                // -1=左が黒, +1=右が黒, 0=両黒/不明（最後に黒を見た側の記録）

// ------------------ 低レベル：モータ制御 ------------------
void drivePins(int IN1, int IN2, bool cw) {
  digitalWrite(IN1, cw ? HIGH : LOW);
  digitalWrite(IN2, cw ? LOW  : HIGH);
}

// 速度符号付き指定：正=前進、負=後退、0=停止（惰性寄り）
void setWheelA(int speedSigned) {
  int spd = constrain(abs(speedSigned), MIN_PWM, MAX_PWM);
  if (speedSigned > 0)      drivePins(A_IN1, A_IN2, true);
  else if (speedSigned < 0) drivePins(A_IN1, A_IN2, false);
  else { digitalWrite(A_IN1, LOW); digitalWrite(A_IN2, LOW); }
  analogWrite(A_PWM, spd);
}

void setWheelB(int speedSigned) {
  int spd = constrain(abs(speedSigned), MIN_PWM, MAX_PWM);
  if (speedSigned > 0)      drivePins(B_IN1, B_IN2, true);
  else if (speedSigned < 0) drivePins(B_IN1, B_IN2, false);
  else { digitalWrite(B_IN1, LOW); digitalWrite(B_IN2, LOW); }
  analogWrite(B_PWM, spd);
}

// 左右同時設定（LEFT_IS_Aに合わせて割当）
void setWheels(int leftSpeed, int rightSpeed) {
  if (LEFT_IS_A) { setWheelA(leftSpeed); setWheelB(rightSpeed); }
  else           { setWheelA(rightSpeed); setWheelB(leftSpeed); }
}

// ------------------ センサ関連（struct 定義後に関数を定義） ------------------
Sense readSensors() {
  Sense s;
  s.rawL = analogRead(pinL);
  s.rawR = analogRead(pinR);

  // ヒステリシス付き判定
  static bool lastLBlack=false, lastRBlack=false;
  int thH = THRESHOLD + HYST;
  int thL = THRESHOLD - HYST;

  if (lastLBlack) s.isBlackL = (s.rawL > thL);
  else            s.isBlackL = (s.rawL > thH);

  if (lastRBlack) s.isBlackR = (s.rawR > thL);
  else            s.isBlackR = (s.rawR > thH);

  lastLBlack = s.isBlackL;
  lastRBlack = s.isBlackR;

  s.bothBlack = s.isBlackL && s.isBlackR;
  s.bothWhite = !s.isBlackL && !s.isBlackR;

  // 最後に黒を見た側を更新
  if (s.isBlackL && !s.isBlackR)      lastBlackDir = -1;
  else if (s.isBlackR && !s.isBlackL) lastBlackDir = +1;
  else if (s.bothBlack)               lastBlackDir = 0;

  return s;
}

// 誤差（右-左）：正→右が濃い(黒寄り)
float computeError(int rawL, int rawR) {
  float l = (float)(rawL - THRESHOLD);
  float r = (float)(rawR - THRESHOLD);
  const float span = 1000.0f - 40.0f; // 想定レンジ
  float nl = l / span;
  float nr = r / span;
  return (nr - nl); // 右-左
}

// ------------------ 端点（両白）検出のデバウンス ------------------
bool endpointSeen(bool bothWhiteNow) {
  unsigned long t = millis();
  if (bothWhiteNow) {
    if (!lastBothWhite) bothWhiteSince = t; // 立ち上がり
    lastBothWhite = true;
    return (t - bothWhiteSince) >= END_WHITE_MS;
  } else {
    lastBothWhite = false;
    return false;
  }
}

const char* runModeLabel(RunMode mode) {
  switch (mode) {
    case RUNMODE_RECIP: return "Reciprocal";
    case RUNMODE_LOOP:  return "Loop";
    case RUNMODE_UTURN: return "UTurn";
    default:            return "Unknown";
  }
}

void applyDipRunMode() {
  if (RUNMODE_DIP_PIN < 0) return;

  if (RUNMODE_DIP_USE_PULLUP) pinMode(RUNMODE_DIP_PIN, INPUT_PULLUP);
  else                        pinMode(RUNMODE_DIP_PIN, INPUT);

  int level = digitalRead(RUNMODE_DIP_PIN);
  runMode = (level == RUNMODE_DIP_ACTIVE_LEVEL) ? RUNMODE_DIP_ACTIVE_MODE : RUNMODE_DIP_INACTIVE_MODE;

  Serial.print("DIP run mode selection (pin ");
  Serial.print(RUNMODE_DIP_PIN);
  Serial.print("): ");
  Serial.println(runModeLabel(runMode));
}

bool parseRunModeCommand(const String& cmd, RunMode& out) {
  String normalized = cmd;
  normalized.trim();
  normalized.toLowerCase();
  if (normalized == "loop") {
    out = RUNMODE_LOOP;
    return true;
  }
  if (normalized == "uturn" || normalized == "turn") {
    out = RUNMODE_UTURN;
    return true;
  }
  if (normalized == "recip" || normalized == "reciprocal") {
    out = RUNMODE_RECIP;
    return true;
  }
  return false;
}

void awaitSerialRunModeOverride(unsigned long waitMs) {
  if (waitMs == 0) return;

  Serial.print("Send 'recip' or 'loop' within ");
  Serial.print(waitMs);
  Serial.println(" ms to override run mode...");

  String buffer;
  unsigned long deadline = millis() + waitMs;
  while (millis() < deadline) {
    if (Serial.available()) {
      char c = (char)Serial.read();
      if (c == '\n' || c == '\r') {
        if (buffer.length() > 0) break;
      } else {
        buffer += c;
      }
    }
  }

  if (buffer.length() == 0) {
    Serial.println("Using existing run mode selection.");
    return;
  }

  RunMode parsed;
  if (parseRunModeCommand(buffer, parsed)) {
    runMode = parsed;
    Serial.print("Run mode override via Serial: ");
    Serial.println(runModeLabel(runMode));
  } else {
    Serial.print("Unknown run mode command: ");
    Serial.println(buffer);
    Serial.println("Keeping previous selection.");
  }
}

FollowResult runLineTraceCommon(const Sense& s, int travelDir) {
  FollowResult res { false, false };

  if (s.bothWhite) {
    if (whiteSinceFollow == 0) whiteSinceFollow = millis();
    if (millis() - whiteSinceFollow > LOST_MS) {
      res.lineLost = true;
      return res;
    }
  } else {
    whiteSinceFollow = 0;
  }

  float e = computeError(s.rawL, s.rawR);
  float kp = (travelDir > 0) ? KP_FWD : KP_BACK;
  int base = (travelDir > 0) ? BASE_FWD : BASE_BACK;
  int corr = (int)(kp * e * 255.0f);

  int left, right;
  if (travelDir > 0) {
    left  = constrain(base + corr, MIN_PWM, MAX_PWM);
    right = constrain(base - corr, MIN_PWM, MAX_PWM);
  } else {
    left  = constrain(-base - corr, -MAX_PWM, -MIN_PWM);
    right = constrain(-base + corr, -MAX_PWM, -MIN_PWM);
  }
  setWheels(left, right);

  res.endpoint = endpointSeen(s.bothWhite);
  return res;
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

// ------------------ setup / loop ------------------
void setup() {
  Serial.begin(115200);
  runMode = COMPILE_TIME_RUNMODE;
  applyDipRunMode();
  awaitSerialRunModeOverride(3000);
  pinMode(A_IN1, OUTPUT); pinMode(A_IN2, OUTPUT);
  pinMode(B_IN1, OUTPUT); pinMode(B_IN2, OUTPUT);
  setWheels(0, 0);
  Serial.print("Power-on (run mode: ");
  Serial.print(runModeLabel(runMode));
  Serial.println(") -> SEEK_LINE_FWD");
}

void loop() {
  Sense s = readSensors();

  switch (state) {
    // 端点(白)から前進して黒ラインを掴む
    case SEEK_LINE_FWD: {
      setWheels(SEEK_SPEED, SEEK_SPEED);
      if (s.isBlackL || s.isBlackR) {
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
      int l = BASE_FWD, r = BASE_FWD;
      if (lastBlackDir > 0) {          // 右で黒 → 右へ寄る
        l = BASE_FWD + REC_STEER;
        r = BASE_FWD - REC_STEER;
      } else if (lastBlackDir < 0) {   // 左で黒 → 左へ寄る
        l = BASE_FWD - REC_STEER;
        r = BASE_FWD + REC_STEER;
      } else {
        // 不明なら左右に小刻みに振る（300ms周期）
        bool rightBias = (millis() / 300) % 2;
        l = BASE_FWD + (rightBias ? +REC_STEER : -REC_STEER);
        r = BASE_FWD - (rightBias ? +REC_STEER : -REC_STEER);
      }
      setWheels(constrain(l, MIN_PWM, MAX_PWM), constrain(r, MIN_PWM, MAX_PWM));

      // 再捕捉したら復帰
      if (s.isBlackL || s.isBlackR || s.bothBlack) {
        whiteSinceFollow = 0;
        state = FOLLOW_FWD;
        Serial.println("Recovered (forward) -> FOLLOW_FWD");
      }

      // リカバリ中でも端点ならモードに応じて処理
      if (endpointSeen(s.bothWhite)) {
        handleForwardEndpoint("(recover fwd)");
      }
      break;
    }

    // 折り返し：後退で黒ライン再捕捉
    case SEEK_LINE_BACK: {
      setWheels(-SEEK_SPEED, -SEEK_SPEED);
      if (s.isBlackL || s.isBlackR) {
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
        setWheels(0, 0);
        Serial.println("Back to start. DONE.");
        state = DONE;
      }
      break;
    }

    // 後退のリカバリ：最後に黒を見た側の“逆”へ切る（進行方向が逆）
    case RECOVER_BACK: {
      int l = -BASE_BACK, r = -BASE_BACK;
      if (lastBlackDir > 0) {          // 右で黒 → 後退では左へ切る
        l = -BASE_BACK - REC_STEER;
        r = -BASE_BACK + REC_STEER;
      } else if (lastBlackDir < 0) {   // 左で黒 → 後退では右へ切る
        l = -BASE_BACK + REC_STEER;
        r = -BASE_BACK - REC_STEER;
      } else {
        bool rightBias = (millis() / 300) % 2;
        l = -BASE_BACK + (rightBias ? -REC_STEER : +REC_STEER);
        r = -BASE_BACK - (rightBias ? -REC_STEER : +REC_STEER);
      }
      setWheels(constrain(l, -MAX_PWM, -MIN_PWM),
                constrain(r, -MAX_PWM, -MIN_PWM));

      if (s.isBlackL || s.isBlackR || s.bothBlack) {
        whiteSinceFollow = 0;
        state = FOLLOW_BACK;
        Serial.println("Recovered (back) -> FOLLOW_BACK");
      }

      if (endpointSeen(s.bothWhite) && runMode == RUNMODE_RECIP) {
        setWheels(0, 0);
        Serial.println("Back to start. DONE.");
        state = DONE;
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
