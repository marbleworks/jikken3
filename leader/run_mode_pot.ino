// 可変抵抗による走行モード選択ロジック
// 配線: 可変抵抗の中央端子を A2、両端を 5V / GND に接続すること

inline constexpr int RUNMODE_POT_PIN = A2;
inline constexpr int RUNMODE_POT_SAMPLES = 8;

extern RunMode runMode;
extern const char* runModeLabel(RunMode mode);

void applyPotRunMode() {
  if (RUNMODE_POT_PIN < 0) return;

  pinMode(RUNMODE_POT_PIN, INPUT);

  long total = 0;
  for (int i = 0; i < RUNMODE_POT_SAMPLES; ++i) {
    total += analogRead(RUNMODE_POT_PIN);
  }

  int average = total / RUNMODE_POT_SAMPLES;

  const int thresholdLoop = 1024 / 3;        // ≒341
  const int thresholdUTurn = (1024 * 2) / 3; // ≒682

  RunMode selected;
  if (average < thresholdLoop) {
    selected = RUNMODE_RECIP;
  } else if (average < thresholdUTurn) {
    selected = RUNMODE_LOOP;
  } else {
    selected = RUNMODE_UTURN;
  }

  runMode = selected;

  Serial.print("Pot run mode selection (pin ");
  Serial.print(RUNMODE_POT_PIN);
  Serial.print(", avg=");
  Serial.print(average);
  Serial.print("): ");
  Serial.println(runModeLabel(runMode));
}
