// モータピン定義
constexpr uint8_t A_PWM = 3;
constexpr uint8_t A_IN1 = 4;
constexpr uint8_t A_IN2 = 5;
constexpr uint8_t B_PWM = 6;
constexpr uint8_t B_IN1 = 7;
constexpr uint8_t B_IN2 = 8;

// 0: 直進(255,255)  1: 右旋回(255,128)  2: 左旋回(128,255)
constexpr int MODE = 0;

void setup() {
  pinMode(A_IN1, OUTPUT);
  pinMode(A_IN2, OUTPUT);
  pinMode(A_PWM, OUTPUT);
  pinMode(B_IN1, OUTPUT);
  pinMode(B_IN2, OUTPUT);
  pinMode(B_PWM, OUTPUT);

  // 前進方向
  digitalWrite(A_IN1, HIGH);
  digitalWrite(A_IN2, LOW);
  digitalWrite(B_IN1, HIGH);
  digitalWrite(B_IN2, LOW);

  int leftPWM, rightPWM;
  switch (MODE) {
    case 1:  leftPWM = 255; rightPWM = 128; break;  // 右旋回
    case 2:  leftPWM = 128; rightPWM = 255; break;  // 左旋回
    default: leftPWM = 100; rightPWM = 100; break;  // 直進
  }

  analogWrite(A_PWM, leftPWM);   // 左=A
  analogWrite(B_PWM, rightPWM);  // 右=B
}

void loop() {
}
