#pragma once
#include <Arduino.h>

// ===== センサ配線 =====
inline constexpr uint8_t pinL  = A0;  // 前左
inline constexpr uint8_t pinC  = A1;  // 前中央
inline constexpr uint8_t pinR  = A2;  // 前右
inline constexpr uint8_t pinRL = A3;  // 後左
inline constexpr uint8_t pinRR = A4;  // 後右

// ===== 実行モード選択用可変抵抗 =====
inline constexpr int RUNMODE_POT_PIN = A5;
inline constexpr bool RUNMODE_POT_ENABLED = true;

inline constexpr uint8_t LED_WARN  = 2; 

// ===== モータ配線（例） =====
// モータA（左）
inline constexpr uint8_t A_PWM = 3;
inline constexpr uint8_t A_IN1 = 4;
inline constexpr uint8_t A_IN2 = 5;
// モータB（右）
inline constexpr uint8_t B_PWM = 6;
inline constexpr uint8_t B_IN1 = 7;
inline constexpr uint8_t B_IN2 = 8;

// ===== センサ状態表示用 LED =====
inline constexpr uint8_t LED_LEFT_SENSOR       = 9;   // 前左センサの黒検出表示
inline constexpr uint8_t LED_CENTER_SENSOR     = 10;  // 前中央センサの黒検出表示
inline constexpr uint8_t LED_RIGHT_SENSOR      = 11;  // 前右センサの黒検出表示
inline constexpr uint8_t LED_REAR_LEFT_SENSOR  = 12;  // 後左センサの黒検出表示
inline constexpr uint8_t LED_REAR_RIGHT_SENSOR = 13;  // 後右センサの黒検出表示

// 左右が A/B のどちらか（配線に合わせて変更）
inline constexpr bool LEFT_IS_A = true; // 左がAならtrue、右がAならfalse
