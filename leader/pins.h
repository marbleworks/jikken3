#pragma once
#include <Arduino.h>

// ===== センサ配線 =====
inline constexpr uint8_t pinL = A0;  // 左
inline constexpr uint8_t pinR = A1;  // 右

// ===== モータ配線（例） =====
// モータA（左）
inline constexpr uint8_t A_IN1 = 7;
inline constexpr uint8_t A_IN2 = 8;
inline constexpr uint8_t A_PWM = 5;
// モータB（右）
inline constexpr uint8_t B_IN1 = 9;
inline constexpr uint8_t B_IN2 = 10;
inline constexpr uint8_t B_PWM = 6;

// 左右が A/B のどちらか（配線に合わせて変更）
inline constexpr bool LEFT_IS_A = true; // 左がAならtrue、右がAならfalse
