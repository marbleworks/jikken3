#pragma once
#include <Arduino.h>

// ===== センサ配線（6センサ・弓形配置）=====
// 配置:     S2  S3      (中央・前)
//         S1      S4    (中間)
//       S0          S5  (外側・後)
inline constexpr uint8_t SENSOR_PINS[6] = {A0, A1, A2, A3, A4, A5};

// ===== モータ配線 =====
// モータA（左）
inline constexpr uint8_t A_PWM = 3;
inline constexpr uint8_t A_IN1 = 4;
inline constexpr uint8_t A_IN2 = 5;
// モータB（右）
inline constexpr uint8_t B_PWM = 6;
inline constexpr uint8_t B_IN1 = 7;
inline constexpr uint8_t B_IN2 = 8;

// 左右が A/B のどちらか
inline constexpr bool LEFT_IS_A = true;
