#pragma once

#include <Arduino.h>

constexpr uint8_t PIN_IR_OBST    = 2;

constexpr uint8_t PIN_LEFT_PWM   = 3;
constexpr uint8_t PIN_LEFT_IN1   = 4;
constexpr uint8_t PIN_LEFT_IN2   = 5;

constexpr uint8_t PIN_RIGHT_PWM  = 6;
constexpr uint8_t PIN_RIGHT_IN1  = 7;
constexpr uint8_t PIN_RIGHT_IN2  = 8;

constexpr uint8_t PIN_TRIG_LEFT  = 9;
constexpr uint8_t PIN_ECHO_LEFT  = 10;

constexpr uint8_t PIN_TRIG_RIGHT = 11;
constexpr uint8_t PIN_ECHO_RIGHT = 12;

// LCD表示ピン (ACM0802C-NLW-BBH)
constexpr uint8_t PIN_LCD_RS = A0;
constexpr uint8_t PIN_LCD_E  = A1;
constexpr uint8_t PIN_LCD_D4 = A2;
constexpr uint8_t PIN_LCD_D5 = A3;
constexpr uint8_t PIN_LCD_D6 = A4;
constexpr uint8_t PIN_LCD_D7 = A5;
