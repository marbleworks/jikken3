#pragma once

#include <Arduino.h>

// 速度符号付き指定：正=前進、負=後退、0=停止（惰性寄り）
void setupWheelPins();
void setWheelA(int speedSigned);
void setWheelB(int speedSigned);

// 左右同時設定（LEFT_IS_Aに合わせて割当）
// left>right → 右旋回（ω<0）、right>left → 左旋回（ω>0）
void setWheels(int leftSpeed, int rightSpeed);

int getLastLeftCommand();
int getLastRightCommand();
