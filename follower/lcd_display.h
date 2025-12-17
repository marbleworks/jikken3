#pragma once
#include <Arduino.h>
#include <LiquidCrystal.h>

class LCDDisplay {
public:
  LCDDisplay(uint8_t rs, uint8_t e, uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7);
  void begin();
  void showDistances(float left, float right);

private:
  LiquidCrystal lcd;
};
