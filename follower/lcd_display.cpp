#include "lcd_display.h"

LCDDisplay::LCDDisplay(uint8_t rs, uint8_t e, uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7)
  : lcd(rs, e, d4, d5, d6, d7) {}

void LCDDisplay::begin() {
  lcd.begin(8, 2);  // 8文字 x 2行
}

void LCDDisplay::showDistances(float left, float right) {
  // 1行目: "L:XXX.X" または "L:---" (左距離)
  lcd.setCursor(0, 0);
  lcd.print("L:");
  if (isnan(left)) {
    lcd.print("---   ");
  } else {
    lcd.print(left, 1);
    lcd.print("  ");
  }

  // 2行目: "R:XXX.X" または "R:---" (右距離)
  lcd.setCursor(0, 1);
  lcd.print("R:");
  if (isnan(right)) {
    lcd.print("---   ");
  } else {
    lcd.print(right, 1);
    lcd.print("  ");
  }
}
