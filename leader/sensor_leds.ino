#include "pins.h"
#include "sensor_leds.h"

void displaySensorStates(bool leftBlack, bool centerBlack, bool rightBlack) {
  digitalWrite(LED_LEFT_SENSOR,   leftBlack   ? HIGH : LOW);
  digitalWrite(LED_CENTER_SENSOR, centerBlack ? HIGH : LOW);
  digitalWrite(LED_RIGHT_SENSOR,  rightBlack  ? HIGH : LOW);
}

void setupSensorLeds() {
  pinMode(LED_LEFT_SENSOR, OUTPUT);
  pinMode(LED_CENTER_SENSOR, OUTPUT);
  pinMode(LED_RIGHT_SENSOR, OUTPUT);

  displaySensorStates(false, false, false);
}
