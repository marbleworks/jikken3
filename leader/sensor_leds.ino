#include "pins.h"
#include "sensor_leds.h"

void displaySensorStates(bool frontLeftBlack,
                         bool frontCenterBlack,
                         bool frontRightBlack,
                         bool rearLeftBlack,
                         bool rearRightBlack) {
  digitalWrite(LED_LEFT_SENSOR,       frontLeftBlack   ? HIGH : LOW);
  digitalWrite(LED_CENTER_SENSOR,     frontCenterBlack ? HIGH : LOW);
  digitalWrite(LED_RIGHT_SENSOR,      frontRightBlack  ? HIGH : LOW);
  digitalWrite(LED_REAR_LEFT_SENSOR,  rearLeftBlack    ? HIGH : LOW);
  digitalWrite(LED_REAR_RIGHT_SENSOR, rearRightBlack   ? HIGH : LOW);
}

void setupSensorLeds() {
  pinMode(LED_LEFT_SENSOR, OUTPUT);
  pinMode(LED_CENTER_SENSOR, OUTPUT);
  pinMode(LED_RIGHT_SENSOR, OUTPUT);
  pinMode(LED_REAR_LEFT_SENSOR, OUTPUT);
  pinMode(LED_REAR_RIGHT_SENSOR, OUTPUT);

  displaySensorStates(false, false, false, false, false);
}
