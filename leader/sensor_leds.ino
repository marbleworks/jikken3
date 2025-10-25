#include "pins.h"
#include "sensor_leds.h"
#include "sensors.h"

void displaySensorStates(SensorOrientation orientation,
                         bool leftBlack,
                         bool centerBlack,
                         bool rightBlack) {
  if (orientation == SensorOrientation::Front) {
    digitalWrite(LED_LEFT_SENSOR,   leftBlack   ? HIGH : LOW);
    digitalWrite(LED_CENTER_SENSOR, centerBlack ? HIGH : LOW);
    digitalWrite(LED_RIGHT_SENSOR,  rightBlack  ? HIGH : LOW);
    digitalWrite(LED_BACK_LEFT_SENSOR,  LOW);
    digitalWrite(LED_BACK_RIGHT_SENSOR, LOW);
  } else {
    digitalWrite(LED_LEFT_SENSOR,   LOW);
    digitalWrite(LED_CENTER_SENSOR, LOW);
    digitalWrite(LED_RIGHT_SENSOR,  LOW);
    digitalWrite(LED_BACK_LEFT_SENSOR,  leftBlack  ? HIGH : LOW);
    digitalWrite(LED_BACK_RIGHT_SENSOR, rightBlack ? HIGH : LOW);
  }
}

void setupSensorLeds() {
  pinMode(LED_LEFT_SENSOR, OUTPUT);
  pinMode(LED_CENTER_SENSOR, OUTPUT);
  pinMode(LED_RIGHT_SENSOR, OUTPUT);
  pinMode(LED_BACK_LEFT_SENSOR, OUTPUT);
  pinMode(LED_BACK_RIGHT_SENSOR, OUTPUT);

  displaySensorStates(SensorOrientation::Front, false, false, false);
}
