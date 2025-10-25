#pragma once

enum class SensorOrientation;

void setupSensorLeds();
void displaySensorStates(SensorOrientation orientation,
                         bool leftBlack,
                         bool centerBlack,
                         bool rightBlack);
