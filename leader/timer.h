#pragma once

#include <Arduino.h>

class Timer {
public:
  Timer() : startMs(0), active(false) {}

  void start() {
    startMs = millis();
    active = true;
  }

  void reset() {
    active = false;
    startMs = 0;
  }

  bool running() const {
    return active;
  }

  unsigned long elapsed() const {
    return active ? (millis() - startMs) : 0;
  }

private:
  unsigned long startMs;
  bool active;
};

