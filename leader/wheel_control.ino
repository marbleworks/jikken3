#include "wheel_control.h"
#include "pins.h"

extern int MAX_PWM;
extern int MIN_PWM;

namespace {
void drivePins(int IN1, int IN2, bool cw) {
  digitalWrite(IN1, cw ? HIGH : LOW);
  digitalWrite(IN2, cw ? LOW  : HIGH);
}

void setWheel(int speedSigned, uint8_t in1, uint8_t in2, uint8_t pwmPin) {
  int spd = constrain(abs(speedSigned), MIN_PWM, MAX_PWM);
  if (speedSigned > 0)      drivePins(in1, in2, true);
  else if (speedSigned < 0) drivePins(in1, in2, false);
  else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
  analogWrite(pwmPin, spd);
}
}  // namespace

void setupWheelPins() {
  pinMode(A_IN1, OUTPUT);
  pinMode(A_IN2, OUTPUT);
  pinMode(A_PWM, OUTPUT);
  pinMode(B_IN1, OUTPUT);
  pinMode(B_IN2, OUTPUT);
  pinMode(B_PWM, OUTPUT);

  digitalWrite(A_IN1, LOW);
  digitalWrite(A_IN2, LOW);
  digitalWrite(B_IN1, LOW);
  digitalWrite(B_IN2, LOW);
  analogWrite(A_PWM, 0);
  analogWrite(B_PWM, 0);
}

void setWheelA(int speedSigned) {
  setWheel(speedSigned, A_IN1, A_IN2, A_PWM);
}

void setWheelB(int speedSigned) {
  setWheel(speedSigned, B_IN1, B_IN2, B_PWM);
}

void setWheels(int leftSpeed, int rightSpeed) {
  if (LEFT_IS_A) {
    setWheelA(leftSpeed);
    setWheelB(rightSpeed);
  } else {
    setWheelA(rightSpeed);
    setWheelB(leftSpeed);
  }
}
