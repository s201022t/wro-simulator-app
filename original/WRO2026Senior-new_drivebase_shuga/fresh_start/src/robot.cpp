#include "robot.h"
#include "motor.h"

namespace {
void waitForStableLevel(int pin, int level) {
  unsigned long stableSince = millis();
  while (true) {
    encoderPrintSerial();
    if (digitalRead(pin) != level) {
      stableSince = millis();
    } else if (millis() - stableSince >= 30) {
      return;
    }
    delay(1);
  }
}
}

void Robot::begin() {
  if (initialized_) return;
  motorSetup(config_);
  encoderSetup(config_);
  pinMode(startButtonPin_, INPUT_PULLUP);
  initialized_ = true;
}

void Robot::waitForStart() {
  begin();
  stop();
  waitForStableLevel(startButtonPin_, HIGH);
  waitForStableLevel(startButtonPin_, LOW);
}

void Robot::setPower(int left, int right) {
  if (initialized_) motorPower(left, right);
}

void Robot::stop() {
  if (initialized_) motorStop();
}

bool Robot::movePulses(int32_t left, int32_t right, int power, unsigned long timeoutMs) {
  return initialized_ && motorMovePulses(left, right, power, timeoutMs);
}

bool Robot::moveRevolutions(float left, float right, int power, unsigned long timeoutMs) {
  return initialized_ && motorMoveRevolutions(left, right, power, timeoutMs);
}

bool Robot::moveDegrees(float leftDegrees, float rightDegrees, int power, unsigned long timeoutMs) {
  return moveRevolutions(leftDegrees / 360.0f, rightDegrees / 360.0f, power, timeoutMs);
}

EncoderCounts Robot::counts() const {
  return initialized_ ? encoderRead() : EncoderCounts{0, 0};
}

void Robot::resetCounts() {
  if (initialized_) encoderReset();
}

void Robot::printResult(Print& output, bool completed) const {
  const EncoderCounts current = counts();
  output.print(completed ? "TARGET REACHED" : "FAILED / TIMEOUT");
  output.print("  L=");
  output.print(current.left);
  output.print("  R=");
  output.println(current.right);
}
