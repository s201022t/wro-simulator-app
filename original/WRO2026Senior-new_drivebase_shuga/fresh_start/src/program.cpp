#include <Arduino.h>
#include "program.h"
#ifdef DRIVE_FF_CALIBRATION
#include "ff_calibration.h"
#endif

namespace {
constexpr int START_BUTTON_PIN = 20;
const DriveConfig driveConfig{};
}

Robot robot(START_BUTTON_PIN, driveConfig);

void setup() {
  Serial.begin(115200);
  robot.begin();
#ifdef DRIVE_FF_CALIBRATION
  if (Serial) Serial.println("FF calibration: press and release start button. Press again to abort.");
#endif
}

void loop() {
  robot.waitForStart();
#ifdef DRIVE_FF_CALIBRATION
  const bool completed = runFfCalibration(robot, START_BUTTON_PIN);
#else
  const bool completed = runProgram();
#endif
  robot.stop();
  if (Serial) robot.printResult(Serial, completed);
}
