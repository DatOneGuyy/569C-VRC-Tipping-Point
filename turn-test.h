#include "vex.h"
#include "robot-config.h"
#include <cmath>

using namespace vex;

double map(double x, double n) {
  return pow(1 - pow(x, n), 1 / n);
}

void turn_left(double angle, double speed) {
  inertial5.resetRotation();
  while (inertial5.rotation(degrees) > angle) {
    left_drive.spin(fwd, -speed * map(std::abs(inertial5.rotation(degrees) / angle), 4), rpm);
    right_drive.spin(fwd, speed * map(std::abs(inertial5.rotation(degrees) / angle), 4), rpm);
  }
  left_drive.stop(brake);
  right_drive.stop(brake);
}

void turn_right(double angle, double speed) {
  inertial5.resetRotation();
  while (inertial5.rotation(degrees) < angle) {
    left_drive.spin(fwd, speed * map(std::abs(inertial5.rotation(degrees) / angle), 4), rpm);
    right_drive.spin(fwd, -speed * map(std::abs(inertial5.rotation(degrees) / angle), 4), rpm);
  }
  left_drive.stop(brake);
  right_drive.stop(brake);
}
