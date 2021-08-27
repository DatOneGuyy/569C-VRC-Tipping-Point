#include "vex.h"
#include "math_library.h"

using namespace vex;

void reset(int type) {
  switch (type) {
    case 1:
      front_left.resetRotation();
      front_right.resetRotation();
      back_left.resetRotation();
      back_right.resetRotation();
    default:
      front_left.resetRotation();
      front_right.resetRotation();
      back_left.resetRotation();
      back_right.resetRotation();
      inertial5.angle(deg);
      break;
  }
}

void forward(double distance, double speed) {
  reset(0);

  double left_motor_power;
  double right_motor_power;
  double modifier;

  double kp = 0.25;
  double ki = 0.02;
  double kd = 0.03;
  double kg = 0.01;

  double left_position = max(front_left.position(deg), back_left.position(deg));
  double right_position = max(front_right.position(deg), back_left.position(deg));

  double error;
  double integral = 0;
  double derivative;
  double last_error;

  while (average(left_position, right_position) <= distance) {
    modifier = curve(2, 100 * average(left_position, right_position) / distance) * speed / 100;

    error = left_position - right_position;
    integral += error;
    derivative = error - last_error;
    last_error = error; 

    left_motor_power = modifier;
    right_motor_power = modifier * (right_motor_power + error * kp + integral * ki + derivative * kd - kg * inertial5.angle(deg));

    front_right.spin(fwd, right_motor_power, pct);
    back_right.spin(fwd, right_motor_power, pct);
    front_left.spin(fwd, right_motor_power, pct);
    back_left.spin(fwd, right_motor_power, pct);
  }

  front_right.stop();
  back_right.stop();
  front_left.stop();
  back_left.stop();
  
}
