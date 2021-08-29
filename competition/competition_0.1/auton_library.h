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

void forward(double distance, double speed, double degree) {
  reset(0);

  double left_motor_power;
  double right_motor_power = speed;
  double modifier;

  double kp = 0.25;
  double ki = 0.02;
  double kd = 0.03;
  double kg = 0.01;

  double left_position = std::max(front_left.position(deg), back_left.position(deg));
  double right_position = std::max(front_right.position(deg), back_right.position(deg));

  double error;
  double integral = 0;
  double derivative;
  double last_error;

  while (average(left_position, right_position) <= distance) {

    left_position = std::max(front_left.position(deg), back_left.position(deg));
    right_position = std::max(front_right.position(deg), back_right.position(deg));

    modifier = curve(degree, 100 * average(left_position, right_position) / distance) * speed / 100;

    error = left_position - right_position;
    integral += error;
    derivative = error - last_error;
    last_error = error; 

    left_motor_power = modifier;
    right_motor_power = modifier + error * kp + integral * ki + derivative * kd - kg * inertial5.angle(deg);

    front_right.spin(fwd, right_motor_power, pct);
    back_right.spin(fwd, right_motor_power, pct);
    front_left.spin(fwd, left_motor_power, pct);
    back_left.spin(fwd, left_motor_power, pct);
  }

  front_right.stop();
  back_right.stop();
  front_left.stop();
  back_left.stop();
  
}

void backward(double distance, double speed, double degree) {
  reset(0);

  double left_motor_power;
  double right_motor_power = speed;
  double modifier;

  double kp = 0.25;
  double ki = 0.02;
  double kd = 0.03;
  double kg = 0.01;

  double left_position = std::min(front_left.position(deg), back_left.position(deg));
  double right_position = std::min(front_right.position(deg), back_right.position(deg));

  double error;
  double integral = 0;
  double derivative;
  double last_error;

  while (std::abs(average(left_position, right_position)) <= distance) {

    left_position = std::min(front_left.position(deg), back_left.position(deg));
    right_position = std::min(front_right.position(deg), back_right.position(deg));

    modifier = -curve(degree, 100 * std::abs(average(left_position, right_position)) / distance) * speed / 100;

    error = left_position - right_position;
    integral += error;
    derivative = error - last_error;
    last_error = error; 

    left_motor_power = modifier;
    right_motor_power = modifier + error * kp + integral * ki + derivative * kd - kg * inertial5.angle(deg);

    front_right.spin(fwd, right_motor_power, pct);
    back_right.spin(fwd, right_motor_power, pct);
    front_left.spin(fwd, left_motor_power, pct);
    back_left.spin(fwd, left_motor_power, pct);
  }

  front_right.stop();
  back_right.stop();
  front_left.stop();
  back_left.stop();
  
}

void left(double distance, double speed, double degree) {
  reset(0);

  double left_motor_power;
  double right_motor_power = speed;
  double modifier;

  double kp = 0.25;
  double ki = 0.02;
  double kd = 0.03;

  double left_position = std::min(front_left.position(deg), back_left.position(deg));
  double right_position = std::max(front_right.position(deg), back_right.position(deg));

  double error;
  double integral = 0;
  double derivative;
  double last_error;

  while (average(std::abs(left_position), right_position) <= distance) {

    left_position = std::min(front_left.position(deg), back_left.position(deg));
    right_position = std::max(front_right.position(deg), back_right.position(deg));

    modifier = curve(degree, 100 * average(std::abs(left_position), right_position) / distance) * speed / 100;

    error = std::abs(left_position) - right_position;
    integral += error;
    derivative = error - last_error;
    last_error = error; 

    left_motor_power = -modifier;
    right_motor_power = modifier + error * kp + integral * ki + derivative * kd;

    front_right.spin(fwd, right_motor_power, pct);
    back_right.spin(fwd, right_motor_power, pct);
    front_left.spin(fwd, left_motor_power, pct);
    back_left.spin(fwd, left_motor_power, pct);
  }

  front_right.stop();
  back_right.stop();
  front_left.stop();
  back_left.stop();
  
}

void right(double speed, double distance, double degree) {
  reset(0);

  double left_motor_power;
  double right_motor_power = speed;
  double modifier;

  double kp = 0.25;
  double ki = 0.02;
  double kd = 0.03;

  double left_position = std::max(front_left.position(deg), back_left.position(deg));
  double right_position = std::min(front_right.position(deg), back_right.position(deg));

  double error;
  double integral = 0;
  double derivative;
  double last_error;

  while (average(left_position, std::abs(right_position)) <= distance) {

    left_position = std::max(front_left.position(deg), back_left.position(deg));
    right_position = std::min(front_right.position(deg), back_right.position(deg));

    modifier = curve(degree, 100 * average(left_position, std::abs(right_position)) / distance) * speed / 100;

    error = std::abs(right_position) - left_position;
    integral += error;
    derivative = error - last_error;
    last_error = error; 

    left_motor_power = modifier;
    right_motor_power = -modifier + error * kp + integral * ki + derivative * kd;

    front_right.spin(fwd, right_motor_power, pct);
    back_right.spin(fwd, right_motor_power, pct);
    front_left.spin(fwd, left_motor_power, pct);
    back_left.spin(fwd, left_motor_power, pct);
  }

  front_right.stop();
  back_right.stop();
  front_left.stop();
  back_left.stop();
  
}
