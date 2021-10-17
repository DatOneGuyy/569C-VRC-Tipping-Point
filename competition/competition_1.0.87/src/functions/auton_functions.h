#pragma once

#include "vex.h"
#include "math_library.h"
#include <iostream>

using namespace vex;

void forward_(int dist, int speed) 
{
  Drivetrain.driveFor(directionType::fwd, dist, distanceUnits::in, speed, velocityUnits::pct);
}

void move_back_arm(int dist, bool dir) {
  while (back_arm.position(degrees) > 5 * dist) {
    back_arm.spin(fwd, dir * 12.0, volt);
  }
}

void move_back_arm_up(int dist, bool dir) {
  while (back_arm.position(degrees) > 5 * dist) {
    back_arm.spin(fwd, dir * 12.0, volt);
  }
}



void backward(int dist, int speed)
{
  Drivetrain.driveFor(directionType::rev, dist, distanceUnits::in, speed, velocityUnits::pct);
}

void turnLeft(int angle, int speed)
{
  Drivetrain.turn(turnType::left, speed, velocityUnits::pct);
}

void turnRight(int angle, int speed)
{
  Drivetrain.turn(turnType::right, speed, velocityUnits::pct);
}

/*
void drive_forward(double distance, double speed, double degree) {
  reset(0);

  double left_motor_power;
  double right_motor_power = speed;
  double modifier;

  double kp = 0.03;
  double ki = 0.0;
  double kd = 0.0;
  double kg = 1.0;

  double left_position = std::max(front_left.position(deg), back_left.position(deg));
  double right_position = std::max(front_right.position(deg), back_right.position(deg));

  double error;
  double integral = 0;
  double derivative;
  double last_error;

  while (daverage(left_position, right_position) <= distance) {

    left_position = std::max(front_left.position(deg), back_left.position(deg));
    right_position = std::max(front_right.position(deg), back_right.position(deg));

    modifier = curve(degree, 100 * daverage(left_position, right_position) / distance) * speed / 100;

    error = left_position - right_position;
    integral += error;
    derivative = error - last_error;
    last_error = error; 

    left_motor_power = modifier;
    right_motor_power = modifier + error * kp + integral * ki + derivative * kd - kg * angleConvert(inertial5.angle(deg));

    front_right.spin(fwd, right_motor_power, pct);
    back_right.spin(fwd, right_motor_power, pct);
    front_left.spin(fwd, left_motor_power, pct);
    back_left.spin(fwd, left_motor_power, pct);
  }

  front_right.stop(brake);
  back_right.stop(brake);
  front_left.stop(brake);
  back_left.stop(brake);
  
}

void backward(double distance, double speed, double degree) {
  reset(0);

  while (-back_left.position(deg) < distance) {
    Brain.Screen.print(speed);

    front_left.spin(vex::directionType::rev, speed, pct);
    front_right.spin(vex::directionType::rev, speed, pct);
    back_left.spin(vex::directionType::rev, speed, pct);
    back_right.spin(vex::directionType::rev, speed, pct);
  }

  front_right.stop(vex::brakeType::hold);
  back_right.stop(vex::brakeType::hold);
  front_left.stop(vex::brakeType::hold);
  back_left.stop(vex::brakeType::hold);

}

void backward_(double distance, double speed, double degree) {
  reset(0);

  double left_motor_power;
  double right_motor_power = -speed;
  double modifier;

  double kp = 0.15;
  double ki = 0.0;
  double kd = 0.0;
  double kg = 0.2;

  double left_position = std::min(front_left.position(deg), back_left.position(deg));
  double right_position = std::min(front_right.position(deg), back_right.position(deg));

  double error;
  double integral = 0;
  double derivative;
  double last_error;

  inertial5.resetHeading();

  Brain.Screen.print(std::abs(daverage(left_position, right_position)) <= distance);

  while (std::abs(daverage(left_position, right_position)) <= distance) {

    left_position = std::min(front_left.position(deg), back_left.position(deg));
    right_position = std::min(front_right.position(deg), back_right.position(deg));

    modifier = -curve(degree, 100 * std::abs(daverage(left_position, right_position)) / distance) * speed / 100;

    error = left_position - right_position;
    integral += error;
    derivative = error - last_error;
    last_error = error; 

    left_motor_power = speed;
    right_motor_power = speed;// + error * kp;//kg * angleConvert(inertial5.heading());.

    Brain.Screen.print(speed);

    if (left_motor_power > 0 || right_motor_power > 0) {
      break;
    }

    Brain.Screen.print("k");

    front_right.spin(fwd, right_motor_power, pct);
    back_right.spin(fwd, right_motor_power, pct);
    front_left.spin(fwd, left_motor_power, pct);
    back_left.spin(fwd, left_motor_power, pct);

    wait(10, msec);
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
  double kd = 0.0;

  double left_position = std::min(front_left.position(deg), back_left.position(deg));
  double right_position = std::max(front_right.position(deg), back_right.position(deg));

  double error;
  double integral = 0;
  double derivative;
  double last_error;

  while (daverage(std::abs(left_position), right_position) <= distance) {

    left_position = std::min(front_left.position(deg), back_left.position(deg));
    right_position = std::max(front_right.position(deg), back_right.position(deg));

    modifier = curve(degree, 100 * daverage(std::abs(left_position), right_position) / distance) * speed / 100;

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

  while (daverage(left_position, std::abs(right_position)) <= distance) {

    left_position = std::max(front_left.position(deg), back_left.position(deg));
    right_position = std::min(front_right.position(deg), back_right.position(deg));

    modifier = curve(degree, 100 * daverage(left_position, std::abs(right_position)) / distance) * speed / 100;

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

void speed_left_curve_reverse(double speed, double degree, double distance, double ratio, bool in_sequence) {
  reset(0);

  double left_motor_power;
  double right_motor_power = speed;
  double modifier;

  double kp = 0.05;
  double ki = 0.003;
  double kd = 0.02;

  double left_position = std::min(front_left.position(deg), back_left.position(deg));
  double right_position = std::min(front_right.position(deg), back_right.position(deg));

  double error;
  double integral = 0;
  double derivative;
  double last_error;

  while (left_position >= -distance) {

    left_position = std::max(front_left.position(deg), back_left.position(deg));
    right_position = std::max(front_right.position(deg), back_right.position(deg));

    modifier = !in_sequence ? curve(degree, 100 * -daverage(left_position, right_position) / distance) * speed / 100 : linear_accel(-daverage(left_position, right_position)) * speed / 100;
    modifier = linear_accel(-daverage(left_position, right_position)) * speed / 100;

    error = std::abs(right_position) - left_position;
    integral += error;
    derivative = error - last_error;
    last_error = error; 

    left_motor_power = -modifier;
    right_motor_power = -modifier * ratio + error * kp;// + integral * ki + derivative * kd;

    front_right.spin(fwd, right_motor_power, pct);
    back_right.spin(fwd, right_motor_power, pct);
    front_left.spin(fwd, left_motor_power, pct);
    back_left.spin(fwd, left_motor_power, pct);

    wait(10, msec);
  }

  front_right.stop();
  back_right.stop();
  front_left.stop();
  back_left.stop();
}*/