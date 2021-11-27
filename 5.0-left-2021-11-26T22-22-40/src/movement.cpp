#include <cmath>
#include "vex.h"

using namespace vex;

void auton_back_arm_down() {
  
  back_arm.stop(hold);
}

void auton_back_arm_up() {
  while (!back_arm_limit.pressing()) {
    back_arm.spin(fwd, 12.0, volt);
    if (back_arm.position(degrees) > -300) {
      break;
    }
  }
  back_arm.stop(hold);
}

void auton_four_bar_down() {
  while (!four_bar_limit.pressing()) {
    four_bar.spin(fwd, -12.0, volt);
  }
  four_bar.stop(hold);
}

void auton_four_bar_up() {
  while (!four_bar_limit2.pressing()) {
    four_bar.spin(fwd, 12.0, volt);
  }
  four_bar.stop(hold);
}

void run_action(double current_percentage, double percentage, int index) {
  Brain.Screen.printAt(300, 100, "%f, %f", current_percentage * 100, percentage);
  if (current_percentage * 100 >= percentage) {
    switch (index) {
      case 0: {
        back_arm.spinToPosition(-640, degrees, false);
        break;
      }
      case 1: {
        thread ABAU(auton_back_arm_down);
        ABAU.detach();
        break;
      }
      case 2: {
        thread AFBD(auton_four_bar_down);
        AFBD.detach();
        break;
      }
      case 3: {
        thread AFBU(auton_four_bar_down);
        AFBU.detach();
        break;
      }
      case 4: {

        break;
      }
    }
  }
}

double r(double input) {return std::round(input * 100) / 100;}
double s(double input) {return input == 0 ? 0 : std::abs(input) / input;}
double c(double input, double max, double min) {if (input < min) {return min;} else if (input > max) {return max;} else {return input;}}

void drive(double target_left, double target_right, double pid_left[], double pid_right[], double gyro_target, int actions[], int action_count) {
  leftdrive.resetPosition();
  rightdrive.resetPosition();

  int threshold = 4;
  int threshold_counter = 0;

  double slew_left = 4.1;
  double slew_right = 4.1;
  int slew_counter_left = 0;  
  int slew_counter_right = 0;
  bool slew_left_enabled = true;
  bool slew_right_enabled = true;

  double error_left = 0;
  double error_right = 0;
  double percent_left = 0;
  double percent_right = 0;

  double integral_left = 0;
  double integral_right = 0;

  double derivative_left = 0;
  double derivative_right = 0;

  double gyro_error = 0;

  double power_left = slew_left + 1;
  double power_right = slew_right + 1;
  double power_left_pid = 100;
  double power_right_pid = 100;

  double decel = 10;

  int i = 0;
  
  while (threshold_counter < 30) {
    i = 0;
    if (std::abs(leftdrive.position(degrees) - target_left) < threshold && std::abs(rightdrive.position(degrees) - target_right) < threshold) {
      threshold_counter++;
    } else {
      threshold_counter = 0;
    }

    slew_counter_left++;
    slew_counter_right++;

    derivative_left = target_left - leftdrive.position(degrees) - error_left;
    derivative_right = target_right - rightdrive.position(degrees) - error_right;
    
    error_left = target_left - leftdrive.position(degrees);
    error_right = target_right - rightdrive.position(degrees);
    percent_left = (std::abs((error_left - target_left) / target_left));
    percent_right = (std::abs((error_right - target_right) / target_right));

    if (std::abs(error_left) < 200) {
      integral_left += error_left;
    }
    if (std::abs(error_right) < 200) {
      integral_right += error_right;
    }
    if (s(target_left) == s(target_right)) {
      gyro_error = inertial0.rotation(degrees) - gyro_target;
    } else {
      gyro_error = 0;
    }
    
    power_left_pid = error_left * pid_left[0] + integral_left * pid_left[1] + derivative_left * pid_left[2];
    power_left_pid = power_left_pid - gyro_error * pid_left[3];
    power_left_pid = power_left_pid * (1 - std::pow(percent_left, decel));

    power_right_pid = error_right * pid_right[0] + integral_right * pid_right[1] + derivative_right * pid_right[2];
    power_right_pid = power_right_pid + gyro_error * pid_right[3];
    power_right_pid = power_right_pid * (1 - std::pow(percent_right, decel));

    if (slew_counter_left * slew_left <= std::abs(power_left_pid) && slew_left_enabled) {
      power_left = slew_counter_left * slew_left;
      power_left = s(error_left) * power_left;
      power_left = power_left - gyro_error * pid_left[3];
      Brain.Screen.printAt(300, 20, "Stage: Slew");
    } else {
      slew_left_enabled = false;
      power_left = power_left_pid;
      Brain.Screen.printAt(300, 20, "Stage: PID ");
    }
    if (slew_counter_right * slew_right <= std::abs(power_right_pid) && slew_right_enabled) {
      power_right = slew_counter_right * slew_right;
      power_right = s(error_right) * power_right;
      power_right = power_right + gyro_error * pid_right[3];
    } else {
      slew_right_enabled = false;
      power_right = power_right_pid;
    }

    leftdrive.spin(fwd, power_left, pct);
    rightdrive.spin(fwd, power_right, pct);

    Brain.Screen.printAt(10, 20, "Left error: %f", r(error_left));
    Brain.Screen.printAt(10, 40, "Right error: %f", r(error_right));
    Brain.Screen.printAt(10, 60, "Gyro error: %f, %f", r(gyro_error * pid_left[3]), r(gyro_error * pid_right[3]));
    Brain.Screen.printAt(10, 80, "Error KP: %f", (error_left * pid_left[0]));
    Brain.Screen.printAt(10, 100, "Slew counter left: %d", slew_counter_left);
    Brain.Screen.printAt(10, 120, "Slew counter right: %d", slew_counter_right);
    Brain.Screen.printAt(10, 140, "Integral: %f", integral_left * pid_left[1]);
    Brain.Screen.printAt(10, 160, "Left speed: %f", r(power_left));
    Brain.Screen.printAt(10, 180, "Right speed, %f", r(power_right));
    Brain.Screen.printAt(300, 40, "Actions: %d", action_count);
    
    Brain.Screen.drawPixel(slew_counter_left, 240 - std::abs(power_left));
    
    while (i < action_count) {
      Brain.Screen.printAt(300, 80, "Indices: %d, %d", i, actions[i * 2]);
      run_action(percent_left, actions[i * 2], actions[i * 2 + 1]);
      i++;
    }

    wait(10, msec);
  }

  leftdrive.stop(brake);
  rightdrive.stop(brake);
}