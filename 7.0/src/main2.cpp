/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\IL NAMKUNG                                       */
/*    Created:      Sat Oct 16 2021                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Vision17             vision        17              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <cmath>

using namespace vex;

competition Competition;

vex::thread timeout_thread0;
vex::thread timeout_thread1;

int TIMEOUT0 = 0;
bool TIMED_OUT0 = false;
int front_arm_state = false;
void reset_timeout0() {
  TIMED_OUT0 = false;
  TIMEOUT0 = 0;
}
int timeout0() {
  TIMED_OUT0 = false;
  this_thread::sleep_for(TIMEOUT0);
  TIMED_OUT0 = true;
  return 0;
}
bool activate = false;
int four_bar_movement() {
  bool L1 = false;
  bool L1p = false;

  bool arm_state = false;

  while (1) {
    L1 = Controller1.ButtonL1.pressing();
    front_arm_state = arm_state;

    if (!L1 && L1p && !arm_state) {
      reset_timeout0();
      TIMEOUT0 = 2500;
      timeout_thread0 = thread(timeout0);
      timeout_thread0.detach();

      arm_state = true;

      while (!four_bar_limit2.pressing()) {
        four_bar.spin(fwd, 12.0, volt);
        four_bar2.spin(fwd, 12.0, volt);

        if (TIMED_OUT0) {
          timeout_thread0.interrupt();
          reset_timeout0();
          break;
        }
      }

      timeout_thread0.interrupt();
      reset_timeout0();

      four_bar.stop(hold);
      four_bar2.stop(hold);
    } else if (!L1 && L1p && arm_state) {
      reset_timeout0();
      TIMEOUT0 = 2500;
      timeout_thread0 = thread(timeout0);
      timeout_thread0.detach();

      arm_state = false;

      while (!four_bar_limit.pressing()) {
        four_bar.spin(fwd, -12.0, volt);
        four_bar2.spin(fwd, -12.0, volt);

        if (TIMED_OUT0) {
          timeout_thread0.interrupt();
          reset_timeout0();
          break;
        }
      }

      timeout_thread0.interrupt();
      reset_timeout0();

      four_bar.stop(hold);
      four_bar2.stop(hold);
    }

    if (!arm_state && activate && !four_bar_limit.pressing()) {
      four_bar.spin(fwd, 0, volt);
      four_bar2.spin(fwd, 0, volt);
    }

    L1p = L1;
    
    this_thread::sleep_for(10);
  }
  return 0;
}

int TIMEOUT1 = 0;
bool TIMED_OUT1 = false;
void reset_timeout1() {
  TIMED_OUT1 = false;
  TIMEOUT1 = 0;
}
int timeout1() {
  TIMED_OUT1 = false;
  this_thread::sleep_for(TIMEOUT1);
  TIMED_OUT1 = true;
  return 0;
}
int back_arm_movement() {
  bool L2 = false;
  bool L2p = false;
  
  bool arm_state = true;

  int movements = 0;

  while (1) {
    L2 =  Controller1.ButtonR2.pressing();
    if (!L2 && L2p && !arm_state) {
      reset_timeout1();
      TIMEOUT1 = 2500;
      timeout_thread1 = thread(timeout1);
      timeout_thread1.detach();

      arm_state = true;

      while (!back_arm_limit.pressing()) {
        back_arm.spin(fwd, 12.0, volt);
        if (back_arm.position(degrees) > -200) {
          break;
        }
        if (TIMED_OUT1) {
          timeout_thread1.interrupt();
          reset_timeout1();
          break;
        }
      }

      back_arm.stop(hold);
    } else if (!L2 && L2p && arm_state) {
      reset_timeout1();
      TIMEOUT1 = 2500;
      timeout_thread1 = thread(timeout1);

      arm_state = false;

      while (back_arm.position(degrees) > -620) {

        back_arm.spin(fwd, -12.0, volt);

        if (TIMED_OUT1) {
          timeout_thread1.interrupt();
          reset_timeout1();
          break;
        }
      }

      movements++;
      back_arm.stop(hold);
    }

    L2p = Controller1.ButtonR2.pressing();
    Brain.Screen.print(back_arm.position(degrees));
    Brain.Screen.newLine();

    this_thread::sleep_for(10);
  }
  return 0;
}

int pneumatics_movement() {
  bool R1 = false;
  bool R1p = false;

  front_pneumatics.set(false);

  while (1) {
    R1 = Controller1.ButtonR1.pressing();

    if (R1 && !R1p) {
      front_pneumatics.set(!front_pneumatics.value());
    }
    
    R1p = Controller1.ButtonR1.pressing();
  }
  return 0;
}

int controller_timer() {
  double seconds = 105;

  Controller1.Screen.clearScreen();

  while (true) {
    seconds--;
    wait(1, sec);
    Controller1.Screen.clearLine();
    Controller1.Screen.print(seconds);
    if (seconds == 50) {
      Controller1.rumble("-----");
    }

    if (seconds == 30) {
      Controller1.rumble("-.-.-");
    }
  }
}

double r(double input) {return std::round(input * 100) / 100;}
double s(double input) {return input == 0 ? 0 : std::abs(input) / input;}
double c(double input, double max, double min) {if (input < min) {return min;} else if (input > max) {return max;} else {return input;}}

void drive(double target, double pid_left[], double pid_right[]) {
  leftdrive.resetPosition();
  rightdrive.resetPosition();
  double gyro_angle = inertial0.rotation();

  int threshold = 4;
  int threshold_counter = 0;

  double slew_left = 1.3;
  double slew_right = 1.3;
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
  
  while (threshold_counter < 30) {
    if (std::abs(leftdrive.position(degrees) - target) < threshold && std::abs(rightdrive.position(degrees) - target) < threshold) {
      threshold_counter++;
    } else {
      threshold_counter = 0;
    }

    slew_counter_left++;
    slew_counter_right++;

    derivative_left = target - leftdrive.position(degrees) - error_left;
    derivative_right = target - rightdrive.position(degrees) - error_right;
    
    error_left = target - leftdrive.position(degrees);
    error_right = target - rightdrive.position(degrees);
    percent_left = (std::abs((error_left - target) / target));
    percent_right = (std::abs((error_right - target) / target));

    if (std::abs(error_left) < 200) {
      integral_left += error_left;
    }
    if (std::abs(error_right) < 200) {
      integral_right += error_right;
    }

    gyro_error = inertial0.rotation(degrees) - gyro_angle;
    
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
      power_right = s(error_left) * power_right;
      power_right = power_right + gyro_error * pid_right[3];
    } else {
      slew_right_enabled = false;
      power_right = power_right_pid;
    }

    /*
    power_left = fmin(slew_counter_left * slew_left, std::abs(power_left));
    power_right = fmin(slew_counter_right * slew_right, std::abs(power_right));

    power_left = s(target) * power_left - gyro_error * pid_left[3];
    power_right = s(target) * power_right + gyro_error * pid_right[3];

    power_left = error_left * pid_left[0] + integral_left * pid_left[1] + derivative_left * pid_left[2];
    power_right = error_right * pid_right[0] + integral_right * pid_right[1] + derivative_right * pid_right[2];
    */

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
    
    Brain.Screen.drawPixel(slew_counter_left, 240 - std::abs(power_left));

    wait(10, msec);
  }

  leftdrive.stop(brake);
  rightdrive.stop(brake);
}


void pre_auton() {
}

void autonomous() {

}

bool intake_active = false;
bool R2, R2p = false;

double pid_left[4] = {0.32, 0.0061, 0.00, 2};
double pid_right[4] = {0.32, 0.0122, 0.00, 2};

void user_control() {
  activate = true;
  //vex::thread four_bar_thread(four_bar_movement);
  vex::thread back_arm_thread(back_arm_movement);
  vex::thread pneumatics_thread(pneumatics_movement);
  if (Competition.isFieldControl()) {
    vex::thread controller_timer_thread(controller_timer);
    thread(controller_timer_thread).detach();
  }

  //thread(four_bar_thread).detach();
  thread(back_arm_thread).detach();
  thread(pneumatics_thread).detach();

  front_pneumatics.set(false);
  
  while (1) {
    activate = true;
    
    R2 = Controller1.ButtonX.pressing();

    if (R2 && !R2p) {
      intake_active = !intake_active;
    }

    four_bar.spin(fwd, 12 * (Controller1.ButtonL1.pressing() - Controller1.ButtonL2.pressing()), volt);
    four_bar2.spin(fwd, 12 * (Controller1.ButtonL1.pressing() - Controller1.ButtonL2.pressing()), volt);

    intake.spin(fwd, 12.0 * intake_active, volt);
  }
  /*
    if (Controller1.ButtonLeft.pressing()) {
      back_arm.spin(fwd, -12.0, volt);
    } else if (Controller1.ButtonUp.pressing()) {
      back_arm.spin(fwd, 12.0, volt);
    } else {
      back_arm.stop(hold);
    }*/

    R2p = R2;
    wait(10, msec);
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  drive(-1000, pid_left, pid_right);
  drive(1500, pid_left, pid_right);
  drive(-500, pid_left, pid_right);
  //Competition.autonomous(autonomous);
  //Competition.drivercontrol(user_control);

  pre_auton();

  while (1) {
    wait(100, msec);

  }
}
