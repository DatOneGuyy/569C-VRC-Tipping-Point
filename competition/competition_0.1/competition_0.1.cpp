// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// front_left           motor         11              
// back_left            motor         12              
// front_right          motor         13              
// back_right           motor         14              
// inertial5            inertial      5               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "auton_functions.h"

using namespace vex;

competition Competition;

void pre_auton() {
  vexcodeInit();
  reset(0);
}

void autonomous() {

}

int left_speed = 0;
int right_speed = 0;
double drive_kp = 0.3;
double drive_kd = 0.05;
double drive_error = 0;
double previous_drive_error;
double drive_derivative = 0;

void user_control() {
  while (1) {
    left_speed = Controller1.Axis1.position(pct);
    right_speed = Controller1.Axis3.position(pct);

    if (std::abs(Controller1.Axis1.position(pct) - Controller1.Axis3.position(pct)) < 5 && std::abs(Controller1.Axis1.position(pct) > 10)) {
      drive_error = left_speed - right_speed;
      drive_derivative = previous_drive_error - drive_error;
      previous_drive_error = drive_error;
      drive_derivative = 
      right_speed = left_speed;
      left_speed = left_speed - drive_error * drive_kp - drive_derivative * drive_kd;
    }

    front_left.spin(fwd, map(left_speed), pct);
    back_left.spin(fwd, map(left_speed), pct);
    front_right.spin(fwd, map((double)right_speed), pct);
    back_right.spin(fwd, map((double)right_speed), pct);
  }
}

int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(user_control);

  pre_auton();

  while (1) {

    wait(100, msec);

  }
}
