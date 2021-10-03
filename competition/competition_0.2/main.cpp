// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// front_left           motor         11              
// back_left            motor         12              
// front_right          motor         9               
// back_right           motor         10              
// inertial5            inertial      5               
// back_arm             motor         13              
// front_mogo           motor         3               
// four_bar             motor         2               
// four_bar_limit       limit         A               
// back_arm_limit       limit         B               
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

double left_speed = 0;
double right_speed = 0;

int deadzone = 5;

void user_control() {
  while (1) {
    left_speed = (double)map(Controller1.Axis3.position(pct)) * 3 / 25 * 1.05;
    right_speed = (double)map(Controller1.Axis2.position(pct)) * 3 / 25 * 1.05;

    if (std::abs(left_speed) < deadzone * 3 / 25) {
      left_speed = 0;
    }
    if (std::abs(right_speed) < deadzone * 3 / 25) {
      right_speed = 0;
    }

    if (std::abs(left_speed - right_speed) < 0.4) {
      left_speed = daverage(left_speed, right_speed);
      right_speed = left_speed;
    }
    
    front_left.spin(fwd, left_speed, volt);
    back_left.spin(fwd, left_speed, volt);
    front_right.spin(fwd, right_speed, volt);
    back_right.spin(fwd, right_speed, volt);
    
    four_bar.spin(fwd, 12 * ((int)Controller1.ButtonL1.pressing() - (int)Controller1.ButtonL2.pressing() * (int)!four_bar_limit.pressing()), volt);
    back_arm.spin(fwd, 12 * (((int)Controller1.ButtonR1.pressing() * (int)!back_arm_limit.pressing()) - (int)Controller1.ButtonR2.pressing()), volt);
    front_mogo.spin(fwd, 12 * ((int)Controller1.ButtonX.pressing() - (int)Controller1.ButtonA.pressing()), volt);

    if (!Controller1.ButtonL1.pressing() && !Controller1.ButtonL2.pressing()) {
      four_bar.setBrake(brake);
    }
    if (!Controller1.ButtonR1.pressing() && !Controller1.ButtonR2.pressing()) {
      back_arm.setBrake(brake);
    }
    if (!Controller1.ButtonX.pressing() && !Controller1.ButtonA.pressing()) {
      front_mogo.setBrake(brake);
    }

    if (abs(Controller1.Axis2.position()) < 5) {
      front_right.setBrake(brake);
      back_right.setBrake(brake);
    } 
    if (abs(Controller1.Axis3.position()) < 5) {
      back_left.setBrake(brake);
      front_left.setBrake(brake);
    }
    
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
