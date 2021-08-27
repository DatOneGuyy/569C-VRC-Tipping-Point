// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// inertial5            inertial      5               
// front_left           motor         1               
// front_right          motor         3               
// back_left            motor         2               
// back_right           motor         4               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "auton_library.h"

using namespace vex;

competition Competition;

void pre_auton() {
  vexcodeInit();
  reset(0);
}

void autonomous() {

}

void user_control() {

}

int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(user_control);

  pre_auton();

  while(1) {
    wait(100, msec);
  }
}
