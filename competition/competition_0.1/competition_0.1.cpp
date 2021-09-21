// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// inertial5            inertial      5               
// front_left           motor         11              
// front_right          motor         13              
// back_left            motor         12              
// back_right           motor         14              
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

int left_speed = 0;
int right_speed = 0;
double drive_kp = 0.3;
double drive_kd = 0.05;
int drive_error = 0;
int previous_drive_error;
int drive_derivative = 0;

void user_control() {
  left_speed = Controller1.Axis1.position(pct);
  right_speed = Controller1.Axis3.position(pct);
  if (std::abs(Controller1.Axis1.position(pct) - Controller1.Axis3.position(pct)) < 5) {
    drive_error = left_speed - right_speed;
    drive_derivative = previous_drive_error - drive_error;
    previous_drive_error = drive_error;
    drive_derivative = 
    right_speed = left_speed;
}

  front_left.spin(fwd, map((double)Controller1.Axis1.position(pct)), pct);
  back_left.spin(fwd, map((double)Controller1.Axis1.position(pct)), pct);
  front_right.spin(fwd, map((double)Controller1.Axis3.position(pct)), pct);
  back_right.spin(fwd, map((double)Controller1.Axis3.position(pct)), pct);
}

int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(user_control);

  pre_auton();

  while(1) {

    user_control();

  }
}
