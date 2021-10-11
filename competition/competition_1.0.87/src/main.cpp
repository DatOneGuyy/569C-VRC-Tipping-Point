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
// intake               motor         7               
// four_bar_limit2      limit         C               
// four_bar_encoder     encoder       E, F            
// front_pneumatics     digital_out   G               
// front_mogo_limit     limit         D               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "functions/auton_functions.h"
#include "control_threads/four_bar.h"
#include "control_threads/back_arm.h"
#include "control_threads/front_mogo.h"
#include "control_threads/pneumatics.h"

using namespace vex;

competition Competition;

void pre_auton() {
  vexcodeInit();
  reset(0);
}

void autonomous() {
  speed_left_curve_reverse(70, 0.8, 400, 0.6, true);
  backward(500, 70, 0.8);
}

double left_speed = 0;
double right_speed = 0;

double deadzone = 5;

void user_control() {
  vex::thread four_bar_thread(four_bar_movement);
  vex::thread back_arm_thread(back_arm_movement);
  vex::thread front_mogo_thread(front_mogo_movement);
  vex::thread pneumatics_thread(pneumatics_movement);
  
  thread(four_bar_thread).detach();
  thread(back_arm_thread).detach();
  thread(front_mogo_thread).detach();
  thread(pneumatics_thread).detach();

  while (1) {

    Brain.Screen.print(0);
    Brain.Screen.newLine();
    
    left_speed = (double)map(Controller1.Axis3.position(pct)) * 3 / 25 * 1.05;
    right_speed = (double)map(Controller1.Axis2.position(pct)) * 3 / 25 * 1.05;

    if (std::abs(left_speed) < deadzone * 3 / 25) {
      left_speed = 0;
    }
    if (std::abs(right_speed) < deadzone * 3 / 25) {
      right_speed = 0;
    }

    if (Controller1.ButtonRight.pressing()) {
      autonomous();
    }

    if (std::abs(left_speed - right_speed) < 0.4) {
      left_speed = daverage(left_speed, right_speed);
      right_speed = left_speed;
    }
    
    front_left.spin(fwd, left_speed, volt);
    back_left.spin(fwd, left_speed, volt);
    front_right.spin(fwd, right_speed, volt);
    back_right.spin(fwd, right_speed, volt);

    if (abs(Controller1.Axis2.position()) < 5) {
      front_right.setBrake(brake);
      back_right.setBrake(brake);
    } 
    if (abs(Controller1.Axis3.position()) < 5) {
      back_left.setBrake(brake);
      front_left.setBrake(brake);
    }

    //intake.spin(fwd, 12 * ((int)Controller1.ButtonRight.pressing() - (int)Controller1.ButtonDown.pressing()), volt);
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
