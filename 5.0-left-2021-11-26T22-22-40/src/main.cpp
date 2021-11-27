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

using namespace vex;

void pre_auton() {
}

void autonomous() {

}

double pid_left[4] = {0.32, 0.0244, 0.00, 5};
double pid_right[4] = {0.32, 0.0244, 0.00, 5};

int noaction[2] = {-1, 0};
int actions0[2] = {0, 0};

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  drive(-2700, -2700, pid_left, pid_right, 0, actions0, 1);
  auton_back_arm_up();
  drive(1500, 1500, pid_left, pid_right, 0, noaction, 0);
  //Competition.autonomous(autonomous);
  //Competition.drivercontrol(user_control);

  pre_auton();

  while (1) {
    wait(100, msec);

  }
}
