// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Drivetrain           drivetrain    11, 12, 9, 10, 5
// Controller1          controller                    
// back_arm             motor         13              
// front_mogo           motor         3               
// four_bar             motor         2               
// intake               motor         7               
// four_bar_limit       limit         A               
// back_arm_limit       limit         B               
// four_bar_limit2      limit         C               
// four_bar_encoder     encoder       E, F            
// front_pneumatics     digital_out   G               
// front_mogo_limit     limit         D               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "functions/auton_functions.h"
#include "control_threads/four_bar.h"
#include "control_threads/back_arm_timeout.h"
//#include "control_threads/back_arm.h"
#include "control_threads/front_mogo.h"
#include "control_threads/pneumatics.h"
#include "back_arm_auton.h"
#include "four_bar_auton.h"
#include <iostream> 

using namespace vex;

competition Competition;

int auton1 = 0;

int back_arm_movement() {
  bool L2 = false;
  bool L2p = false;
  
  bool arm_state = true;

  int movements = 0;
  
  double error = 0;
  double kp = 0.5;

  back_arm.setPosition(0, degrees);

  while (1) {
    L2 = Controller1.ButtonL2.pressing();

    if (!L2 && L2p && !arm_state) {
      reset_timeout1();
      TIMEOUT1 = 2500;
      timeout_thread1 = thread(timeout1);
      timeout_thread1.detach();

      arm_state = true;

      while (back_arm.position(degrees) < -800) {
        back_arm.spin(fwd, 12.0, volt);

        if (TIMED_OUT1) {
          timeout_thread1.interrupt();
          reset_timeout1();
          break;
        }
      }

      back_arm.stop(brake);
    } else if (!L2 && L2p && arm_state) {
      reset_timeout1();
      TIMEOUT1 = 2500;
      timeout_thread1 = thread(timeout1);

      arm_state = false;

      while (back_arm.position(degrees) > (-1770 + movements * 5)) {

        back_arm.spin(fwd, -12.0, volt);

        if (TIMED_OUT1) {
          timeout_thread1.interrupt();
          reset_timeout1();
          break;
        }
      }

      movements++;
      back_arm.stop(brake);
    }

    if (arm_state && !back_arm_limit.pressing() && movements > 0) {
      error = -800 - back_arm.position(degrees);
      back_arm.spin(fwd, error * kp, pct);
    }

    L2p = Controller1.ButtonL2.pressing();

    this_thread::sleep_for(25);
  }
  return 0;
}

void pre_auton() {
  vexcodeInit();
}


//Right Side Auton
void right_auton() 
{
  //move_back_arm(-1700, false);
  /*
  auton_back_arm_position = -1700;
  move0 = true;
  wait(2000, msec);
  move0 = true;
  auton_back_arm_position = -800;*/
  back_arm.spin(fwd, -12.0, volt);
  wait(500, msec);
  back_arm.stop(brakeType :: brake);
  wait(500, msec);
  //backward(80, 80);
 // move_back_arm(-800, true);
 // back_arm.stop(brake);
 // back_arm.spin(fwd, 12.0, volt);
 // wait(1000, msec);
  //back_arm.stop(brake);
}


void left_auton() {

}

void autonomous() {
  vex::thread back_arm_auton_thread(back_arm_auton);
  back_arm_auton_thread.detach();

  switch (auton1) {
    case 0:
      right_auton();
      break;
    case 1: 
      left_auton();
      break;
    default:
      right_auton();
      break;
  }
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
    if (Controller1.ButtonRight.pressing()) {
      autonomous();
    }
    /*
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
    
    leftMotorA.spin(fwd, left_speed, volt);
    leftMotorB.spin(fwd, left_speed, volt);
    rightMotorA.spin(fwd, right_speed, volt);
    rightMotorB.spin(fwd, right_speed, volt);

    if (abs(Controller1.Axis2.position()) < 5) {
      leftMotorA.stop(brake);
      leftMotorB.stop(brake);
    } 
    if (abs(Controller1.Axis3.position()) < 5) {
      rightMotorA.stop(brake);
      rightMotorB.stop(brake);
    }
*/
    intake.spin(fwd, 12 * ((int)Controller1.ButtonUp.pressing() - (int)Controller1.ButtonLeft.pressing()), volt);
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
