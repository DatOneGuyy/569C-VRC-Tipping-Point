/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\IL NAMKUNG                                       */
/*    Created:      Sat Oct 16 2021                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

competition Competition;

vex::thread timeout_thread0;
vex::thread timeout_thread1;
vex::thread timeout_thread2;

int TIMEOUT0 = 0;
bool TIMED_OUT0 = false;
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
int four_bar_movement() {
  bool L1 = false;
  bool L1p = false;

  bool arm_state = false;

  double error = 0;

  double kp = 0.5;

  while (1) {
    L1 = Controller1.ButtonL1.pressing();

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
      four_bar_encoder.resetRotation();
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

      four_bar.stop(coast);
      four_bar2.stop(coast);
    }

    if (arm_state && !four_bar_limit2.pressing()) {
      error = four_bar_encoder.position(degrees) + 5;
      four_bar.spin(fwd, error * kp, volt);
      four_bar2.spin(fwd, error * kp, volt);
      
    }

    L1p = L1;
    
    this_thread::sleep_for(24);
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

      while (back_arm.position(degrees) < -400) {
        back_arm.spin(fwd, 12.0, volt);

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

      while (back_arm.position(degrees) > (-900 + movements * 5)) {

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

    if (arm_state && !back_arm_limit.pressing() && movements > 0) {
      error = -800 - back_arm.position(degrees);
      back_arm.spin(fwd, error * kp, pct);
    }

    L2p = Controller1.ButtonL2.pressing();

    this_thread::sleep_for(25);
  }
  return 0;
}

int TIMEOUT2 = 0;
bool TIMED_OUT2 = false;
void reset_timeout2() {
  TIMED_OUT2 = false;
  TIMEOUT2 = 0;
}
int timeout2() {
  TIMED_OUT2 = false;
  this_thread::sleep_for(TIMEOUT2);
  TIMED_OUT2 = true;
  return 0;
}
int front_mogo_movement() {
  bool R2 = false;
  bool R2p = false;

  bool X = false;
  bool Xp = false;
  
  int arm_state = 2;

  front_mogo.resetPosition();

  while (1) {
    R2 = Controller1.ButtonR2.pressing();
    X = Controller1.ButtonX.pressing();

    if (!R2 && R2p && arm_state == 0) {
      reset_timeout2();
      TIMEOUT2 = 2500;
      timeout_thread2 = thread(timeout2);
      timeout_thread2.detach();

      arm_state = 1;

      while (front_mogo.position(degrees) < -400) {
        front_mogo.spin(fwd, 12.0, volt);

        if (TIMED_OUT2) {
          timeout_thread2.interrupt();
          reset_timeout2();
          break;
        }
      }

      front_mogo.stop(hold);
    } else if (!R2 && R2p && arm_state > 0) {
      reset_timeout2();
      TIMEOUT2 = 2500;
      timeout_thread2 = thread(timeout2);

      arm_state = 0;

      while (front_mogo.position(degrees) > -830) {
        front_mogo.spin(fwd, -12.0, volt);

        if (TIMED_OUT2) {
          timeout_thread2.interrupt();
          reset_timeout2();
          break;
        }
      }

      front_mogo.stop(hold);
    }

    if (!X && Xp && arm_state < 2) {
      reset_timeout2();
      TIMEOUT2 = 2500;
      timeout_thread2 = thread(timeout2);

      arm_state = 2;

      while (!front_mogo_limit.pressing()) {
        front_mogo.spin(fwd, 12.0, volt);

        if (TIMED_OUT2) {
          timeout_thread2.interrupt();
          reset_timeout2();
          break;
        }
      }

      front_mogo.stop(hold);
      front_mogo.resetPosition();
    }

    R2p = Controller1.ButtonR2.pressing();
    Xp = Controller1.ButtonX.pressing();

    this_thread::sleep_for(25);
  }
  return 0;
}

int pneumatics_movement() {
  bool R1 = false;
  bool R1p = false;

  front_pneumatics.set(false);

  while (1) {
    R1 = Controller1.ButtonR1.pressing();

    if (!R1 && R1p && !front_pneumatics.value()) {
      front_pneumatics.set(true);
    } else if (!R1 && R1p && front_pneumatics.value()) {
      front_pneumatics.set(false);
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

void drive_forward(int dist, int speed) {
  Drivetrain.driveFor(dist, inches, speed * 2, rpm, false);
}
void drive_backward(int dist, int speed) {
  Drivetrain.driveFor(-dist, inches, speed * 2, rpm, false);
}
void turn(int angle, int speed) {
  Drivetrain.turnToRotation(angle, degrees, speed * 2, rpm, false);
}
void move_back_arm_up(int position) {
  while (back_arm.position(degrees) < position) {
    back_arm.spin(fwd, 12.0, volt);
  }
  back_arm.stop(hold);
}
void move_back_arm_down(int position) {
  while (back_arm.position(degrees) > position) {
    back_arm.spin(fwd, -12.0, volt);
    Brain.Screen.clearLine();
    Brain.Screen.print(back_arm.position(degrees));
  }
  back_arm.stop(hold);
}
void move_four_bar_up() {
  while (!four_bar_limit2.pressing()) {
    four_bar.spin(fwd, 12.0, volt);
    four_bar2.spin(fwd, 12.0, volt);
  }
  four_bar.stop(hold);
  four_bar2.stop(hold);
}
void move_four_bar_down() {
  while (!four_bar_limit.pressing()) {
    four_bar.spin(fwd, -12.0, volt);
    four_bar2.spin(fwd, -12.0, volt);
  }
  four_bar.stop(hold);
  four_bar.stop(hold);
}
void move_front_mogo_up(int position) {
  while (front_mogo.position(degrees) < position) {
    front_mogo.spin(fwd, 12.0, volt);
  }
  front_mogo.stop(hold);
} 
void move_front_mogo_down(int position) {
  while (front_mogo.position(degrees) > position) {
    front_mogo.spin(fwd, -12.0, volt);
  }
  front_mogo.stop(hold);
}

void pre_auton() {

}

void autonomous() {
  drive_backward(20, 70);
  move_back_arm_down(-900);
  wait(300, msec);
  move_back_arm_up(-400);
}

void user_control() {
  vex::thread four_bar_thread(four_bar_movement);
  vex::thread back_arm_thread(back_arm_movement);
  vex::thread front_mogo_thread(front_mogo_movement);
  vex::thread pneumatics_thread(pneumatics_movement);
  vex::thread controller_timer_thread(controller_timer);

  thread(four_bar_thread).detach();
  thread(back_arm_thread).detach();
  thread(front_mogo_thread).detach();
  thread(pneumatics_thread).detach();
  thread(controller_timer_thread).detach();

  while (1) {
    if (Controller1.ButtonRight.pressing()) {
      autonomous();
    }
    wait(10, msec);
  }
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  Competition.autonomous(autonomous);
  Competition.drivercontrol(user_control);

  pre_auton();

  while (1) {

    wait(100, msec);

  }
}
