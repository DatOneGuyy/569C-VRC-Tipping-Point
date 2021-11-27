#include "vex.h"
#include <cmath>

using namespace vex;

thread timeout_thread0;
thread timeout_thread1;

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
void four_bar_movement() {
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

        if (TIMED_OUT0) {
          timeout_thread0.interrupt();
          reset_timeout0();
          break;
        }
      }

      timeout_thread0.interrupt();
      reset_timeout0();

      four_bar.stop(hold);
    } else if (!L1 && L1p && arm_state) {
      reset_timeout0();
      TIMEOUT0 = 2500;
      timeout_thread0 = thread(timeout0);
      timeout_thread0.detach();

      arm_state = false;

      while (!four_bar_limit.pressing()) {
        four_bar.spin(fwd, -12.0, volt);

        if (TIMED_OUT0) {
          timeout_thread0.interrupt();
          reset_timeout0();
          break;
        }
      }

      timeout_thread0.interrupt();
      reset_timeout0();

      four_bar.stop(hold);
    }

    if (!arm_state && activate && !four_bar_limit.pressing()) {
      four_bar.spin(fwd, 0, volt);
    }

    L1p = L1;
    
    this_thread::sleep_for(10);
  }
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

bool back_arm_state = true;
void back_arm_movement() {
  bool L2 = false;
  bool L2p = false;

  int movements = 0;

  while (1) {
    L2 =  Controller1.ButtonR2.pressing();
    if (!L2 && L2p && !back_arm_state) {
      reset_timeout1();
      TIMEOUT1 = 2500;
      timeout_thread1 = thread(timeout1);
      timeout_thread1.detach();

      back_arm_state = true;

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
    } else if (!L2 && L2p && back_arm_state) {
      reset_timeout1();
      TIMEOUT1 = 2500;
      timeout_thread1 = thread(timeout1);

      back_arm_state = false;

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

    this_thread::sleep_for(10);
  }
}

void pneumatics_movement() {
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
}

void controller_timer() {
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