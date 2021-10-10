#include "vex.h"
#include "four_bar_timeout.h"

using namespace vex;

int front_arm_movement() {
  bool L1 = false;
  bool L1p = false;

  bool arm_state = false; //false- down, true- up

  double error = 0;

  double kp = 0.03;

  while (1) {
    L1 = Controller1.ButtonL1.pressing();

    if (!L1 && L1p && !arm_state) {
      reset_timeout0();
      TIMEOUT0 = 2500;
      timeout_thread0 = thread(timeout0);

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

      four_bar.stop(brake);
      //four_bar_encoder.resetRotation();
    } else if (!L1 && L1p && arm_state) {
      reset_timeout0();
      TIMEOUT0 = 2500;
      timeout_thread0 = thread(timeout0);

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

      four_bar.stop(coast);
    }

    if (arm_state) {
      error = four_bar_encoder.position(degrees);
      //four_bar.spin(fwd, error * kp, volt);
      
    }
    //Brain.Screen.clearScreen();
    //Brain.Screen.print(four_bar_encoder.position(degrees));
    if (!arm_state && !four_bar_limit.pressing()) {
      four_bar.spin(fwd, -3.0, volt);
    }
    L1p = L1;
    
    this_thread::sleep_for(24);
  }
  return 0;
}