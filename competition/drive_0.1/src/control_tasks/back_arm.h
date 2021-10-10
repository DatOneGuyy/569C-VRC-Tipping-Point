#include "vex.h"
#include "back_arm_timeout.h"

using namespace vex;

int back_arm_movement() {
  bool L2;
  bool L2p;
  
  bool arm_state = true;

  while (1) {
    L2 = Controller1.ButtonL2.pressing();

    if (back_arm_limit.pressing()) {
      back_arm.resetPosition();
    }

    if (!L2 && L2p && !arm_state) {
      reset_timeout1();
      TIMEOUT1 = 2500;
      timeout_thread1 = thread(timeout1);

      arm_state = true;

      while (back_arm.position(degrees) > -1800) {
        back_arm.spin(fwd, 12.0, volt);

        if (TIMED_OUT1) {
          timeout_thread1.interrupt();
          reset_timeout1();
          break;
        }
      }

      back_arm.stop(brake);
    } else if (L2 && !L2p && arm_state) {
      reset_timeout1();
      TIMEOUT1 = 2500;
      timeout_thread1 = thread(timeout1);

      arm_state = false;

      while (back_arm.position(degrees) < -400) {
        back_arm.spin(fwd, -12.0, volt);

        if (TIMED_OUT1) {
          timeout_thread1.interrupt();
          reset_timeout1();
          break;
        }
      }

      back_arm.stop(brake);
    }

    L2p = Controller1.ButtonL2.pressing();

    this_thread::sleep_for(25);
  }
  return 0;
}