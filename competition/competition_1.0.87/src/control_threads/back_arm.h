#include "vex.h"
#include "back_arm_timeout.h"

using namespace vex;

int back_arm_movement() {
  bool L2 = false;
  bool L2p = false;
  
  bool arm_state = true;

  int movements = 0;
  
  double error = 0;
  double kp = 0.5;

  back_arm.resetPosition();

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

      while (back_arm.position(degrees) > -1600 + movements * 15) {
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