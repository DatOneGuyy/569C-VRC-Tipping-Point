#include "vex.h"
#include "front_mogo_timeout.h"

using namespace vex;

int front_mogo_movement() {
  bool R2 = false;
  bool R2p = false;

  bool X = false;
  bool Xp = false;
  
  int arm_state = 2;

  int movements = 0;
  
  double error = 0;
  double kp = 0.5;

  front_mogo.resetPosition();

  while (1) {
    R2 = Controller1.ButtonR2.pressing();
    X = Controller1.ButtonX.pressing();

    if (!R2 && R2p && arm_state == 0 && four_bar_limit2.pressing()) {
      reset_timeout2();
      TIMEOUT2 = 2500;
      timeout_thread2 = thread(timeout2);
      timeout_thread2.detach();

      arm_state = 1;

      while (front_mogo.position(degrees) < -800) {
        front_mogo.spin(fwd, 100, pct);

        if (TIMED_OUT2) {
          timeout_thread2.interrupt();
          reset_timeout2();
          break;
        }
      }

      front_mogo.stop(brake);
    } else if (!R2 && R2p && arm_state > 0 && four_bar_limit2.pressing()) {
      reset_timeout2();
      TIMEOUT2 = 2500;
      timeout_thread2 = thread(timeout2);

      arm_state = 0;

      while (front_mogo.position(degrees) > -1600 + movements * 15) {
        front_mogo.spin(fwd, -100, pct);

        if (TIMED_OUT2) {
          timeout_thread2.interrupt();
          reset_timeout2();
          break;
        }
      }

      movements++;
      front_mogo.stop(brake);
    }

    if (!X && Xp && arm_state < 2 && four_bar_limit2.pressing()) {
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

      front_mogo.stop(brake);
      front_mogo.resetPosition();
    }

    if (arm_state == 1 && movements > 0) {
      error = -800 - front_mogo.position(degrees);
      front_mogo.spin(fwd, error * kp, pct);
    }

    R2p = Controller1.ButtonR2.pressing();
    Xp = Controller1.ButtonX.pressing();

    this_thread::sleep_for(25);
  }
  return 0;
}