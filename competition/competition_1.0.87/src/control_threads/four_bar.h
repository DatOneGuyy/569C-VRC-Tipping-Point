#include "vex.h"
#include "four_bar_timeout.h"

using namespace vex;

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

        if (TIMED_OUT0) {
          timeout_thread0.interrupt();
          reset_timeout0();
          break;
        }
      }

      timeout_thread0.interrupt();
      reset_timeout0();

      four_bar.stop(brake);
      four_bar_encoder.resetRotation();
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

      four_bar.stop(coast);
    }

    if (arm_state && !four_bar_limit2.pressing()) {
      error = four_bar_encoder.position(degrees) + 5;
      four_bar.spin(fwd, error * kp, volt);
      
    }
    
    if (!arm_state && !four_bar_limit.pressing()) {
      four_bar.spin(fwd, -4.0, volt);
    }

    L1p = L1;
    
    this_thread::sleep_for(24);
  }
  return 0;
}