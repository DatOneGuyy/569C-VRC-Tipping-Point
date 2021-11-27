#include "vex.h"

using namespace vex;

bool intake_active = false;
bool R2, R2p = false;

void user_control() {
  thread four_bar_thread(four_bar_movement);
  thread back_arm_thread(back_arm_movement);
  thread pneumatics_thread(pneumatics_movement);
  if (Competition.isFieldControl()) {
    thread controller_timer_thread(controller_timer);
    controller_timer_thread.detach();
  }

  four_bar_thread.detach();
  back_arm_thread.detach();
  pneumatics_thread.detach();

  front_pneumatics.set(false);
  
  while (1) {
    R2 = Controller1.ButtonX.pressing();

    if (R2 && !R2p) {
      intake_active = !intake_active;
    }

    intake.spin(fwd, 12.0 * intake_active, volt);

    R2p = R2;
    wait(10, msec);
  }
}