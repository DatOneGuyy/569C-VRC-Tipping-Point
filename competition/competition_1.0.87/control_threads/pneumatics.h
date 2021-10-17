#include "vex.h"

using namespace vex;

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