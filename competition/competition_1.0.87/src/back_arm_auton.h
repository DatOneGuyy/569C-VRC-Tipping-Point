#include "vex.h"
#include <cmath>
#include "functions/math_library.h"

using namespace vex;

int auton_back_arm_position = 0;
bool move0 = false;
int e = 0;

int back_arm_auton() {
  while (1) {
    if (move0) {
      e++;
      while (std::abs(back_arm.position(degrees) - auton_back_arm_position) > 20) {
        back_arm.spin(fwd, normalize(back_arm.position(degrees) + auton_back_arm_position) * 12, volt);
        if (std::abs(back_arm.position(degrees) - auton_back_arm_position) > 20) {
          break;
        }
      }
      back_arm.stop(brake);
      move0 = false;
      Brain.Screen.print(move0);
    }
  }

  return 0;
}