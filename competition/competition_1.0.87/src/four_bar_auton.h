#include "vex.h"
#include <cmath>
#include "functions/math_library.h"

using namespace vex;

bool up = false;
bool move1 = false;

int four_bar_auton() {
  while (1) {
    if (move1 && up) {
      while (!four_bar_limit.pressing()) {
        four_bar.spin(fwd, -12.0, volt);
      }
      move1 = false;
      four_bar.spin(fwd, -4.0, volt);
    } else if (move1 && !up) {
      while (!four_bar_limit2.pressing()) {
        four_bar.spin(fwd, 12.0, volt);
      }
      move1 = false;
      four_bar.stop(brake);
    }
    if (up) {
      four_bar.spin(fwd, four_bar_encoder.position(degrees) + 5 * 0.5, volt);
    }
  }

  return 0;
}