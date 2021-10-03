/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\IL NAMKUNG                                       */
/*    Created:      Sat Oct 02 2021                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

int x = 0;
int px = 0;
double speed = 100;

void drive(double voltage) {
  front_left.spin(fwd, voltage, volt);
  back_left.spin(fwd, voltage, volt);
  front_right.spin(fwd, voltage, volt);
  back_right.spin(fwd, voltage, volt);
}

void stop() {
  front_left.stop(brake);
  back_left.stop(brake);
  front_right.stop(brake);
  back_right.stop(brake);
}

double position() {
  return (double)(front_left.position(degrees) + front_right.position(degrees) + back_left.position(degrees) + back_right.position(degrees)) / 4;
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  while (position() < 1200) {
    drive(speed * 3 / 25);
    x = position();
    Brain.Screen.print(x - px);
    Brain.Screen.newLine();
    px = position();
    wait(100, msec);
  }
  stop();
}
