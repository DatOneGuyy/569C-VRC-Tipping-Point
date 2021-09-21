#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
inertial inertial5 = inertial(PORT5);
motor front_left = motor(PORT11, ratio18_1, false);
motor front_right = motor(PORT13, ratio18_1, true);
motor back_left = motor(PORT12, ratio18_1, false);
motor back_right = motor(PORT14, ratio18_1, true);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}
