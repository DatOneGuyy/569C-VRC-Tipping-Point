#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor front_left = motor(PORT11, ratio18_1, false);
motor back_left = motor(PORT12, ratio18_1, false);
motor front_right = motor(PORT9, ratio18_1, true);
motor back_right = motor(PORT10, ratio18_1, true);
inertial inertial5 = inertial(PORT5);
motor back_arm = motor(PORT13, ratio18_1, true);
motor front_mogo = motor(PORT3, ratio18_1, true);
motor four_bar = motor(PORT2, ratio18_1, false);
limit four_bar_limit = limit(Brain.ThreeWirePort.A);
limit back_arm_limit = limit(Brain.ThreeWirePort.B);
motor intake = motor(PORT7, ratio18_1, false);
limit four_bar_limit2 = limit(Brain.ThreeWirePort.C);
encoder four_bar_encoder = encoder(Brain.ThreeWirePort.E);
digital_out front_pneumatics = digital_out(Brain.ThreeWirePort.G);

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