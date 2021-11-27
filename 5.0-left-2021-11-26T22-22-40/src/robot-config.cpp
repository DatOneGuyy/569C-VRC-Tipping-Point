#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;
competition Competition;

// VEXcode device constructors
/*vex-vision-config:begin*/
signature Vision17__BLUE_MOGO = signature (1, -2493, 1, -1246, -1, 7681, 3840, 0.9, 0);
signature Vision17__YELLOW_MOGO = signature (2, 349, 2311, 1330, -4147, -2699, -3423, 1.1, 0);
signature Vision17__SIG_3 = signature (3, 0, 0, 0, 0, 0, 0, 3, 0);
signature Vision17__SIG_4 = signature (4, 0, 0, 0, 0, 0, 0, 3, 0);
signature Vision17__SIG_5 = signature (5, 0, 0, 0, 0, 0, 0, 3, 0);
signature Vision17__SIG_6 = signature (6, 0, 0, 0, 0, 0, 0, 3, 0);
signature Vision17__SIG_7 = signature (7, 0, 0, 0, 0, 0, 0, 3, 0);
vision Vision17 = vision (PORT17, 146, Vision17__BLUE_MOGO, Vision17__YELLOW_MOGO, Vision17__SIG_3, Vision17__SIG_4, Vision17__SIG_5, Vision17__SIG_6, Vision17__SIG_7);
/*vex-vision-config:end*/

// VEXcode device constructors
motor leftMotorA = motor(PORT20, ratio18_1, true);
motor leftMotorB = motor(PORT15, ratio18_1, true);
motor_group leftdrive = motor_group(leftMotorA, leftMotorB);
motor rightMotorA = motor(PORT9, ratio18_1, false);
motor rightMotorB = motor(PORT1, ratio18_1, false);
motor_group rightdrive = motor_group(rightMotorA, rightMotorB);
controller Controller1 = controller(primary);
motor back_arm = motor(PORT14, ratio36_1, true);
motor intake = motor(PORT19, ratio18_1, false);
motor four_barMotorA = motor(PORT10, ratio36_1, true);
motor four_barMotorB = motor(PORT2, ratio36_1, false);
motor_group four_bar = motor_group(four_barMotorA, four_barMotorB);
inertial inertial0 = inertial(PORT18);
limit four_bar_limit = limit(Brain.ThreeWirePort.B);
limit back_arm_limit = limit(Brain.ThreeWirePort.F);
limit four_bar_limit2 = limit(Brain.ThreeWirePort.G);
digital_out front_pneumatics = digital_out(Brain.ThreeWirePort.H);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;
void vexcodeInit( void ) {
  Brain.Screen.print("Device initialization...");
  Brain.Screen.setCursor(2, 1);
  // calibrate the drivetrain Inertial
  wait(200, msec);
  inertial0.calibrate();
  Brain.Screen.print("Calibrating Inertial");
  // wait for the Inertial calibration process to finish
  while (inertial0.isCalibrating()) {
    wait(25, msec);
  }
  // reset the screen now that the calibration is complete
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1,1);
}
// VEXcode generated functions