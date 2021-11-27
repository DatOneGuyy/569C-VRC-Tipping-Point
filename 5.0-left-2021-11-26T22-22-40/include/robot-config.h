#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

using namespace vex;

extern brain Brain;
extern competition Competition;

using signature = vision::signature;

// VEXcode devices
extern signature Vision17__BLUE_MOGO;
extern signature Vision17__YELLOW_MOGO;
extern signature Vision17__SIG_3;
extern signature Vision17__SIG_4;
extern signature Vision17__SIG_5;
extern signature Vision17__SIG_6;
extern signature Vision17__SIG_7;
extern vision Vision17;

extern motor_group leftdrive;
extern motor_group rightdrive;
extern motor back_arm;
extern motor_group four_bar;
extern motor intake;
extern controller Controller1;
extern inertial inertial0;
extern limit back_arm_limit;
extern limit four_bar_limit;
extern limit four_bar_limit2;
extern digital_out front_pneumatics;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );

#endif 