using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor front_left;
extern motor back_left;
extern motor front_right;
extern motor back_right;
extern inertial inertial5;
extern motor back_arm;
extern motor front_mogo;
extern motor four_bar;
extern limit four_bar_limit;
extern limit back_arm_limit;
extern motor intake;
extern limit four_bar_limit2;
extern encoder four_bar_encoder;
extern digital_out front_pneumatics;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );