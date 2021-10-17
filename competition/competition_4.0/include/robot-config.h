using namespace vex;

extern brain Brain;

// VEXcode devices
extern smartdrive Drivetrain;
extern controller Controller1;
extern motor back_arm;
extern motor front_mogo;
extern motor four_bar;
extern motor four_bar2;
extern limit four_bar_limit;
extern limit back_arm_limit;
extern limit four_bar_limit2;
extern encoder four_bar_encoder;
extern digital_out front_pneumatics;
extern limit front_mogo_limit;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );