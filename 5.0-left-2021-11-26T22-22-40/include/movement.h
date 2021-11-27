#ifndef MOVEMENT_H
#define MOVEMENT_H

void drive(double target_left, double target_right, double pid_left[], double pid_right[], double gyro_target, int actions[], int action_count);

void auton_back_arm_up(void);
void auton_back_arm_down(void);
void auton_four_bar_up(void);
void auton_four_bar_down(void);

#endif