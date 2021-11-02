#include "main.h"
#include "okapi/api.hpp"

using namespace okapi;

Controller controller;

std::shared_ptr<OdomChassisController> drive =
	ChassisControllerBuilder()
		.withMotors(
			{11, 1},
			{-6, -16}
		)
		.withGains(
			{0.001, 0, 0.0001}, // Distance controller gains
			{0.001, 0, 0.0001}, // Turn controller gains
			{0.001, 0, 0.0001}  // Angle controller gains (helps drive straight)
		)
		.withDimensions({AbstractMotor::gearset::green, 2}, {{3.25_in, 15.125_in}, imev5GreenTPR})
		.withOdometry()
		.buildOdometry();

Motor four_bar(2);
Motor four_bar2(-12);
ADIButton four_bar_limit('F');
ADIButton four_bar_limit2('E');
ControllerButton buttonL1(ControllerDigital::L1);

bool four_bar_running, four_bar_state, L1, L1p = false;

void run_four_bar() {
	four_bar.setBrakeMode(AbstractMotor::brakeMode::hold);
	four_bar2.setBrakeMode(AbstractMotor::brakeMode::hold);

	four_bar.setGearing(AbstractMotor::gearset::red);
	four_bar2.setGearing(AbstractMotor::gearset::red);

	L1 = buttonL1.isPressed();

	if (L1 && !L1p) {
		four_bar_running = true; //On button press, activate movement
	}

	if (four_bar_running) { //Run while movement is active
		if (four_bar_limit2.isPressed() && !four_bar_state) { //If top limit is pressed and arm is still considered down
			four_bar.moveVelocity(0);
			four_bar2.moveVelocity(0);

			four_bar_running = false; //Indicate that the arm is no longer moving
			four_bar_state = true; //Indicate that arm state is now up
		} else if (four_bar_limit.isPressed() && four_bar_state) { //If bottom limit is pressed and arm is still considered up
			four_bar.moveVelocity(0);
			four_bar2.moveVelocity(0);

			four_bar_running = false; //Indicate that the arm is no longer moving
			four_bar_state = false; //Indicate that arm state is now down
		} else { //If no limit is pressed but arm is running
			if (!four_bar_state) { //If arm is marked as down, go up
				four_bar.moveVelocity(100);
				four_bar2.moveVelocity(100);
			} else { //If arm is marked as up, go down
				four_bar.moveVelocity(-100);
				four_bar2.moveVelocity(-100);
			}
		}
	}

	L1p = L1;

}

Motor back_arm(17);
ADIButton back_arm_limit('C');
ControllerButton buttonL2(ControllerDigital::L2);

bool back_arm_running, L2, L2p = false;
bool back_arm_state = true;

void run_back_arm() {
	back_arm.setBrakeMode(AbstractMotor::brakeMode::hold);

	back_arm.setGearing(AbstractMotor::gearset::red);

	L2 = buttonL2.isPressed();

	if (L2 && L2p) {
		back_arm_running = true; //On button press, activate movement
	}

	if (back_arm_running) { //Run while movement is active
		if ((back_arm_limit.isPressed() || back_arm.getPosition() > -1500) && !back_arm_state) { //If limit is pressed and arm is still considered down, or if the arm goes above the height threshold
			back_arm.moveVelocity(0);

			back_arm_running = false; //Indicate that the arm is no longer moving
			back_arm_state = true; //Indicate that arm state is now up
		} else if (back_arm_state && back_arm.getPosition() < -3200) { //If arm is below horizontal threshold and arm is still considered up
			back_arm.moveVelocity(0);

			back_arm_running = false; //Indicate that the arm is no longer moving
			back_arm_state = false; //Indicate that arm state is now down
		} else { //If arm is running
			if (!back_arm_state) { //If arm is marked as down, go up
				back_arm.moveVelocity(100);
			} else { //If arm is marked as up, go down
				back_arm.moveVelocity(-100);
			}
		}
	}

	L2p = L2;
}

pros::ADIPort pneumatics(7, pros::E_ADI_DIGITAL_OUT);
ControllerButton buttonR1(ControllerDigital::R1);

bool pneumatics_closed, R1, R1p = false;

void run_pneumatics() {
	R1 = buttonR1.isPressed();

	if (R1 && !R1p) { //If button pressed, toggle pnematics state
		pneumatics_closed = !pneumatics_closed;
		pneumatics.set_value(pneumatics_closed ? 1 : 0);
	}

	R1p = R1;
}

Motor intake(7);
ControllerButton buttonR2(ControllerDigital::R2);

bool intake_active, R2, R2p = false;

void run_intake() {
	R2 = buttonR2.isPressed();

	if (R2 && !R2p) { //If button pressed, toggle intake_active state

		intake_active = !intake_active;
		if (!four_bar_state && four_bar_limit.isPressed()) { //If four bar is down and limit is pressed, stop the intake
			intake.moveVelocity(0);
		} else {
			intake.moveVelocity(intake_active ? 200 : 0);
		}
	}

	R2p = R2;
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {

}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	drive->setState({0_in, 0_in, 0_deg});
	drive->driveToPoint({20_in, 0_in});
}


/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	pros::screen::erase();

	while (true) {
		drive->getModel()->left(controller.getAnalog(ControllerAnalog::leftY));
		drive->getModel()->right(controller.getAnalog(ControllerAnalog::rightY));
		drive->getModel()->setBrakeMode(AbstractMotor::brakeMode::hold);

		run_four_bar();
		run_back_arm();
		run_pneumatics();
		run_intake();

		if (buttonL1.isPressed()) {
			//autonomous();
		}

		pros::delay(10);
	}
}
