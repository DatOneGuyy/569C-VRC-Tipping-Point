#include "main.h"
#include "okapi/api.hpp"

using namespace okapi;

Motor back_arm(-14);
MotorGroup four_bar({-10, 2});
Motor intake(19);
ADIButton back_arm_limit('F');
ADIButton four_bar_limit('B');
ADIButton four_bar_limit2('G');

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void four_bar_movement(void*) {
	bool arm_state = true;

	ControllerButton L1(ControllerDigital::L1);
	ControllerButton Up(ControllerDigital::up);
	ControllerButton Left(ControllerDigital::left);
	four_bar.setBrakeMode(AbstractMotor::brakeMode::hold);

	bool x = false;
	bool y = false;

	int timeout_counter = 5;

	while (true) {
		x = L1.isPressed();

    if (x && !y && !arm_state) {
      while (!four_bar_limit2.isPressed()) {
        four_bar.moveVelocity(200);
				timeout_counter++;
				if (timeout_counter > 300) {
					break;
				}
				pros::delay(10);
      }
			timeout_counter = 0;
			arm_state = true;
			four_bar.moveVelocity(0);
    } else if (x && !y && arm_state) {
      while (!four_bar_limit.isPressed()) {
				timeout_counter++;
        four_bar.moveVelocity(-200);
				if (timeout_counter > 300) {
					break;
				}
				pros::delay(10);
      }
			timeout_counter = 0;
	    arm_state = false;
      four_bar.moveVelocity(0);
    }
		y = x;
		if (Up.isPressed()) {
			four_bar.moveAbsolute(1600, 200);
			pros::lcd::print(2, "%d", four_bar.getPosition());
		}
		four_bar.moveVelocity((Up.isPressed() * !four_bar_limit2.isPressed() - Left.isPressed() * !four_bar_limit.isPressed()) * 200);
		pros::lcd::print(4, "%d", (Up.isPressed() - Left.isPressed()) * 200);
		pros::delay(10);
	}
}
void back_arm_movement(void*) {
	bool back_arm_state = true;
	ControllerButton L2(ControllerDigital::L2);
	ControllerButton X(ControllerDigital::X);
	ControllerButton A(ControllerDigital::A);

	back_arm.setBrakeMode(AbstractMotor::brakeMode::hold);

	bool x = false;
	bool y = false;

	int timeout_counter = 0;

	while (true) {
		back_arm.moveVelocity(200 * (X.isPressed() * !back_arm_limit.isPressed() - A.isPressed()));
		x = L2.isPressed();
    if (x && !y && !back_arm_state) {
      back_arm_state = true;
      while (!back_arm_limit.isPressed()) {
        back_arm.moveVelocity(200);
				timeout_counter++;
        if (back_arm.getPosition() > -1000 || timeout_counter > 300) {
          break;
        }
				pros::delay(10);
      }
			timeout_counter = 0;
      back_arm.moveVelocity(0);
    } else if (x && !y && back_arm_state) {
      back_arm_state = false;
      while (back_arm.getPosition() > -3200) {
        back_arm.moveVelocity(-100);
				timeout_counter++;
				if (timeout_counter > 300) {
					break;
				}
				pros::delay(10);
      }
			timeout_counter = 0;
      back_arm.moveVelocity(0);
    }
		x = y;
    pros::delay(10);
  }
}
void pneumatics_movement(void*) {
	bool pneumatics_state = false;
	ControllerButton R1(ControllerDigital::R1);
	pros::ADIPort front_pneumatics ('H', pros::E_ADI_DIGITAL_OUT);

	bool x = false;
	bool y = false;
	while (true) {
		x = R1.isPressed();
		if (x && !y) {
			pneumatics_state = !pneumatics_state;
			pros::lcd::print(0, "%d", pneumatics_state);
			front_pneumatics.set_value(pneumatics_state);

		}
		y = x;
		pros::delay(10);
	}
}
void intake_movement(void*) {
	ControllerButton R2(ControllerDigital::R2);
	bool intake_state = false;

	bool x = false;
	bool y = false;

	while (true) {
		x = R2.isPressed();
		if (x && !y) {
			intake_state = !intake_state;
			intake.moveVelocity(intake_state * 200);
		}
		y = x;
		pros::delay(10);
	}
}
void driver_timer(void*) {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	int time = 1050;
	while (time > 0) {
		master.clear_line(0);
		if ((time % 600 / 10) > 10) {
			master.print(0, 0, "Time: %d:%d.%d", time >= 600, time % 600 / 10, time % 10);
		} else {
			master.print(0, 0, "Time: %d:0%d.%d", time >= 600, time % 600 / 10, time % 10);
		}
		pros::delay(100);
		time--;
		if (time == 450) {
			master.rumble(".-.-.");
		}
		if (time == 300) {
			master.rumble("-----");
		}
	}
	master.print(0, 0, "Time: 0:00.0");
}
void auton_back_arm(void*) {
	back_arm.setBrakeMode(AbstractMotor::brakeMode::hold);
	while (!back_arm_limit.isPressed()) {
		back_arm.moveVelocity(200);
		if (back_arm.getPosition() > -1000) {
			break;
		}
	}
	back_arm.moveVelocity(0);
}

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

double dtor(double angle) {
	return angle * PI / 180;
}
bool recalibrate(double current_angle, double threshold) {

}

class CustomOdometry {
	public:
		double x = 0;
		double y = 0;
		double angle = 0;

		void tick(double left_position, double right_position) {
			calculate_encoder_change(left_position, right_position);
			rtoi_deltas();
			calculate_angles();
			calculate_position();
		}


	private:
		const double track_width = 16.25;
		const double wheel_radius = 4.125;

		double left_encoder = 0;
		double right_encoder = 0;
		double left_encoder_previous = 0;
		double right_encoder_previous = 0;

		double delta_left = 0;
		double delta_right = 0;

		double delta_angle = (delta_right - delta_left) / track_width;
		double turn_radius = (delta_left / delta_angle) + (track_width / 2);

		double delta_x = turn_radius * cos(delta_angle) - turn_radius;
		double delta_y = turn_radius * sin(delta_angle);

		void calculate_encoder_change(double left_position, double right_position) {
			left_encoder = left_position;
			right_encoder = right_position;
			delta_left = left_encoder_previous - left_encoder;
			delta_right = right_encoder_previous - right_encoder;
			left_encoder_previous = left_encoder;
			right_encoder_previous = right_encoder;
		}
		void rtoi_deltas() {
			delta_left = dtor(delta_left) * wheel_radius;
			delta_right = dtor(delta_right) * wheel_radius;
		}
		void calculate_angles() {
			delta_angle = (delta_right - delta_left) / track_width;
			if (delta_angle != 0) {
				turn_radius = (delta_left / delta_angle) + (track_width / 2);
			} else {
				turn_radius = 0;
			}
			angle += delta_angle;
		}
		void calculate_position() {
			if (turn_radius != 0) {
				delta_x = turn_radius * cos(delta_angle) - turn_radius;
				delta_y = turn_radius * sin(delta_angle);
			} else {
				delta_x = cos(angle);
				delta_y = sin(angle);
			}
			x += delta_x;
			y += delta_y;
		}
};

void odometry() {
	//counterclockwise -> +angle
	//units: inches/radians
	//dtor: degree->radian conversion

	/*
	 -+ |	++
	----+----
	 -- |	+-
	*/
	MotorGroup left_drive({-1, -6});
	MotorGroup right_drive({11, 16});

	CustomOdometry odom;

	while (true) {

		pros::delay(5);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
}

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
	back_arm.setGearing(AbstractMotor::gearset::red);
	back_arm.setBrakeMode(AbstractMotor::brakeMode::hold);
	four_bar.setGearing(AbstractMotor::gearset::red);
	four_bar.setBrakeMode(AbstractMotor::brakeMode::hold);
	intake.setGearing(AbstractMotor::gearset::green);
	intake.setBrakeMode(AbstractMotor::brakeMode::coast);
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
	std::shared_ptr<OdomChassisController> chassis =
		ChassisControllerBuilder()
			.withMotors(
				{-20, -15}, //-15
				{9, 1} //1
			)
			.withDimensions(AbstractMotor::gearset::green, {{3.25_in, 15.2_in}, imev5GreenTPR})
			.withGains(
				{0.0015, 0.00005, 0.0001},
				{0.002, 0, 0.0001},
				{0.001, 0, 0.00}
			)
			.withOdometry()
			.buildOdometry();

	std::shared_ptr<AsyncMotionProfileController> profileController =
		AsyncMotionProfileControllerBuilder()
			.withLimits({3.0, 2.0, 10.0})
			.withOutput(chassis)
			.buildMotionProfileController();

	pros::ADIPort front_pneumatics ('H', pros::E_ADI_DIGITAL_OUT);
	chassis->setState({0_ft, 0_ft, -45_deg});
	front_pneumatics.set_value(false);
	chassis->driveToPoint({4.5_ft, -4.3_ft});
	pros::delay(200);
	front_pneumatics.set_value(true);
	four_bar.moveAbsolute(300, 100);
	back_arm.moveAbsolute(-3250, -100);
	chassis->driveToPoint({2.2_ft, -2.3_ft}, true);
	front_pneumatics.set_value(false);
	chassis->driveToPoint({2.6_ft, -0.6_ft}, true);
	pros::Task back_arm_auton(auton_back_arm);
	four_bar.moveAbsolute(0, 100);
	pros::delay(1000);
	chassis->driveToPoint({5_ft, -1.9_ft}, false);
	front_pneumatics.set_value(true);
	intake.moveVelocity(200);
	four_bar.moveAbsolute(800, 100);
	chassis->moveDistance(-4_ft);
	pros::delay(500);
	back_arm.moveAbsolute(-3200, 200);
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
	//autonomous();
	Controller controller;

	pros::Task four_bar_task(four_bar_movement);
	pros::Task back_arm_task(back_arm_movement);
	pros::Task pneumatics_task(pneumatics_movement);
	pros::Task intake_task(intake_movement);

	if ((pros::competition::get_status() & (COMPETITION_CONNECTED == true)) && !pros::competition::is_autonomous()) {
		pros::Task driver_timer_task(driver_timer);
	}

	std::shared_ptr<ChassisController> chassis =
		ChassisControllerBuilder()
			.withMotors(
				{-20, -15}, //-15
				{9, 1} //1
			)
			.withDimensions(AbstractMotor::gearset::green, {{3.25_in, 15.2_in}, imev5GreenTPR})
			.build();

	while (true) {
		chassis->getModel()->tank(
			1.3 * controller.getAnalog(ControllerAnalog::leftY),
			1.3 * controller.getAnalog(ControllerAnalog::rightY)
		);
		chassis->getModel()->setBrakeMode(AbstractMotor::brakeMode::hold);
		pros::delay(10);
	}
}
