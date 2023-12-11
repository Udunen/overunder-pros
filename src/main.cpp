#include "main.h"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
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
void competition_initialize() {}

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
void autonomous() {}

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
	pros::Controller	master(pros::E_CONTROLLER_MASTER);

	/*
	left front - 19
	left mid - 17
	left back - 18
	right front - 8
	right mid - 9
	right back - 7
	*/
	//double check the reverse booleans
	pros::Motor			left_front(19, MOTOR_GEAR_BLUE, false);
	pros::Motor			left_mid(17, MOTOR_GEAR_BLUE, false);
	pros::Motor			left_back(18, MOTOR_GEAR_BLUE, false);
	pros::Motor			right_front(8, MOTOR_GEAR_BLUE, false);
	pros::Motor			right_mid(9, MOTOR_GEAR_BLUE, false);
	pros::Motor			right_back(7,  MOTOR_GEAR_BLUE, false);
	pros::Motor_Group	left_drive({19, 17, 18});
	pros::Motor_Group	right_drive({8, 9, 7});
	pros::Motor			flyWheel(1, MOTOR_GEAR_BLUE, false);


	/*
	intake - 3
	arm - 2
	shooter - 1
	*/
	//make sure u set the gearset and reverse booleans
	pros::Motor			intake(3, false);
	pros::Motor			arm(2, false);
	pros::Motor			shooter(1, false);

	//change from 0 when figure out real port lol
	pros::ADIDigitalOut	wings('A', false);
	
	double drive, turn;
	int wingToggle;

	while (true) {
		drive = master.get_analog(ANALOG_LEFT_Y);
		turn = master.get_analog(ANALOG_RIGHT_X) / 1.5;
	

		// uncomment when ready to drive
		// left_drive.move_velocity((drive + turn)*6);
		// right_drive.move_velocity((drive - turn)*6);

		if(master.get_digital(DIGITAL_B) == 1) {
			// intake.set_velocity();
		}

		if(master.get_digital(DIGITAL_A) == 1) {
			if(wingToggle == 1) {
				wingToggle == 0;
			} else {
				wingToggle == 1;
			}
		}
		wings.set_value(wingToggle);


	}
}
