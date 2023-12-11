#include "main.h"
#include "lemlib/api.hpp"

pros::Controller	master(pros::E_CONTROLLER_MASTER);
//double check the reverse booleans
pros::Motor			left_front(19, MOTOR_GEAR_BLUE, true);
pros::Motor			left_mid(17, MOTOR_GEAR_BLUE, true);
pros::Motor			left_back(18, MOTOR_GEAR_BLUE, true);
pros::Motor			right_front(8, MOTOR_GEAR_BLUE, false);
pros::Motor			right_mid(9, MOTOR_GEAR_BLUE, false);
pros::Motor			right_back(7,  MOTOR_GEAR_BLUE, false);
pros::Motor_Group	left_drive({left_front, left_mid, left_back});
pros::Motor_Group	right_drive({right_front, right_mid, right_back});
lemlib::Drivetrain_t drivetrain {
    &left_drive, // left drivetrain motors
    &right_drive, // right drivetrain motors
    13, // track width
    3.25, // wheel diameter
    600 // wheel rpm
};
// need to setup tracking wheel for drivetrain

pros::Motor			flyWheel(1, MOTOR_GEAR_BLUE, false);
pros::Motor			intake(3, MOTOR_GEAR_GREEN, false);
pros::Motor			arm(2, MOTOR_GEAR_GREEN, true);

pros::ADIDigitalOut	wings('A', false);

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
	
	double drive, turn;
	bool wingToggle;

	while (true) {
		drive = master.get_analog(ANALOG_LEFT_Y);
		turn = master.get_analog(ANALOG_RIGHT_X) / 1.5;
		arm.set_brake_mode(MOTOR_BRAKE_HOLD);

		// uncomment when ready to drive
		// also we change to LemLib driving bc its better pepega, we just have this rn for testing purposes :)
		// since its a blue motor, we have to multiply the velocity by 6 because pros is weird and move_velocity() reads in +/- 600 for blues
		left_drive.move_velocity((drive + turn)*6);
		right_drive.move_velocity((drive - turn)*6);

		// set button bindings and velocity of the intake. we multiply the percent by 2 because its a green motor,
		// 		and move_velocity() reads in +/- 200 for green motors. also we want to make code easier to read
		//		if u want to change the percent speed that the motor moves, change the number being mutiplied by 2
		if(master.get_digital(DIGITAL_B) == 1) {
			intake.move_velocity(50*2);
		} else if (master.get_digital(DIGITAL_Y) == 1) {
			intake.move_velocity(-50*2);
		} else {
			intake.move_velocity(0);
		}

		// set button bindings and velocity of arm. again multiply by 2
		if(master.get_digital(DIGITAL_R2) == 1) {
			arm.move_velocity(50*2);
		} else if(master.get_digital(DIGITAL_R1) == 1) {
			arm.move_velocity(-50*2);
		} else {
			arm.move_velocity(0);
		}

		// toggles pneumatics for wings
		// currently doesnt work need to fix
		if(master.get_digital(DIGITAL_A) == 1) {
			wingToggle ^= false;
		}
		wings.set_value(wingToggle);

	}
}
