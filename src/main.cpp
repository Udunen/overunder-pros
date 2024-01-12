#include "main.h"
#include "lemlib/api.hpp"

pros::Controller	master(pros::E_CONTROLLER_MASTER);

// 36:1, 100 RPM, Red gear set
// 18:1, 200 RPM, Green gear set
// 6:1, 600 RPM, Blue gear set

pros::Motor			left_front(1, MOTOR_GEAR_BLUE, true);
pros::Motor			left_mid(3, MOTOR_GEAR_BLUE, true);
pros::Motor			left_back(2, MOTOR_GEAR_BLUE, true);
pros::Motor			right_front(4, MOTOR_GEAR_BLUE, false);
pros::Motor			right_mid(5, MOTOR_GEAR_BLUE, false);
pros::Motor			right_back(17,  MOTOR_GEAR_BLUE, false);
pros::Motor_Group	left_drive({left_front, left_mid, left_back});
pros::Motor_Group	right_drive({right_front, right_mid, right_back});
pros::Motor			flyWheel(10, MOTOR_GEAR_BLUE, false);
pros::Motor			intake(9, MOTOR_GEAR_GREEN, false);
pros::Motor			arm(8, MOTOR_GEAR_GREEN, false);
pros::ADIDigitalOut	wings('A', false);
pros::Imu			imu(13);
pros::Rotation 		xTracking(7, true);


lemlib::Drivetrain_t drivetrain {
    &left_drive, // left drivetrain motors
    &right_drive, // right drivetrain motors
    12.4375, // track width
    3.25, // wheel diameter
    360 // wheel rpm
};

lemlib::TrackingWheel left_tracking_wheel(
	&left_drive, // encoder
	3.25, // " wheel diameter
	-12.4375/2, // " offset from tracking center
	5.0/3.0 // gear ratio
);

lemlib::TrackingWheel right_tracking_wheel(
	&right_drive, // encoder
	3.25, // " wheel diameter
	12.4375/2, // " offset from tracking center
	5.0/3.0 // gear ratio
);

lemlib::TrackingWheel x_tracking_wheel(
	&xTracking, // encoder
	3.25, // " wheel diameter
	-6.25, // " offset from tracking center
	1 // gear ratio
);
lemlib::OdomSensors_t sensors {
    &left_tracking_wheel, // vertical tracking wheel 1
    &right_tracking_wheel, // vertical tracking wheel 2
    &x_tracking_wheel, // horizontal tracking wheel 1
    nullptr, // we don't have a second tracking wheel, so we set it to nullptr
    nullptr // inertial sensor
};
// forward/backward PID (untuned)
lemlib::ChassisController_t lateralController {
    10, // kP
    0, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    3 // slew rate
};
// turning PID (untuned)
lemlib::ChassisController_t angularController {
    0, // kP
    0, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    0 // slew rate
};
lemlib::Chassis chassis(drivetrain, lateralController, angularController, sensors);




void screen() {
	// while (true) {
    //     lemlib::Pose pose = chassis.getPose(); // get the current position of the robot
    //     pros::lcd::print(0, "x: %f", pose.x); // print the x position
    //     pros::lcd::print(1, "y: %f", pose.y); // print the y position
    //     pros::lcd::print(2, "heading: %f", pose.theta); // print the heading
    //     pros::delay(10);
    // }
	while (true) {
		// pros::lcd::print(0, "arm: %f", arm.get_position());
		master.print(0, 0, "arm: %f", arm.get_position());
		master.clear_line(0);
		pros::delay(10);
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
	chassis.calibrate();
	chassis.setPose(0, 0, 0);
	pros::Task screenTask(screen);
	arm.set_brake_mode(MOTOR_BRAKE_HOLD);
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
void autonomous() {
	// chassis.calibrate();
	// chassis.setPose(0, 0, 0);
	//chassis.moveTo(0, 20, 5);
	//chassis.moveTo(-12, 24, 5);

	//dont do this until u tune pid or else
	//chassis.follow("start.txt", 2000, 15, false);
	// arm.move_voltage(6000);
	// while(arm.get_position() < 3000){
	// 	pros::delay(10);
	// }
	// arm.move_voltage(-1000);
	// flyWheel.move_voltage(11000);
	// pros::delay(30000);
	// flyWheel.move_voltage(0);
	// arm.move_absolute(0, -50);
	// while(arm.get_position() > 0){
	// 	pros::delay(10);
	// }  
	// chassis.follow("afterShooting.txt", 10000, 15, true);
	// wings.set_value(true);
	// chassis.follow("wingsOut1.txt", 5000, 15, false);
	// wings.set_value(false);
	// chassis.follow("wingsIn1.txt", 5000, 15, false);
	// wings.set_value(true);
	// chassis.follow("wingsOut2.txt", 5000, 15, false);
	// wings.set_value(false);

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

	double drive, turn;
	bool wingToggle = false;

	while (true) {
		drive = master.get_analog(ANALOG_LEFT_Y);
		turn = master.get_analog(ANALOG_RIGHT_X) / 2.5 * ((int)(abs(drive)/60)*0.5+1);

		// also we change to LemLib driving bc its better pepega, we just have this rn for testing purposes :)
		// since its a blue motor, we have to multiply the velocity by 6 because pros is weird and asks for the rpm instead of percent speed
		left_drive.move_velocity((drive + turn)*6);
		right_drive.move_velocity((drive - turn)*6);

		

		// set button bindings and velocity of the intake. we multiply the percent by 2 because its a green motor,
		// 		and move_velocity() reads in +/- 200 for green motors. also we want to make code easier to read
		//		if u want to change the percent speed that the motor moves, change the number being mutiplied by 2
		if(master.get_digital(DIGITAL_R2)) {
			intake.move_voltage(12000);
		} else if (master.get_digital(DIGITAL_R1)) {
			intake.move_voltage(-12000);
		} else {
			intake.move_velocity(0);
		}

		// set button bindings and velocity of arm. again multiply by 2
		if(master.get_digital(DIGITAL_UP)) {
			arm.move_voltage(6000);
		} else if(master.get_digital(DIGITAL_DOWN)) {
			arm.move_voltage(-6000);
		} else {
			arm.move_velocity(0);
		}

		// toggles pneumatics for wings
		// wont work if its anything else idk why
		if(master.get_digital_new_press(DIGITAL_A)) {
			if(wingToggle == false) {
				wings.set_value(true);
				wingToggle = true;
			} else {
				wings.set_value(false);
				wingToggle = false;
			}
		}

		// shooter :)
		// multiply pct speed by 6 bc its a blue motor
		if(master.get_digital(DIGITAL_L2)) {
			flyWheel.move_voltage(11000);
		} else if (master.get_digital(DIGITAL_L1)) {
			flyWheel.move_voltage(-11000);
		} else {
			flyWheel.move_velocity(0);
		}
	}
}
