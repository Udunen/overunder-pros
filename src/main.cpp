#include "main.h"
#include "lemlib/api.hpp"
#include "PID.cpp"

pros::Controller	master(pros::E_CONTROLLER_MASTER);

// 36:1, 100 RPM, Red gear set
// 18:1, 200 RPM, Green gear set
// 6:1, 600 RPM, Blue gear set

//set up drive motors
pros::Motor			left_front(1, MOTOR_GEAR_BLUE, true);
pros::Motor			left_mid(3, MOTOR_GEAR_BLUE, true);
pros::Motor			left_back(2, MOTOR_GEAR_BLUE, true);
pros::Motor			right_front(4, MOTOR_GEAR_BLUE, false);
pros::Motor			right_mid(5, MOTOR_GEAR_BLUE, false);
pros::Motor			right_back(17,  MOTOR_GEAR_BLUE, false);
pros::Motor_Group	left_drive({left_front, left_mid, left_back});
pros::Motor_Group	right_drive({right_front, right_mid, right_back});



//set up other stuff
pros::Motor			flyWheel(10, MOTOR_GEAR_BLUE, false);
pros::Motor			intake(9, MOTOR_GEAR_GREEN, false);
pros::Motor			arm(8, MOTOR_GEAR_GREEN, false);
pros::ADIDigitalOut	wings('A', false);
pros::Imu			imu(18);
pros::Rotation 		xTracking(7, true);


void matchLoad(int degrees, double millivolts, double seconds) { 
	int wantTime = seconds * 1000 + pros::millis();
	while ( pros::millis() < wantTime ) {
		flyWheel.move_voltage(millivolts);
		arm.move_absolute(degrees, 300);
	}
	flyWheel.move_voltage(0);
	arm.move_absolute(200, 300);	
};

void matchLoad(int degrees, double speed) {
	flyWheel.move_velocity(speed);
	arm.move_absolute(degrees, 300);
}


//lem lib :)
lemlib::Drivetrain drivetrain {
	&left_drive, // left motor group
	&right_drive, // right motor group
	12.5625, // " track width
	lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
	360, // drivetrain rpm is 360
	2 // chase power is 2. If we had traction wheels, it would have been 8
};

// lemlib::TrackingWheel left_tracking_wheels(
// 	&left_drive, // encoder
// 	3.25, // " wheel diameter
// 	-12.4375/2.0, // " offset from tracking center
// 	1 // gear ratio
// );

// lemlib::TrackingWheel right_tracking_wheels(
// 	&right_drive, // encoder
// 	3.25, // " wheel diameter
// 	12.4375/2.0, // " offset from tracking center
// 	1 // gear ratio
// );

lemlib::TrackingWheel x_tracking_wheel(
	&xTracking, // encoder
	lemlib::Omniwheel::NEW_325, // " wheel diameter
	3.95 // " offset from tracking center
);

lemlib::OdomSensors sensors {
	nullptr, // vertical tracking wheel 1
	nullptr, // vertical tracking wheel 2
	&x_tracking_wheel, // horizontal tracking wheel 1
    nullptr, // we don't have a second tracking wheel, so we set it to nullptr
    &imu // inertial sensor
};

// forward/backward PID (probably tuned)
lemlib::ControllerSettings lateralController {
	9.7, // proportional gain (kP)
	0, // integral gain (kI)
	5, // derivative gain (kD)
	3, // anti windup
	1, // small error range, in inches
	100, // small error range timeout, in milliseconds
	3, // large error range, in inches
	500, // large error range timeout, in milliseconds
	10 // maximum acceleration (slew)
};

// turning PID (probably tuned)
lemlib::ControllerSettings angularController {
	2.1, // proportional gain (kP)
	0, // integral gain (kI)
	0, // derivative gain (kD)
	3, // anti windup
	1, // small error range, in degrees
	100, // small error range timeout, in milliseconds
	3, // large error range, in degrees
	500, // large error range timeout, in milliseconds
	1 // maximum acceleration (slew)
};

lemlib::Chassis chassis(drivetrain, lateralController, angularController, sensors);




/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize(); // initialize brain screen
	chassis.calibrate();
	while(imu.is_calibrating()) {
		pros::delay(20);
	}
	// master.clear();
	// pros::Task screenTask([&]() {
    //     lemlib::Pose pose(0, 0, 0);
    //     while (true) {
			
    //         // print robot location to the brain screen
    //         // pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
    //         // pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
    //         // pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
	// 		master.print(0, 0, "x: %f", chassis.getPose().x);
	// 		pros::delay(50);
	// 		master.print(1, 0, "y: %f", chassis.getPose().y);
	// 		pros::delay(50);
	// 		master.print(2, 0, "theta: %f", chassis.getPose().theta);
    //         // log position telemetry
    //         lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
    //         // delay to save resources
    //         pros::delay(50);
    //     }
    // });
	left_drive.set_brake_modes(MOTOR_BRAKE_BRAKE);
	right_drive.set_brake_modes(MOTOR_BRAKE_BRAKE);

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
ASSET(test2_txt);
ASSET(test3_txt);
ASSET(test4_txt);
ASSET(test5_txt);
ASSET(test6_txt);
void autonomous() {
	// chassis.calibrate();
	// while(imu.is_calibrating()) {
	// 	pros::delay(20);
	// }
	arm.set_brake_mode(MOTOR_BRAKE_HOLD);
	arm.set_zero_position(arm.get_position());


	left_drive.move_voltage(-12000);
	right_drive.move_voltage(-3000);
	pros::delay(1000);
	left_drive.move_voltage(0);
	right_drive.move_voltage(0);
	matchLoad(1500, -9000, 30);



	chassis.setPose(-43.602, -60.143, 45);

	left_drive.move_voltage(7000);
	right_drive.move_voltage(1000);
	pros::delay(300);
	left_drive.move_voltage(0);
	right_drive.move_voltage(0);


	chassis.turnTo(-37.788, -60, 2000, false, 75);
	chassis.moveToPoint(-37.788, -59.937, 10000, false);
	
	chassis.follow(test2_txt, 8, 13000, false);
	
	chassis.follow(test3_txt, 8, 10000, true);
	chassis.turnTo(12, -30, 50, 2000, false);
	wings.set_value(true);
	chassis.follow(test4_txt, 9, 10000, false);
	wings.set_value(false);
	chassis.follow(test5_txt, 9, 10000, true);
	wings.set_value(true);
	chassis.follow(test6_txt, 9, 10000, false);
	wings.set_value(false);
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
	// //remove this when not testing
	// chassis.calibrate();
	// chassis.setPose(0, 0, 0); // set this to specific points on the field when we're not tuning
	// while(imu.is_calibrating()) {
	// 	pros::delay(20);
	// }

	double drive, turn;
	bool wingToggle = false;
	arm.set_brake_mode(MOTOR_BRAKE_HOLD);
 
	while (true) {
		drive = master.get_analog(ANALOG_LEFT_Y);
		turn = master.get_analog(ANALOG_RIGHT_X) / 2.5 * ((int)(abs(drive)/60)*0.5+1);

		// we could do chassis.curvature instead, where the right stick 
		// controls the curvature the robot drives with instead of the speed it turns
		// chassis.arcade(drive, turn);
		// chassis.curvature(drive, turn, 2.7);
		left_drive.move_velocity((drive + turn)*6);
		right_drive.move_velocity((drive - turn)*6);	

		
		// set button bindings and velocity of the intake
		if(master.get_digital(DIGITAL_R2)) {
			intake.move_voltage(12000);
		} else if (master.get_digital(DIGITAL_R1)) {
			intake.move_voltage(-12000);
		} else {
			intake.move_velocity(0);
		}

		// set button bindings and velocity of arm
		if(master.get_digital(DIGITAL_UP)) {
			arm.move_voltage(6000);
		} else if(master.get_digital(DIGITAL_DOWN)) {
			arm.move_voltage(-6000);
		} else if (master.get_digital(DIGITAL_Y) ) {
			arm.move_absolute(800, 300);
		} else {
			arm.move_velocity(0);
		}

		// toggles pneumatics for wings
		// wont work if its anything else idk why
		// tried wingToggle != wingToggle and ^=, neither worked so we do this
		if(master.get_digital_new_press(DIGITAL_A)) {
			if(wingToggle == false) {
				wings.set_value(true);
				wingToggle = true;
			} else {
				wings.set_value(false);
				wingToggle = false;
			}
		}

		// flywheel :)
		if(master.get_digital(DIGITAL_L2)) {
			flyWheel.move_velocity(350);
		} else if (master.get_digital(DIGITAL_L1)) {
			flyWheel.move_velocity(-350);
		} else {
			flyWheel.move_velocity(0);
		}

		if(master.get_digital_new_press(DIGITAL_LEFT)) {
			flyWheel.move_velocity(-350);
			pros::delay(35000);
			flyWheel.move_velocity(0);
		}

		
	}
}
