//REAL CODE

#include "main.h"
//#include "lemlib/api.hpp" //IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "pros/screen.h"
#include "pros/screen.hpp"
#include <iostream>
#include <string>

//lemlib::Pose position = (0, 0, 0);

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

pros::MotorGroup left_motors({-4, -2, 20}); // left motor ports (stacked 20)
pros::MotorGroup right_motors({10, -3, 9}); // right motor ports (stacked 3)
pros::Imu imu(15);                                 // imu port

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(19);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc0(6);
pros::Rotation verticalEnc1(-7);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::OLD_325, 0.5);
lemlib::TrackingWheel vertical0(&verticalEnc0, lemlib::Omniwheel::OLD_325, 3);
lemlib::TrackingWheel vertical1(&verticalEnc1, lemlib::Omniwheel::OLD_325, 3);

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motors, // left motor group
							&right_motors, // right motor group
							10, // 10 inch track width
							lemlib::Omniwheel::OLD_325, // using old 3.25" omnis
							360, // drivetrain rpm is 360
							2 // horizontal drift is 2 (for now)
							);

// lateral motion controller
lemlib::ControllerSettings linearController(2, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            5, // derivative gain (kD)
                                            0, // anti windup
                                            0, // small error range, in inches
                                            0, // small error range timeout, in milliseconds
                                            0, // large error range, in inches
                                            0, // large error range timeout, in milliseconds
                                            0 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(0.3, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             5, // derivative gain (kD)
                                             0, // anti windup
                                             0, // small error range, in degrees
                                             0, // small error range timeout, in milliseconds
                                             0, // large error range, in degrees
                                             0, // large error range timeout, in milliseconds
                                             0  // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(&vertical0, // vertical tracking wheel
	&vertical1, // vertical tracking wheel 2
	&horizontal, // horizontal tracking wheel
	nullptr, // horizontal tracking wheel 2 (DNE)
	&imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
	10, // minimum output where drivetrain will move out of 127
	1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
	10, // minimum output where drivetrain will move out of 127
	1.019 // expo curve gain
);

lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);


//MOTORS
pros::Motor intake (-12);
pros::Motor ladybrown(-1);
const int INTAKE_SPEED = 122;
const int LADYBROWN_SPEED = 85;

//MOGO MECH
bool state1 = LOW;
pros::adi::DigitalOut mogomech0 ('H', state1);
pros::adi::DigitalOut mogomech1 ('G', state1);
bool mgm = false;

//SENSORS
pros::Rotation lbrs (-11); //lady brown rotation sensor (lbrs)

//LADY BROWN
const int numStates = 3;
int states[numStates] = {0, 300, 900};
int curState = 0;
int target = 0;


void nextState()
{
	curState += 1;
	if (curState == numStates)
		curState = 0;
	target = states[curState];
}

void liftControl()
{
	pros::screen::print(
		pros::E_TEXT_LARGE_CENTER,
		1,
		std::to_string(lbrs.get_position()).c_str()
	);

	double kp = 0.2; //TUNE (smaller = less reactive)
	double error = target - lbrs.get_position();
	double velocity = kp * error;
	ladybrown.move(velocity);
}

void printEncs()
{
	pros::screen::print(
		pros::E_TEXT_LARGE_CENTER,
		1,
		std::to_string(verticalEnc0.get_position()).c_str()
	);
	pros::screen::print(
		pros::E_TEXT_LARGE_CENTER,
		3,
		std::to_string(verticalEnc1.get_position()).c_str()
	);
	pros::screen::print(
		pros::E_TEXT_LARGE_CENTER,
		5,
		std::to_string(horizontalEnc.get_position()).c_str()
	);
}

// void baseOdometry()
// {
// 	double prevEncdL = 0;
// 	double prevEncdR = 0;
// 	double prevEncdLat = 0;
// 	double prevAngle = 0;

// 	int count = 0;
// 	while(!COMPETITION_MODE || competition::is_autonomouse())
// 	{
// 		encdL = verticalEnc0.getPosition();
// 		encdR = verticalEnc1.getPosition();
// 		double encdChangeL = (encdL - prevEncdL);
// 		double encdChangeR = (encdR - prevEncdR);
// 		double sumEncdChange = encdChangeL + encdChangeR;
// 		double deltaAngle = (encdChangeL - encdChangeR)/baseWidth;
// 		double halfDeltaAngle = deltaAngle/2;
// 		position.angle += deltaAngle;
// 		if(deltaAngle == 0)
// 		{
// 			position.x += sumEncdChange/2 * sin(position.angle);
// 			position.y += sunEndChange/2 * cos(position.angle);
// 		}
// 	}
// }

// #if USING_LATERAL_ENCD
// 	double encdLat = (double) horizontalEnc.get_position() * inPerDegLat;
// 	double encdChangeLat = encdLat - prevEncdLat;
// 	double latShift = encdChangeLat;
// 	if (deltaAngle) latShift = (encdChangeLat/deltaAngle) * sin(halfDeltaAngle)*2.0;

// 	double xShift = latShift * cos(prevAngle + halfDeltaAngle);
// 	double yShift = latShift * -sin(prevAngle + halfDeltaAngle);
// 	position.x += xShift;
// 	position.y += yShift;
// #endif


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");
	pros::lcd::register_btn1_cb(on_center_button);
	chassis.calibrate();
	
	// pros::Task liftControlTask([]{
	// 	while (true)
	// 	{
	// 		liftControl();
	// 		pros::delay(10);
	// 	}
	// });
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

ASSET(path_jerryio_txt);
void autonomous()
{
	chassis.setPose(-62.7, 0.8, 50, 5000);
	chassis.moveToPose(-41, 13, 345, 1000);
	chassis.moveToPose(-66.5, 65.5, 345, 1000);
	chassis.moveToPose(-38.6, -12.4, 215, 1000);
	chassis.moveToPose(-65.8, -66.5, 30, 1000);
	chassis.moveToPose(-4.9, -5.5, 120, 1000);
	chassis.moveToPose(29.4, -20.4, 70, 1000);
	chassis.moveToPose(53.5, -11.2, 170, 1000);
	chassis.moveToPose(64.3, -67.6, 0, 1000);
	chassis.moveToPose(57.2, 18.6, 0, 1000);
	chassis.moveToPose(64.3, 65.3, 0, 1000);
	//chassis.follow(path_jerryio_txt, 15, 60000);
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
	pros::Controller master(pros::E_CONTROLLER_MASTER);

	//STARTING STATES
	lbrs.reset_position();

	while (true) {
		//Prints status of the emulated screen LCDs
		pros::lcd::print(
			0, "%d %d %d",
			(pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
			(pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
			(pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0
		);
		//Prints lady brown rotation sensor position
		pros::screen::print(
			pros::E_TEXT_LARGE_CENTER,
			1,
			std::to_string(lbrs.get_position()).c_str()
		);

		//Arcade control scheme
		int dir = master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int turn = master.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
		left_motors.move(dir + turn);                  // Sets left motor voltage
		right_motors.move(dir - turn);                 // Sets right motor voltage
		pros::delay(20);                          // Run for 20 ms then update

		//Intake
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
			intake.move(INTAKE_SPEED);
		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
			intake.move(-INTAKE_SPEED);
		else
			intake.move(0);

		//MGM
		if (master.get_digital_new_press(DIGITAL_UP))
			mgm = !mgm;
		if(mgm)
		{
			mogomech0.set_value(true);
			mogomech1.set_value(true);
		}
		else
		{
			mogomech0.set_value(false);
			mogomech1.set_value(false);
		}

		//LADY BROWN
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X))
			//nextState();
			ladybrown.move(-LADYBROWN_SPEED);
		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y))
			ladybrown.move(LADYBROWN_SPEED);
		else
			ladybrown.move(0);
	}
}


		//IGNORE (tism)

		/*
		//LADY DIDDY
		//STAGE 1: 31913
		//22460
		
		const int LB_SPEED = 85;
		bool sendBack = false;

		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X) && lbrs.get_position() > 31930)
		{
			ladybrown.move(70);
		}
		else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
		{
			ladybrown.move(70);
		}
		else
			ladybrown.move(10);

		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X) && lbrs.get_position() > 22460)
		{
			ladybrown.move(LB_SPEED);
		}
		else if (lbrs.get_position() <= 22460)
			sendBack = true;
		else
			ladybrown.move(0);

		if (sendBack)
		{
			ladybrown.move(-LB_SPEED);
			if (lbrs.get_position() > 31930)
			{
				ladybrown.move(0);
				sendBack = false;
			}
		}
		*/