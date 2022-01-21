#include "main.h"
using namespace pros;

Motor leftFrontMotor(12);
Motor leftFrontMiddleMotor(16);
Motor leftBackMiddleMotor(13);
Motor leftBackMotor(14);
Motor rightFrontMotor(19);
Motor rightFrontMiddleMotor(18);
Motor rightBackMiddleMotor(17);
Motor rightBackMotor(20);
Motor fourBarLeftMotor(1);
Motor fourBarRightMotor(8);
Motor mobileGoalLeftMotor(2);
Motor mobileGoalRightMotor(7);
Motor clawMotor(9);
Motor ringIntake(3);
ADIDigitalOut claw(8);
ADIDigitalIn mogoSensor(7);
Controller master(E_CONTROLLER_MASTER);

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	//pros::lcd::set_text(1, "Hello PROS User!");

	//pros::lcd::register_btn1_cb(on_center_button);
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
  mobileGoalLeftMotor.tare_position();
	mobileGoalRightMotor.tare_position();
	int mogoState = 0;
	while (true) {
		lcd::print(0, "mobileGoalLeftMotor: %f\n", mobileGoalLeftMotor.get_position());
		lcd::print(1, "mobileGoalRightMotor: %f\n", mobileGoalRightMotor.get_position());
		int rightX = master.get_analog(ANALOG_RIGHT_X);
		int rightY = master.get_analog(ANALOG_RIGHT_Y);
		int leftX = master.get_analog(ANALOG_LEFT_X);
		int leftY = master.get_analog(ANALOG_LEFT_Y);
		// Drive Motor Control
		leftFrontMotor = rightY + rightX;
		leftFrontMiddleMotor = -rightY - rightX;
		leftBackMiddleMotor = rightY + rightX;
		leftBackMotor = -rightY - rightX;
		rightFrontMotor = -rightY + rightX;
		rightFrontMiddleMotor = rightY - rightX;
		rightBackMiddleMotor = -rightY + rightX;
		rightBackMotor = rightY - rightX;


		// Chainbar and Bar Motor Control
		// Lift Motor Control
	  fourBarLeftMotor = -leftY;
		fourBarRightMotor = leftY;
		// Mogo Motor Control
		if(master.get_digital_new_press(DIGITAL_DOWN) == 1){
			if(mogoSensor.get_value() != 1){
				mogoState = 1;
			}
		}
		else if(master.get_digital_new_press(DIGITAL_UP) == 1){
			mogoState = 2;
		}
		else{
			if(mogoState == 1){
				mobileGoalLeftMotor = -127;
				mobileGoalRightMotor = 127;

				if(mogoSensor.get_value() == 1){
					mogoState = 3;
				}
			}
			else if(mogoState == 2){
				mobileGoalLeftMotor.move_absolute(1250, 100);
				mobileGoalRightMotor.move_absolute(-1250, 100);
			}
			else if(mogoState == 3){
				mobileGoalLeftMotor = mobileGoalRightMotor = 0;
				mobileGoalLeftMotor.tare_position();
				mobileGoalRightMotor.tare_position();
			}
		}
		//Claw Motor Control
		if(master.get_digital_new_press(DIGITAL_R1) == 1){
			claw.set_value(1);
		}
		else if(master.get_digital_new_press(DIGITAL_R2) == 1){
		  claw.set_value(0);
		}

		if(master.get_digital(DIGITAL_L1) == 1){
			ringIntake = -127;
		}
		else if(master.get_digital(DIGITAL_L2) == 1){
		  ringIntake = 127;
		}
		else{
			ringIntake = 0;
		}
    delay(20);
	}
}
