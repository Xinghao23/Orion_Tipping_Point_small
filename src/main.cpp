#include "main.h"

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
Motor mobileGoalRightMotor(7, true);
Motor clawMotor(9);
Motor ringIntake(3);
Imu imu(4);
ADIEncoder leftEncoder(5, 6, true);
ADIEncoder backEncoder(3, 4, true);
ADIDigitalOut claw(8);
ADIDigitalIn mogoSensor(7);
ADIDigitalIn armSensor(2);
Controller master(E_CONTROLLER_MASTER);
bool auto_complete = false;

/*
 * Odometry Task
 * Tracks the position of the robot
*/
Task odom (odom_task, NULL, TASK_PRIORITY_DEFAULT - 1, TASK_STACK_DEPTH_DEFAULT, "ODOM");

void initialize() {
	lcd::initialize();
	while (imu.is_calibrating() == true) {

	}
}

void disabled() {}

void competition_initialize() {}

void autonomous() {

	set_up_auto();

	while (auto_complete == false) {
		if (master.get_digital(DIGITAL_B)) break;

		Skills();

		delay(10);
	}
}

void opcontrol() {

	mobileGoalLeftMotor.tare_position();
	mobileGoalRightMotor.tare_position();
	unsigned int mogoState = 3;
	bool mogo_started = false;
	bool claw_state = false;

	while (true) {
		if (master.get_digital_new_press(DIGITAL_X)) {
			auto_complete = false;
			autonomous();
		}

		lcd::print(0, "mobileGoalLeftMotor: %f", mobileGoalLeftMotor.get_position());
		lcd::print(1, "mobileGoalRightMotor: %f", mobileGoalRightMotor.get_position());
		lcd::print(2, "[%3.2f]:x, [%3.2f]:y, [%3.2f]:h", GlobalPosition.x, GlobalPosition.y, imu_value());

		// Drive Motor Control
		power_drive(master.get_analog(ANALOG_RIGHT_Y), master.get_analog(ANALOG_RIGHT_X));

		// Chainbar and Bar Motor Control
		// Lift Motor Control
	  	fourBarLeftMotor = -master.get_analog(ANALOG_LEFT_Y);
		fourBarRightMotor = master.get_analog(ANALOG_LEFT_Y);

		// Mogo Motor Control
		if (master.get_digital_new_press(DIGITAL_R2)) {
			if (mogo_started == false) {
				mogo_started = true;
				mogo_state = 1;
			}
			else {
				if (mogoState == 2) mogoState = 1;
				else mogoState = 2;
			}
		}

		if (mogoState == 1) {
			if (mogoSensor.get_value() != 1) {
				mobileGoalLeftMotor = -40;
				mobileGoalRightMotor = -40;
			}
			else {
				mobileGoalLeftMotor = 0;
				mobileGoalRightMotor = 0;
				mobileGoalLeftMotor.tare_position();
				mobileGoalRightMotor.tare_position();
				mogoState = 3;
			}
		}
		else if (mogoState == 3) {
			mobileGoalLeftMotor = 0;
			mobileGoalRightMotor = 0;
		}
		else {
			mobileGoalLeftMotor.move_absolute(MOGO_UP, 100);
			mobileGoalRightMotor.move_absolute(MOGO_UP + 15, 100);
		}

		//Claw Motor Control
		if(master.get_digital_new_press(DIGITAL_R1) == 1){
			claw_state = !claw_state;
			claw.set_value(claw_state);
		}

		if(master.get_digital(DIGITAL_L1) == 1) ringIntake = -127;
		else if(master.get_digital(DIGITAL_L2) == 1) ringIntake = 127;
		else ringIntake = 0;

    	delay(20);
	}
}
