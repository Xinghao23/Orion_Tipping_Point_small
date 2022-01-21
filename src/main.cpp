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
Motor mobileGoalRightMotor(7);
Motor clawMotor(9);
Motor ringIntake(3);
ADIDigitalOut claw(8);
ADIDigitalIn mogoSensor(7);
Controller master(E_CONTROLLER_MASTER);

void initialize() {
	lcd::initialize();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

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
