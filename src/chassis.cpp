#include "main.h"
#include "chassis.hpp"

void power_drive(double y, double turn) {
    leftFrontMotor = y + turn;
	leftFrontMiddleMotor = -y - turn;
	leftBackMiddleMotor = y + turn;
	leftBackMotor = -y - turn;
	rightFrontMotor = -y + turn;
	rightFrontMiddleMotor = y - turn;
	rightBackMiddleMotor = -y + turn;
	rightBackMotor = y - turn;
}
