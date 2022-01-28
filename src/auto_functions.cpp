#include "main.h"
#include "auto_functions.hpp"

PID chassis_y_pid;
PID chassis_turn_pid;

Timer exit_timer;
Timer timeout_timer;

void move_to_point(int &step, int direction, Vector2D target, double max_power, double timeout, PIDVariables y_pid_vars, PIDVariables turn_pid_vars) {

    double heading = (direction == FORWARD) ? global_angle : (global_angle);

    Vector2D error = target - GlobalPosition;
    Vector2D local_current = GlobalPosition.getHeadingBased(heading);
    Vector2D local_target = target.getHeadingBased(heading);
    bool exit_condition = false;

    switch (step) {
        case FUNC_SETUP :
        chassis_y_pid.set_PID_constants(y_pid_vars);
        chassis_turn_pid.set_PID_constants(turn_pid_vars);
        timeout_timer.reset();
        step = FUNC_BODY;
        break;
        case FUNC_BODY :
        chassis_y_pid.set_PID_variables(local_target.y, max_power, -max_power, 15);
        chassis_turn_pid.set_PID_variables(local_target.x, max_power, -max_power, 2);

        power_drive(chassis_y_pid.output(local_current.y), chassis_turn_pid.output(local_current.x) * direction);

        if (error.getLength() > 3) exit_timer.reset();

        exit_condition = (exit_timer.delta_time() > 250);

        if (exit_condition == true || timeout_timer.delta_time() > timeout) {
            step = FUNC_COMPLETE;
            power_drive(0,0);
        }
        break;
        case FUNC_COMPLETE :
        power_drive(0,0);
        break;
    }
}


void rotate_to_heading(int &step, double heading, double max_power, double timeout, PIDVariables turn_pid_vars) {
    double error = heading - imu_value();
    bool exit_condition = false;

    switch (step) {
        case FUNC_SETUP :
        chassis_turn_pid.set_PID_constants(turn_pid_vars);
        timeout_timer.reset();
        step = FUNC_BODY;
        break;
        case FUNC_BODY :
        chassis_turn_pid.set_PID_variables(heading, max_power, -max_power, 10);

        power_drive(0,chassis_turn_pid.output(imu_value()));

        if (fabs(error) > 1.5) exit_timer.reset();
        else if (exit_timer.delta_time() > 250) exit_condition = true;

        if (exit_condition == true || timeout_timer.delta_time() > timeout) {
            step = FUNC_COMPLETE;
            power_drive(0,0);
        }
        break;
        case FUNC_COMPLETE :
        power_drive(0,0);
        break;
    }
}


void move_mogo(int &mogo_state) {
    if (mogo_state == 1) {
        if (mogoSensor.get_value() != 1) {
            mobileGoalLeftMotor = -40;
            mobileGoalRightMotor = -40;
        }
        else {
            mobileGoalLeftMotor = 0;
            mobileGoalRightMotor = 0;
            mobileGoalLeftMotor.tare_position();
            mobileGoalRightMotor.tare_position();
            mogo_state = 3;
        }
	}
	else if (mogo_state == 3) {
		mobileGoalLeftMotor = 0;
		mobileGoalRightMotor = 0;
	}
	else {
		mobileGoalLeftMotor.move_absolute(MOGO_UP, 100);
		mobileGoalRightMotor.move_absolute(MOGO_UP + 15, 100);
	}
}


double mogo_position() {
    return mobileGoalRightMotor.get_position();
}


double in_to_cm(double in) {
    return (in * 2.54);
}


void move_arm(int &arm_state) {
    switch (arm_state) {
        case 0 :
        if (armSensor.get_value() != 1) {
            fourBarLeftMotor = 40;
            fourBarRightMotor = -40;
        }
        else {
            fourBarLeftMotor = 0;
            fourBarRightMotor = 0;
            fourBarLeftMotor.tare_position();
            fourBarRightMotor.tare_position();
        }
        break;
        case 1 :
        fourBarLeftMotor.move_absolute(-2000, 100);
        fourBarRightMotor.move_absolute(2000, 100);
        break;
        case 2 :
        fourBarLeftMotor.move_absolute(-700, 100);
        fourBarRightMotor.move_absolute(700, 100);
        break;
    }
}

double arm_position() {
    return fourBarRightMotor.get_position();
}

void move_intake(int &intake_state) {
    if (intake_state == 1) {
        ringIntake = -127;
    }
    else if (intake_state == 2) {
        ringIntake = 127;
    }
    else {
        ringIntake = 0;
    }
}

void set_claw(bool &claw_state) {
    claw.set_value(claw_state);
}
