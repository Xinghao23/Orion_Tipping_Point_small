#include "main.h"
#include "auto_functions.hpp"

double imu_heading() {
    double val = (int)((imu.get_heading() + starting_t) * 100);
    return val / 100.0f;
}

double imu_radian_heading() {
    return (imu_heading() / 180 * 3.1415);
}

PID chassis_y_pid;
PID chassis_turn_pid;

Timer exit_timer;
Timer timeout_timer;

void move_to_point(int &step, int direction, Vector2D target, double accuracy, double max_power, double timeout, PIDVariables y_pid_vars, PIDVariables turn_pid_vars) {
    Vector2D error = target - GlobalPosition;
    Vector2D local_current = GlobalPosition.getHeadingBased(imu_radian_heading());
    Vector2D local_target = target.getHeadingBased(imu_radian_heading());
    Vector2D local_error = local_target - local_current;
    bool exit_condition = false;

    switch (step) {
        case FUNC_SETUP :
        chassis_y_pid.set_PID_constants(y_pid_vars);
        chassis_turn_pid.set_PID_constants(turn_pid_vars);
        timeout_timer.reset();
        step = FUNC_BODY;
        break;
        case FUNC_BODY :
        printf("lTx:[%3.2f], lTy:[%3.2f], lCx:[%3.2f], lCy:[%3.2f], lEx:[%3.2f], lEy:[%3.2f]\n", local_target.x, local_target.y, local_current.x, local_current.y, local_error.x, local_error.y);
        chassis_y_pid.set_PID_variables(local_target.y, max_power, -max_power, 15);
        chassis_turn_pid.set_PID_variables(local_target.x, max_power, -max_power, 2);

        power_drive(chassis_y_pid.output(local_current.y), chassis_turn_pid.output(local_current.x) * direction);

        if (error.getLength() > accuracy) exit_timer.reset();

        if (exit_timer.delta_time() > 250) {
            exit_condition = true;
        }

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


void move_to_point(int &step, int direction, Vector2D target, double max_power, double timeout, PIDVariables y_pid_vars, PIDVariables turn_pid_vars) {
    move_to_point(step, direction, target, 2, max_power, timeout, y_pid_vars, turn_pid_vars);
}


void rotate_to_heading(int &step, double heading, double accuracy, double max_power, double timeout, PIDVariables turn_pid_vars) {
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

        if (fabs(error) > accuracy) exit_timer.reset();
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


void rotate_to_heading(int &step, double heading, double max_power, double timeout, PIDVariables turn_pid_vars) {
    rotate_to_heading(step, heading, 1.5, max_power, timeout, turn_pid_vars);
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
    else if (mogo_state == 4) {
        if (mogoSensor.get_value() != 1) {
            mobileGoalLeftMotor = -127;
            mobileGoalRightMotor = -127;
        }
        else {
            mobileGoalLeftMotor = -2;
            mobileGoalRightMotor = -2;
            mobileGoalLeftMotor.tare_position();
            mobileGoalRightMotor.tare_position();
        }
    }
    else if (mogo_state == 5) {
        mobileGoalLeftMotor.move_absolute(MOGO_UP, 100);
		mobileGoalRightMotor.move_absolute(MOGO_UP + 20, 100);
    }
	else {
		mobileGoalLeftMotor.move_absolute(MOGO_UP, 100);
		mobileGoalRightMotor.move_absolute(MOGO_UP + 20, 100);
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
            fourBarLeftMotor = 127;
            fourBarRightMotor = -127;
        }
        else {
            fourBarLeftMotor = 2;
            fourBarRightMotor = -2;
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
        case 3 :
        fourBarLeftMotor.move_absolute(-900, 30);
        fourBarRightMotor.move_absolute(900, 30);
        break;
        case ARM_WAITING :
        break;
    }
}

void stack_mogo(int &arm_step) {
    switch(arm_step) {
        case FUNC_SETUP :
        arm_step = FUNC_BODY;
        arm_state = ARM_WAITING;
        break;
        case FUNC_BODY :
        // ensure that the arm is up 
        if (arm_position() < 1900) {
            fourBarLeftMotor.move_absolute(-2000, 100);
            fourBarRightMotor.move_absolute(2000, 100);
        }
        else arm_step = FUNC_BODY_2;
        break;
        case FUNC_BODY_2 :
        // arm down slowly 
        if (arm_position() > 930) {
            fourBarLeftMotor.move_absolute(-900, 60);
            fourBarRightMotor.move_absolute(900, 60);
        }
        else {
            claw_state = false;
            arm_step = FUNC_BODY_3;
        }
        break;
        case FUNC_BODY_3 :
        if (arm_position() < 1900) {
            fourBarLeftMotor.move_absolute(-2000, 70);
            fourBarRightMotor.move_absolute(2000, 70);
        }
        else {
            arm_step = FUNC_COMPLETE;
        }
        break;
        case FUNC_COMPLETE :
        break;
    }
}

double arm_position() {
    return fourBarRightMotor.get_position();
}

bool intake_jammed = false;
Timer intake_timer;

bool intake_is_running() {
    if (fabs(ringIntake.get_actual_velocity()) < 2) {
        return false;
    } 
    else {
        return true;
    }
}

void move_intake(int &intake_state) {
    if (intake_state == 1 && intake_jammed == false) {
        ringIntake = -127;

        if (intake_is_running() == false) {
            intake_jammed = true;
            intake_timer.reset();
        }
    }
    else if (intake_jammed == true) {
        if (intake_timer.delta_time() < 500) {
            if (intake_is_running() == true) {
                intake_jammed =  false;
            }
        }
        else if (intake_timer.delta_time() >= 500 && intake_timer.delta_time() < 750) {
            ringIntake = 127;
        }
        else {
            intake_jammed = false;
        }
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
