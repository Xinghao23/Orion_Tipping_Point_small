#include "main.h"
#include "auto_functions.hpp"

PID chassis_y_pid;
PID chassis_turn_pid;

Timer exit_timer;
Timer timeout_timer;

void move_to_point_forward(int &step, Vector2D target, double max_power, double timeout, PIDVariables y_pid_vars, PIDVariables turn_pid_vars) {
    Vector2D error = target - GlobalPosition;
    Vector2D local_current = GlobalPosition.getHeadingBased(global_angle);
    Vector2D local_target = target.getHeadingBased(global_angle);
    bool exit_condition = false;

    switch (step) {
        case FUNC_SETUP :
        chassis_y_pid.set_PID_constants(y_pid_vars);
        chassis_turn_pid.set_PID_constants(turn_pid_vars);
        timeout_timer.reset();
        step = FUNC_BODY;
        break;
        case FUNC_BODY :
        chassis_y_pid.set_PID_variables(local_target.y, max_power, -max_power, 5);
        chassis_turn_pid.set_PID_variables(local_target.x, max_power, -max_power, 5);

        power_drive(chassis_y_pid.output(local_current.y), chassis_turn_pid.output(local_current.x));

        if (error.getLength() > 2) exit_timer.reset();

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


void move_to_point_backward(int &step, Vector2D target, double max_power, double timeout, PIDVariables y_pid_vars, PIDVariables turn_pid_vars) {
    Vector2D error = target - GlobalPosition;
    Vector2D local_current = GlobalPosition.getHeadingBased(global_angle);
    Vector2D local_target = target.getHeadingBased(global_angle);
    bool exit_condition = false;

    switch (step) {
        case FUNC_SETUP :
        chassis_y_pid.set_PID_constants(y_pid_vars);
        chassis_turn_pid.set_PID_constants(turn_pid_vars);
        timeout_timer.reset();
        step = FUNC_BODY;
        break;
        case FUNC_BODY :
        chassis_y_pid.set_PID_variables(local_target.y, max_power, -max_power, 5);
        chassis_turn_pid.set_PID_variables(local_target.x, max_power, -max_power, 5);

        power_drive(chassis_y_pid.output(local_current.y), chassis_turn_pid.output(-local_current.x));

        if (error.getLength() > 2) exit_timer.reset();

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
    
}