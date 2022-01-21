#include "main.h"
#include "auto_functions.hpp"

PID chassis_y_pid;
PID chassis_turn_pid;

Timer exit_timer;
Timer timeout_timer;

void move_to_point(int &step, Vector2D target, double timeout, PIDVariables y_pid_vars, PIDVariables turn_pid_vars) {
    Vector2D error = target - GlobalPosition;
    bool exit_condition = false;

    switch (step) {
        case FUNC_SETUP :
        chassis_y_pid.set_PID_constants(y_pid_vars);
        chassis_turn_pid.set_PID_constants(turn_pid_vars);
        timeout_timer.reset();
        step = FUNC_BODY;
        break;
        case FUNC_BODY :
        chassis_y_pid.set_PID_variables(error.getLength(), 127, -127, 10);
        chassis_turn_pid.set_PID_variables(error.getAngle(), 127, -127, 10);

        power_drive(chassis_y_pid.output(0), chassis_turn_pid.output(imu.get_rotation()));

        if (error.getLength() > 1) exit_timer.reset();

        exit_condition = (exit_timer.delta_time() > 250 || timeout_timer.delta_time() > timeout);

        if (exit_condition == true) {
            step = FUNC_COMPLETE;
        }
        break;
        case FUNC_COMPLETE :
        power_drive(0,0);
        break;
    }
}
