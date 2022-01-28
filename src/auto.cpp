#include "main.h"
#include "auto.hpp"

int chassis_step;
int mogo_state;
int arm_state;
int intake_state;
bool claw_state;
int auto_step;

Timer auto_timer_1;

void change_auto_step() {
    auto_step++;
    chassis_step = FUNC_SETUP;
}

void set_up_auto() {
    auto_step = 0;
    chassis_step = FUNC_SETUP;
    arm_state = 0;
    intake_state = 0;
    mogo_state = MOGO_DOWN;
    claw_state = false;
    auto_timer_1.reset();
}

void Skills() {
    move_mogo(mogo_state);
    move_arm(arm_state);
    move_intake(intake_state);
    set_claw(claw_state);
    switch (auto_step) {
        case 0 :
        arm_state = 1;
        if (mogoSensor.get_value() == 1 || auto_timer_1.delta_time() > 2500) change_auto_step();
        break;
        case 1 :
        move_to_point(chassis_step, BACKWARD, Vector2D(87, 30), 70, 5000, PIDVariables(3, 0.15, 10), PIDVariables(5, 0.1, 20));
        if (GlobalPosition.x > 67) mogo_state = MOGO_RAISE;
        if (chassis_step == FUNC_COMPLETE && mogo_position() > 500) change_auto_step();
        break;
        case 2 :
        mogo_state = MOGO_RAISE;
        rotate_to_heading(chassis_step, 180, 90, 5000, PIDVariables(4, 0.2, 10));
        if (chassis_step == FUNC_COMPLETE) change_auto_step();
        break;
        case 3 :
        intake_state = 1;
        move_to_point(chassis_step, BACKWARD, Vector2D(55, 117), 80, 5000, PIDVariables(3, 0.15, 10), PIDVariables(5, 0.1, 20));
        if (chassis_step == FUNC_COMPLETE) change_auto_step();
        break;
        case 4 :
        rotate_to_heading(chassis_step, 90, 90, 5000, PIDVariables(4, 0.2, 10));
        if (chassis_step == FUNC_COMPLETE) change_auto_step();
        break;
        case 5 :
        move_to_point(chassis_step, FORWARD, Vector2D(260, 117), 90, 15000, PIDVariables(3, 0.15, 10), PIDVariables(5, 0.1, 20));
        if (chassis_step == FUNC_COMPLETE) change_auto_step();
        break;
    }
}
