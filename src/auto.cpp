#include "main.h"
#include "auto.hpp"

int chassis_step;
int mogo_state;
int arm_state;
int intake_state;
bool claw_state;
int auto_step;
int auto_part;

int i;

Timer auto_timer_1;

void change_auto_step() {
    auto_step++;
    chassis_step = FUNC_SETUP;
}

void change_auto_part() {
    auto_step = 0;
    chassis_step = FUNC_SETUP;
}

void set_up_auto() {
    i = 0;
    auto_part = 0;
    auto_step = 0;
    chassis_step = FUNC_SETUP;
    arm_state = 0;
    intake_state = 0;
    mogo_state = 4;
    claw_state = false;
    auto_timer_1.reset();
}

bool SkillsPt1() {
    switch (auto_step) {
        case 0 :
        arm_state = 1;
        if (mogoSensor.get_value() == 1 || auto_timer_1.delta_time() > 1750) change_auto_step();
        break;
        case 1 :
        move_to_point(chassis_step, BACKWARD, Vector2D(80, 30), 50, 2000, PIDVariables(3, 0.15, 10), PIDVariables(5, 0.1, 20));
        if (GlobalPosition.x > 67) mogo_state = MOGO_RAISE;
        if (chassis_step == FUNC_COMPLETE && mogo_position() > 500) change_auto_step();
        break;
        case 2 :
        mogo_state = MOGO_RAISE;
        rotate_to_heading(chassis_step, 180, 5, 90, 5000, PIDVariables(4.5, 0.2, 10));
        if (chassis_step == FUNC_COMPLETE) change_auto_step();
        break;
        case 3 :
        intake_state = 1;
        arm_state = 2;
        move_to_point(chassis_step, BACKWARD, Vector2D(55, 140), 4, 80, 2000, PIDVariables(3, 0.25, 10), PIDVariables(5, 0.12, 20));
        if (chassis_step == FUNC_COMPLETE) change_auto_step();
        break;
        case 4 :
        //rotate_to_heading(chassis_step, 90, 5, 90, 5000, PIDVariables(5.5, 0.2, 10));
        //if (chassis_step == FUNC_COMPLETE) {
            change_auto_part();
            i = 90;
            return true;
        //}
        break;
    }

    return false;
}

bool SkillsPt2() {
    switch (auto_step) {
        case 0 :
        move_to_point(chassis_step, FORWARD, Vector2D(245, 120), 30, 5000, PIDVariables(5, 0.25, 10), PIDVariables(5, 0.1, 20));
        /*
        if (GlobalPosition.x > 10 && GlobalPosition.x > 230) {
            if (auto_timer_1.delta_time() > 750) {
                auto_timer_1.reset();
                if (i == 20) {
                    i = 50;
                }
                else {
                    i = 20;
                }
            }
        }
        */
        if (chassis_step == FUNC_COMPLETE) change_auto_step();
        break;
        case 1 :
        rotate_to_heading(chassis_step, 135, 90, 2000, PIDVariables(5.5, 0.2, 10));
        if (chassis_step == FUNC_COMPLETE) {
            change_auto_step();
            arm_state = 0;
            auto_timer_1.reset();
            intake_state = 0;
        }
        break;
        case 2 :
        if (auto_timer_1.delta_time() > 300) mogo_state = MOGO_DOWN;
        move_to_point(chassis_step, FORWARD, Vector2D(320, 50), 80, 2000, PIDVariables(3, 0.25, 10), PIDVariables(5, 0.1, 20));
        if (chassis_step == FUNC_COMPLETE) {
            change_auto_part();
            return true;
        }
        break;
    }

    return false;
}

bool SkillsPt3() {
    switch (auto_step) {
        case 0 :
        rotate_to_heading(chassis_step, 188, 90, 5000, PIDVariables(4.4, 0.2, 10));
        if (chassis_step == FUNC_COMPLETE) {
            auto_timer_1.reset();
            change_auto_step();
        }
        break;
        case 1 :
        if (auto_timer_1.delta_time() > 750) mogo_state = MOGO_RAISE;
        move_to_point(chassis_step, BACKWARD, Vector2D(323, 80), 10, 80, 2000, PIDVariables(3, 0.25, 10), PIDVariables(5, 0.1, 20));
        if (chassis_step == FUNC_COMPLETE) change_auto_step();
        break;
        case 2 :
        change_auto_step();
        break;
        case 3 :
        move_to_point(chassis_step, FORWARD, Vector2D(274, 90), 10, 80, 2000, PIDVariables(3, 0.25, 10), PIDVariables(5, 0.1, 20));
        if (chassis_step == FUNC_COMPLETE) change_auto_step();
        break;
        case 4 :
        claw_state = true;
        intake_state = 1;
        arm_state = 1;
        rotate_to_heading(chassis_step, 405, 70, 2000, PIDVariables(5.5, 0.2, 10));
        if (chassis_step == FUNC_COMPLETE) {
            change_auto_part();
            return true;
        }
        break;
    }

    return false;
}

bool SkillsPt4() {
    switch (auto_step) {
        case 0 :
        //230y 10ft x
        move_to_point(chassis_step, FORWARD, Vector2D(in_to_cm(126), 300), 80, 2000, PIDVariables(6, 0.25, 10), PIDVariables(3, 0.1, 20));
        break;
    }

    return false;
}


void Skills() {
    move_mogo(mogo_state);
    move_arm(arm_state);
    move_intake(intake_state);
    set_claw(claw_state);
    switch (auto_part) {
        case 0 :
        if (SkillsPt1() == true) auto_part++;
        break;
        case 1 :
        if (SkillsPt2() == true) auto_part++;
        break;
        case 2 :
        if (SkillsPt3() == true) auto_part++;
        break;
        case 3 :
        if (SkillsPt4() == true) auto_part++;
        break;
    }
}
