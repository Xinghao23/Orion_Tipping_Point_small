#include "main.h"
#include "auto.hpp"

int chassis_step;
int mogo_state;
int arm_state;
int arm_step;
int intake_state;
bool claw_state;
int auto_step;
int auto_part;

int i;

Timer auto_timer_1;

void change_auto_step() {
    auto_step++;
    arm_step = FUNC_SETUP;
    chassis_step = FUNC_SETUP;
}

void change_auto_part() {
    auto_step = 0;
    arm_step = FUNC_SETUP;
    chassis_step = FUNC_SETUP;
}

void set_up_auto() {
    i = 0;
    auto_part = 0;
    auto_step = 0;
    chassis_step = FUNC_SETUP;
    arm_state = 0;
    arm_step = FUNC_SETUP;
    intake_state = 0;
    mogo_state = 4;
    claw_state = false;
    auto_timer_1.reset();
}

// grab B1 and ready for rings
bool SkillsPt1() {
    switch (auto_step) {
        case 0 :
        arm_state = 1;
        if (mogoSensor.get_value() == 1 || auto_timer_1.delta_time() > 1750) change_auto_step();
        break;
        case 1 :
        move_to_point(chassis_step, BACKWARD, Vector2D(75, 30), 70, 2000, PIDVariables(4, 0.15, 10), PIDVariables(5, 0.1, 20));
        if (GlobalPosition.x > 67) mogo_state = MOGO_RAISE;
        if (chassis_step == FUNC_COMPLETE && mogo_position() > 200) change_auto_step();
        break;
        case 2 :
        mogo_state = 5;
        rotate_to_heading(chassis_step, 180, 5, 90, 5000, PIDVariables(3, 0.2, 10));
        if (chassis_step == FUNC_COMPLETE) change_auto_step();
        break;
        case 3 :
        intake_state = 1;
        arm_state = 2;
        move_to_point(chassis_step, BACKWARD, Vector2D(55, 140), 4, 80, 2000, PIDVariables(3, 0.15, 10), PIDVariables(5, 0.12, 20));
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

// grab rings and place B1
bool SkillsPt2() {
    switch (auto_step) {
        case 0 :
        move_to_point(chassis_step, FORWARD, Vector2D(245, 130), 80, 10000, PIDVariables(5, 0.07, 10), PIDVariables(5, 0.1, 20));
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
        intake_state = 2;
        //rotate_to_heading(chassis_step, 135, 90, 2000, PIDVariables(2.5, 0.25, 10));
        //if (chassis_step == FUNC_COMPLETE) {
            change_auto_step();
            arm_state = 0;
            auto_timer_1.reset();
            intake_state = 0;
        //}
        break;
        case 2 :
        if (auto_timer_1.delta_time() > 750) mogo_state = MOGO_DOWN;
        move_to_point(chassis_step, FORWARD, Vector2D(320, 40), 80, 2000, PIDVariables(3, 0.15, 10), PIDVariables(5, 0.1, 20));
        if (chassis_step == FUNC_COMPLETE) {
            change_auto_part();
            intake_state = 0;
            return true;
        }
        break;
    }

    return false;
}

// grab B2, regrab B1
bool SkillsPt3() {
    switch (auto_step) {
        case 0 :
        //rotate_to_heading(chassis_step, 188, 90, 5000, PIDVariables(4.4, 0.2, 10));
        //if (chassis_step == FUNC_COMPLETE) {
            auto_timer_1.reset();
            change_auto_step();
        //}
        break;
        case 1 :
        if (auto_timer_1.delta_time() > 750) mogo_state = MOGO_RAISE;
        move_to_point(chassis_step, BACKWARD, Vector2D(326, 85), 5, 80, 2000, PIDVariables(3, 0.1, 10), PIDVariables(5, 0.1, 20));
        if (chassis_step == FUNC_COMPLETE) change_auto_step();
        break;
        case 2 :
        change_auto_step();
        auto_timer_1.reset();
        break;
        case 3 :
        if (GlobalPosition.x < 293) claw_state = true;
        move_to_point(chassis_step, FORWARD, Vector2D(280, 92), 5, 80, 5000, PIDVariables(3, 0.25, 10), PIDVariables(8, 0.1, 20));
        if (chassis_step == FUNC_COMPLETE) {
            change_auto_step();
            auto_timer_1.reset();
        }
        break;
        case 4 :
        intake_state = 1;
        arm_state = 1;
        rotate_to_heading(chassis_step, 405, 10, 50, 2000, PIDVariables(3.5, 0.1, 10));
        if (chassis_step == FUNC_COMPLETE) {
            change_auto_part();
            return true;
        }
        break;
    }

    return false;
}

// blue side entry, stack B1, and grab YT
bool SkillsPt4() {
    switch (auto_step) {
        case 0 :
        //230y 10ft x
        move_to_point(chassis_step, FORWARD, Vector2D(315, in_to_cm(94)), 80, 3000, PIDVariables(3.5, 0.175, 10), PIDVariables(2, 0.1, 20));
        if (chassis_step == FUNC_COMPLETE) change_auto_step();
        break;
        case 1 :
        move_to_point(chassis_step, FORWARD, Vector2D(in_to_cm(69.5), in_to_cm(105)), 1, 80, 3000, PIDVariables(2.5, 0.175, 10), PIDVariables(3, 0.1, 20));
        if (chassis_step == FUNC_COMPLETE) {
            intake_state = 0;
            change_auto_step();
        }
        break;
        case 2 :
        rotate_to_heading(chassis_step, 348, 60, 2000, PIDVariables(5.5, 0.2, 10));
        if (chassis_step == FUNC_COMPLETE) change_auto_step();
        break;
        case 3 :
        // stack the first goal
        stack_mogo(arm_step);
        if (GlobalPosition.y < in_to_cm(109)) power_drive(60, 0);
        else power_drive(0,0);
        if (arm_step == FUNC_BODY_3) {
            change_auto_step();
            power_drive(0,0);
            arm_state = 1;
        }
        break;
        case 4 :
        if (imu_value() < 250) arm_state = 0;
        rotate_to_heading(chassis_step, 180, 90, 1000, PIDVariables(3.5, 0.1, 20));
        if (chassis_step == FUNC_COMPLETE) {
            arm_state = 0;
            change_auto_step();
        }
        break;
        case 5 :
        if (GlobalPosition.y < in_to_cm(83)) claw_state = true;
        move_to_point(chassis_step, FORWARD, Vector2D(in_to_cm(70), in_to_cm(78)), 4, 60, 10000, PIDVariables(4.5, 0.25, 10), PIDVariables(6, 0.1, 20));
        if (chassis_step == FUNC_COMPLETE) {
            change_auto_part();
            auto_timer_1.reset();
            claw_state = true;
            return true;
        }
        break;
    }

    return false;
}

// stack YT 
bool SkillsPt5() {
    switch (auto_step) {
        case 0 :
        if (auto_timer_1.delta_time() > 750) {
            arm_state = 1;
            power_drive(0,0);
            change_auto_step();
        }
        else {
            power_drive(20, 0);
        }
        break;
        case 1 :
        rotate_to_heading(chassis_step, 5, 1, 40, 2000, PIDVariables(3.5, 0.2, 10));
        if (chassis_step == FUNC_COMPLETE) change_auto_step();
        break;
        case 2 :
        move_to_point(chassis_step, FORWARD, Vector2D(in_to_cm(70.5), in_to_cm(110)), 5, 60, 6000, PIDVariables(4, 0.2, 20), PIDVariables(3, 0.1, 20));
        if (chassis_step == FUNC_COMPLETE) change_auto_step();
        break;
        case 3 :
        rotate_to_heading(chassis_step, 0, 90, 700, PIDVariables(6.5, 0.3, 10));
        if (chassis_step == FUNC_COMPLETE) change_auto_step();
        change_auto_step();
        break;
        case 4 :
        // stack the big goal
        stack_mogo(arm_step);
        double error = (imu_value());
        if (GlobalPosition.y < in_to_cm(110)) power_drive(50, error * 2);
        else power_drive(0,0);
        if (arm_step == FUNC_BODY_3) {
            change_auto_part();
            arm_state = 3;
            auto_timer_1.reset();
            power_drive(-50,0);
            return true;
        }
        break;
    }

    return false;
}

// grab R1 and R2
bool SkillsPt6() {
    switch(auto_step) {
        case 0 :
        if (auto_timer_1.delta_time() > 300) {
            change_auto_step();
            power_drive(0,0);
        }
        break;
        case 1 :
        rotate_to_heading(chassis_step, -90, 10, 90, 5000, PIDVariables(5.5, 0.2, 10));
        if (imu_value() < -80) {
            arm_state = 0;
            mogo_state = 1;
        }
        if (chassis_step == FUNC_COMPLETE) change_auto_step();
        break;
        case 2 :
        move_to_point(chassis_step, FORWARD, Vector2D(85, 272), 10, 90, 8000, PIDVariables(4, 0.15, 10), PIDVariables(3, 0.1, 20));
        if (chassis_step == FUNC_COMPLETE) change_auto_step();
        break;
        case 3 :
        mogo_state = MOGO_RAISE;
        move_to_point(chassis_step, FORWARD, Vector2D(43, 312), 10, 90, 3000, PIDVariables(6, 0.35, 10), PIDVariables(3, 0.1, 20));
        if (GlobalPosition.x < 50) claw_state = true;
        if (chassis_step == FUNC_COMPLETE) change_auto_step();
        break;
        case 4 :
        arm_state = 1;
        if (arm_position() < 500) power_drive(-70, 0);
        else {
            rotate_to_heading(chassis_step, 45, 3, 60, 5000, PIDVariables(3, 0.1, 20));
            if (chassis_step == FUNC_COMPLETE) {
                change_auto_step();
                mogo_state = 4;
                auto_timer_1.reset();
            }
        }
        break;
        case 5 :
        if (auto_timer_1.delta_time() < 500) power_drive(40, 0);
        else {
            change_auto_step();
            auto_timer_1.reset();
            power_drive(0,0);
        }
        break;
        case 6 :
        if (auto_timer_1.delta_time() > 500) {
            move_to_point(chassis_step, BACKWARD, Vector2D(25, 305), 10, 90, 3000, PIDVariables(6, 0.35, 10), PIDVariables(3, 0.1, 20));
            if (GlobalPosition.x < 25) mogo_state = MOGO_RAISE;
        }

        if (chassis_step == FUNC_COMPLETE) {
            change_auto_part();
            return true;
        }
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
        //rotate_to_heading(chassis_step, 90, 80, 5000, PIDVariables(3, 0.1, 10));
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
        case 4 :
        if (SkillsPt5() == true) auto_part++;
        break;
        case 5 :
        //if (SkillsPt6() == true) auto_part++;
        break;
        case 6 :
        break;
    }
}
