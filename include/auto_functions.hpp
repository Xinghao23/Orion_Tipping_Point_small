#ifndef _AUTO_FUNCTIONS_HPP_
#define _AUTO_FUNCTIONS_HPP_

#include "Vector2D.hpp"
#include "PID.hpp"

#define FUNC_SETUP 0
#define FUNC_BODY 1
#define FUNC_BODY_2 2
#define FUNC_BODY_3 3
#define FUNC_BODY_4 4
#define FUNC_COMPLETE 5

#define FORWARD 1
#define BACKWARD -1

#define ARM_WAITING 4

double in_to_cm(double in);
double imu_heading();

void move_to_point(int &step, int direction, Vector2D target, double accuracy, double max_power, double timeout, PIDVariables y_pid_vars, PIDVariables turn_pid_vars);
void move_to_point(int &step, int direction, Vector2D target, double max_power, double timeout, PIDVariables y_pid_vars, PIDVariables turn_pid_vars);

void rotate_to_heading(int &step, double heading, double accuracy, double max_power, double timeout, PIDVariables turn_pid_vars);
void rotate_to_heading(int &step, double heading, double max_power, double timeout, PIDVariables turn_pid_vars);

void move_mogo(int &mogo_state);

double mogo_position();

void move_arm(int &arm_state);
void stack_mogo(int &arm_step);
double arm_position();

void move_intake(int &intake_state);

void set_claw(bool &claw_state);

#endif