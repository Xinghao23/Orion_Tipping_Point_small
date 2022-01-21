#ifndef _AUTO_FUNCTIONS_HPP_
#define _AUTO_FUNCTIONS_HPP_

#include "Vector2D.hpp"
#include "PID.hpp"

#define FUNC_SETUP 0
#define FUNC_BODY 1
#define FUNC_COMPLETE 2

void move_to_point(int &step, Vector2D target, PIDVariables y_pid_vars, PIDVariables turn_pid_vars);

#endif