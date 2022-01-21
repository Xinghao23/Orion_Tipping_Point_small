#ifndef _PID_HPP_
#define _PID_HPP_


class PIDVariables {
public:
  double kp;
  double ki;
  double kd;
  PIDVariables(double p, double i, double d);
};


// PID class //
/*

PID class is used to make PID objects which each have their own PID variables,
constants, and functions.

*/
class PID {
private:
  // PID variables
  double target;
  double error;
  double past_error;
  double integral;
  double derivative;
  double max;
  double min;
  double integral_limit;

  // PID constants

  double kp;
  double ki;
  double kd;

public:
  // constuctor for PID class
  PID();

  // sets PID targets and max motor power values with the
  void set_PID_variables(double target_input, double max_value, double min_value, double integral_lim);

  // sets the values of the PID constants
  void set_PID_constants(double p, double i, double d);
  void set_PID_constants(PIDVariables pid_var);

  // returns the output of your PID
  double output(double current);
};


#endif
