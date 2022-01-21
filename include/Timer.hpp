#ifndef _TIMER_HPP_
#define _TIMER_HPP_

class Timer {
public:
  Timer();
  double delta_time();
  void reset();
private:
  double start_time;
};

#endif
