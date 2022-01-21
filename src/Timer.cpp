#include "main.h"

Timer::Timer() {
  start_time = millis();
}

void Timer::reset() {
  start_time = millis();
}

double Timer::delta_time() {
  return (millis() - start_time);
}
