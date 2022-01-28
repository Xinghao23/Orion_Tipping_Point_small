#ifndef _ODOMETRY_HPP_
#define _ODOMETRY_HPP_

#include "Vector2D.hpp"

double DistCM(int a);
void CalculatePosition();
void odom_task(void* param);
void odomDebug();
double global_angle_d();
double gyro_value();
double imu_value();

extern double past_angle;
extern double new_angle;
extern double average_angle;
extern double delta_angle;
extern double past_global_angle;
extern double global_angle;
extern Vector2D GlobalPosition;

extern double starting_x;
extern double starting_y;
extern double starting_t;

extern double delta_enc[2];
extern double past_enc[2];

#endif
