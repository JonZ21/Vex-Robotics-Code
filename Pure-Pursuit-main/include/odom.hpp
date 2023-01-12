#ifndef ODOM_HPP
#define ODOM_HPP
#include "api.h"


extern double theta;
extern double pos_x;
extern double pos_y;
extern int counter;
extern bool INTAKEOVERRIDE;
extern pros::Mutex mutex;

extern void odom();
extern void odomTask(void* param);
extern void twoSensorOdom();
extern int fwdRotationValue();
extern double innerAVG();
	// R

int rightRotationValue();
int centerRotationValue();
int leftRotationValue();

#endif
