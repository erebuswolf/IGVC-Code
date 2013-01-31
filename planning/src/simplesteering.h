
#ifndef _simple_steering_h
#define _simple_steering_h
#include "WorldModel.h"
typedef struct {
	float vel;
	float omega;
} AngularRateHolder;

AngularRateHolder simplesteering(float x, float y,PositionState state,float vel_slope=.5, float vel_sat=1.2,float omega_slope=3/3.1415, float omega_sat=.9,float theta_vel_thresh=3.1415/4);
#endif

