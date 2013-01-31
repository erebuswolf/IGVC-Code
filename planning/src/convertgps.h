/**
*	convertgps.h
*	Author: Jesse Fish
*
*/


#ifndef _convertgps_h
#define _convertgps_h
#include "cv.h"
#include "highgui.h"
using namespace cv;

CvPoint2D32f convertgps(float latitude, float longitude);


#endif