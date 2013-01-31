
#ifndef _Position_State_h
#define _Position_State_h

#include "cv.h"
#include "highgui.h"
using namespace cv;

struct PositionState{
  Point_<double> estPosition;
  double estRotRad;
  Point_<double> crioPosition;
  double crioRotRad;
  double crioRot;
  double vel;
  long timeStamp;
  float sonar;
};

#endif
