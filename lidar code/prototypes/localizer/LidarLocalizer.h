/**
*	LidarLocalizer.h
*	Author: Jesse Fish
*
*/

#ifndef _Lidar_Localizer_h
#define _Lidar_Localizer_h

#include "cv.h"
#include "highgui.h"

using namespace cv;

struct PositionState{
  Point_<float> estPosition;
  float estRot;
  Point_<float> crioPosition;
  float crioRotRad;
  float crioRot;
  long timeStamp;
};

class LidarLocalizer{
  public:
	LidarLocalizer();
	~LidarLocalizer();
	void loadMap(char*);
	void loadPresetMap();
	void estimatePosition(PositionState*, CvMat*);
	
  private:
	//this is now useless
//	void find_intersect(double xs, double ys, double xf, double yf, double xc, double yc, double theta,CvMat* dest_val);
	
	//this is now useless
//	void virtualScan(double, double, double,CvMat* ,CvMat* );
	
	double corner_reject_rad;
	
	void display(PositionState* );
	void findMapNormals();
	
	void associatePoints(CvMat*);
	
	int num_points;
	
	int scanCount;
	double scanAngle;
	
	int * line_associations;
	
	CvMat* map;
	CvMat* normals;
	
	CvMat* rvec;
	CvMat* pingxy;
	
	CvMat* lidarScan;
	CvMat* lidarxy;
	
};

#endif

