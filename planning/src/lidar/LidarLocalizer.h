/**
*	LidarLocalizer.h
*	Author: Jesse Fish
*
*/

#ifndef _Lidar_Localizer_h
#define _Lidar_Localizer_h

#include "cv.h"
#include "highgui.h"
#include "PositionState.h"

using namespace cv;

class LidarLocalizer{
  public:
	LidarLocalizer();
	~LidarLocalizer();
	void loadMap(char*);
	void loadPresetMap();
	void estimatePosition(PositionState*,const  CvMat*,int);
	
  private:
	//this is now useless
//	void find_intersect(double xs, double ys, double xf, double yf, double xc, double yc, double theta,CvMat* dest_val);
	
	//this is now useless
//	void virtualScan(double, double, double,CvMat* ,CvMat* );
	
	double corner_reject_rad;
	double acceptable_line_error;
	
	void display(PositionState* ,const CvMat*,const CvMat*,int);
	void findMapNormals();
	
	void associatePoints(const CvMat*,int);
	void calcBetterPos(PositionState*, CvMat*,int);

	double DistanceFromLine(double cx, double cy, double ax, double ay, double bx, double by);
	
	int num_lines;
	
//	int scanCount;
	double scanAngle;
	double max_dist;
	int algorithm_iterations;
	
	int * line_associations;
	int * line_distances;
	int * ping_count;
	
	
	CvMat* map;
	CvMat startxy;
	CvMat endxy;
	
	CvMat* normals;
	
	CvMat* TW;
	CvMat* TWPerp;
	
};

#endif

