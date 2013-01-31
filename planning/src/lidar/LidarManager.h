/**
*	LidarManager.h
*	Author: Jesse Fish
*
*/


#ifndef _Lidar_Manager_h
#define _Lidar_Manager_h

#include "Lidar.h"
#include "cv.h"
#include "highgui.h"
#include <vector>

using namespace cv;

struct LidarData{
	int numPoints;
	
	CvMat *untouchedPoints;
	
	CvMat *scanPoints;
	CvMat *R_ScaledScanPoints;
	
	CvMat *xb;
	CvMat *yb;
	CvMat *x;
	CvMat *y;
	
	int thresholdedPoints [SickLMS2xx::SICK_MAX_NUM_MEASUREMENTS];
	long time_stamp;
};

class LidarManager{
	public:
		LidarManager();
		~LidarManager();
		void init(string);
		int getScan();
		Lidar* lidar;
		
		int convertLast(double angle_orientation,double walldepth,double thresh=1000);
		//stores the magnitudes from the lidar plus some constant: used for obstical drawing
		CvMat* mag;
		CvMat * magb;
		//radius in the same units as the lidars output (cm default) for when an object is too close
		int danger_radius;
		//indexes of points within the danger scan
		//	vector<int> dangerPoints;

		//Struct of xy point from the last scan bundled with a timesamp	
		LidarData lidarData;
		
	private:
//stores the raw magnitudes from the lidar
		CvMat* ang;
		//point to describe the translation between the lidar and the center of the robot
		Point_<double> translation;
		//variable to describe the angle between scan values
		double angle_offset;
		void resetAngles(double);
		int convertScan(unsigned int [], int,double,double,double);
};
#endif

