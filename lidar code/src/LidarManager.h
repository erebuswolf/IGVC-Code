#ifndef _Lidar_Manager_h
#define _Lidar_Manager_h
#include "Lidar.h"
#include "cv.h"
#include "highgui.h"
#include <vector>

using namespace cv;
class LidarManager{
	private:
		Mat mag;
		Mat ang;
		Mat x;
		Mat y;
		Point3f translation;
		//variable to describe the angle between scan values
		double angle_step;
		double angle_offset;
		int convertScan(unsigned int [], int);
		void resetAngles();

	public:
		LidarManager();
		~LidarManager();
		Lidar* lidar;
		int getScan();
		//radius in the same units as the lidars output (cm default) for when an object is too close
		int danger_radius;
		vector<int> dangerPoints;
		
};

#endif
