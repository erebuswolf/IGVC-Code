#include "LidarManager.h"

using namespace cv;
LidarManager::LidarManager(){

//these values should be read in from a config file
	angle_step=0.5;
	//100 cm for danger radius
	danger_radius=75;
	angle_offset=0;
	translation=Point3f(0,0,0);

	dangerPoints.resize(370,0);

	mag= Mat(SickLMS2xx::SICK_MAX_NUM_MEASUREMENTS,1,CV_64F);
	ang= Mat(SickLMS2xx::SICK_MAX_NUM_MEASUREMENTS,1,CV_64F);
	x= Mat(SickLMS2xx::SICK_MAX_NUM_MEASUREMENTS,1,CV_64F);
	y= Mat(SickLMS2xx::SICK_MAX_NUM_MEASUREMENTS,1,CV_64F);

	resetAngles();
}

void LidarManager::resetAngles(){
	for(unsigned int j = 0; j < SickLMS2xx::SICK_MAX_NUM_MEASUREMENTS; j++) {
  	ang.at<double>(j,0)=j*angle_step-angle_offset;
  }
}

int LidarManager::getScan(){
	lidar->getNewValues();
	return convertScan(lidar->values,lidar->num_values);
}

int LidarManager::convertScan(unsigned int  values[], int num_values){
	dangerPoints.clear();
	int return_value=0;
	for(unsigned int i = 0; i <num_values; i++) {
		if(values[i]<danger_radius){
			//the value is within the danger radius
			dangerPoints.push_back(i);
			return_value=1;
		}
  	mag.at<double>(i,0)=values[i]/100.0;
  }
	polarToCart(mag,ang,x,y,true);
	add(x,translation.x,x);
  add(y,translation.y,y);
	return return_value;
}


LidarManager::~LidarManager(){
	mag.release();
	ang.release();
	x.release();
	y.release();
}
