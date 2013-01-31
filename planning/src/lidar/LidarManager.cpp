/**
*	LidarManager.cpp
*	Author: Jesse Fish
*
*/

#include "LidarManager.h"
#include <iostream>
#include <fstream>
#include <math.h>
using namespace cv;


LidarManager::LidarManager(){

  
  //these are read in from the config file
  angle_offset=0;
  danger_radius=0;
  
  mag= cvCreateMat(SickLMS2xx::SICK_MAX_NUM_MEASUREMENTS,1,CV_64F);
  magb= cvCreateMat(SickLMS2xx::SICK_MAX_NUM_MEASUREMENTS,1,CV_64F);
  ang= cvCreateMat(SickLMS2xx::SICK_MAX_NUM_MEASUREMENTS,1,CV_64F);
  
  lidarData.untouchedPoints=cvCreateMat(SickLMS2xx::SICK_MAX_NUM_MEASUREMENTS,2,CV_64F);
  
  lidarData.scanPoints=cvCreateMat(SickLMS2xx::SICK_MAX_NUM_MEASUREMENTS,2,CV_64F);
  lidarData.R_ScaledScanPoints=cvCreateMat(SickLMS2xx::SICK_MAX_NUM_MEASUREMENTS,2,CV_64F);
  
  lidarData.x=cvCreateMatHeader(SickLMS2xx::SICK_MAX_NUM_MEASUREMENTS,1,CV_64F);
  lidarData.y=cvCreateMatHeader(SickLMS2xx::SICK_MAX_NUM_MEASUREMENTS,1,CV_64F);
  lidarData.xb=cvCreateMatHeader(SickLMS2xx::SICK_MAX_NUM_MEASUREMENTS,1,CV_64F);
  lidarData.yb=cvCreateMatHeader(SickLMS2xx::SICK_MAX_NUM_MEASUREMENTS,1,CV_64F);

  cvGetCols(lidarData.scanPoints,lidarData.x,0,1);
  cvGetCols(lidarData.scanPoints,lidarData.y,1,2);
  
  cvGetCols(lidarData.R_ScaledScanPoints,lidarData.xb,0,1);
  cvGetCols(lidarData.R_ScaledScanPoints,lidarData.yb,1,2);
}

void LidarManager::init(string config_filepath){
  //read the config file in and read the angle offset
  //and translation offset
  ifstream config_file (config_filepath.c_str());
  
  if (config_file.is_open())
  {
	config_file>>angle_offset;
	//	cout<<angle_offset<<endl;
	config_file.ignore(300,'\n');
	config_file>>translation.x;
	//	cout<<translation.x<<endl;
	config_file.ignore(300,'\n');
	config_file>>translation.y;
	//cout<<translation.y<<endl;
	config_file.ignore(300,'\n');
	config_file>>danger_radius;
	config_file.close();
  }
  else cerr << "Unable to open lidar config file" << endl; 
  
  resetAngles(0);
}

void LidarManager::resetAngles(double angle_orientation){
  if(lidar==NULL){
	cerr<<"error called resetAngles before assigning a lidar manager to the lidar\n";
	return;
  }
  
  for(unsigned int j = 0; j < SickLMS2xx::SICK_MAX_NUM_MEASUREMENTS; j++) {
	cvmSet(ang,j,0,j*(lidar->angle_step)+angle_offset+angle_orientation);
  }
}

int LidarManager::getScan(){
  lidar->getNewValues();
  return lidar->num_values;
}
///set walldepth to 0 if you want none of the calculations to be used
int LidarManager::convertLast(double angle_orientation,double walldepth,double thresh){
  return convertScan(lidar->values,lidar->num_values ,angle_orientation,walldepth,thresh);
  
}
///set walldepth to 0 if you want no wall depth calculations
int LidarManager::convertScan(unsigned int  values[], int num_values,double angle_orientation_rad,double walldepth,double thresh){
  //conversion constant from cm to meters
  int cm_2_meter=100;
  float rad2deg=180/3.141593;
  
  
  //reset the angle values to be at the position wer are facing
  resetAngles(angle_orientation_rad *rad2deg);
  
  //dangerPoints.clear();
  
  //return value that determines if we are in danger
  int return_value=10000;
  
  for(unsigned int i = 0; i <num_values; i++) {
	if(values[i]<danger_radius){
	  //the value is within the danger radius
//		printf("value less is %d at %d\n",values[i],i);
		if(values[i]<return_value)
			return_value=values[i]; 
	}
	
	//set the mag matrix to the radiai of the scan and the other one to the scan plus a bit
	if(values[i]/cm_2_meter > thresh){
	  cvmSet(mag,i,0,thresh);
	  lidarData.thresholdedPoints[i]=1;
	}else{
	  cvmSet(mag,i,0,double(values[i])/cm_2_meter);
	  lidarData.thresholdedPoints[i]=0;
	}
	if(walldepth!=0){
	  cvmSet(magb,i,0,double(values[i])/cm_2_meter+walldepth);
	}
  }
  
  //store important data
  lidarData.numPoints=num_values;
  lidarData.time_stamp=lidar->time_stamp;
  
  //convert to polar coordinates
  cvPolarToCart(mag,ang,lidarData.x,lidarData.y,true);
  if(walldepth!=0){
	cvPolarToCart(magb,ang,lidarData.xb,lidarData.yb,true);
  }
  //add our position value to the data
  double theta=angle_orientation_rad+angle_offset*rad2deg;
  Point_<double> rotated_translation;
  rotated_translation.x=translation.x *cos(theta)-translation.y*sin(theta);
  rotated_translation.y=translation.x *sin(theta)+translation.y*cos(theta);

  //need to rotate the position value by the angle_orientation_rad+angle_offset
  
  cvAddS(lidarData.x,cvScalar(rotated_translation.x),lidarData.x);
  cvAddS(lidarData.y,cvScalar(rotated_translation.y),lidarData.y);
  cvCopy(lidarData.scanPoints ,lidarData.untouchedPoints);
  
  //add our position value to the data
  if(walldepth!=0){
	cvAddS(lidarData.xb,cvScalar(rotated_translation.x),lidarData.xb);
	cvAddS(lidarData.yb,cvScalar(rotated_translation.y),lidarData.yb);
  }
  return return_value;
}


LidarManager::~LidarManager(){
  cvReleaseMat(&mag);
  cvReleaseMat(&ang);
  cvReleaseMat(&magb);
  
  cvReleaseMat(&lidarData.scanPoints);
  cvReleaseMat(&lidarData.R_ScaledScanPoints);
  
  cvReleaseMat(&lidarData.x);
  cvReleaseMat(&lidarData.y);
  cvReleaseMat(&lidarData.xb);
  cvReleaseMat(&lidarData.yb);
  
}
