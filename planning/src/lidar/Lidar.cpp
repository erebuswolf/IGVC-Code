/**
*	Lidar.cpp
*	Author: Jesse Fish
*
*/



#include "Lidar.h"
#include "TimeKeeper.h"
#include <string>
#include <iostream>
#include <fstream>

using namespace std;
using namespace SickToolbox;

Lidar::Lidar(){
  post_process=false;
}

Lidar::Lidar(  string log_filepath ){
  post_process = true;
  if(post_process)
  {
  	logfile.open(log_filepath.c_str());
  }
  
  if (!logfile.is_open())
  {
  	cout<<"failed to open "<<log_filepath<<endl;
  }
}

void Lidar::init(string config_filepath){
  //string for the baudrate
  string baud;
  
  //read the config file in and read the baudrate and device path
  ifstream config_file (config_filepath.c_str());
  int dev_string_size_limit=50;
  char str[dev_string_size_limit];
  if (config_file.is_open())
  {
	config_file.width(dev_string_size_limit);
	config_file>>str;
	device_str=str;
	config_file.ignore(100,'\n');
	config_file>>str;
	baud=str;
	config_file.close();
  }
  else cerr << "Unable to open lidar config file" << endl; 
  num_values=0;
  for(int i=0;i<SickLMS2xx::SICK_MAX_NUM_MEASUREMENTS;i++)
  {
	//set all values to 20 cm 
	values[i]=20;
  }
  
  if(!post_process){

	//create the SickLMS2xx object and initialize various values
	sick_lms_2xx=new SickLMS2xx(device_str);
	desired_baud=SickLMS2xx::SICK_BAUD_38400;
	if ((desired_baud = SickLMS2xx::StringToSickBaud(baud)) == SickLMS2xx::SICK_BAUD_UNKNOWN) {
	  cerr << "Invalid baud value! Valid values are: 9600, 19200, 38400, and 500000" << endl;
	}
	//	sick_lms_2xx->GetSickScanResolution( );

	/*
	* Initialize the Sick LMS 2xx
	*/
	try {
	  sick_lms_2xx->Initialize(desired_baud);
	}

	catch(...) {
	  cerr << "Initialize failed! Are you using the correct device path?" << endl;
	}

	sick_lms_2xx->SetSickVariant(SickLMS2xx::SICK_SCAN_ANGLE_180,SickLMS2xx::SICK_SCAN_RESOLUTION_100);
	  

	//this value is read in later from the lidar
	angle_step=sick_lms_2xx->GetSickScanResolution();
  //GetSickScanAngle();
  //	cout<<angle_step<<endl;
  }
  else{
  angle_step=1;
  }
}

void Lidar::getNewValues(){
  if(!post_process){
	try {
	  
	  time_stamp=TimeKeeper::GetTime();
	  sick_lms_2xx->GetSickScan(values,num_values);
	}
	/* Catch anything else and exit */ 
	catch(...) {
	  cerr << "An error occurred!" << endl;
	}
  }
  else{
	  if(logfile.is_open()){
		long time;
		logfile>>time;
		logfile>>num_values;
		for(int i=0;i<num_values;i++){
		  logfile>>values[i];
		}
	}
	else{
	cout<<"bad lidar log file\n";
	}
  }
}

Lidar::~Lidar(){
  /*
  * Uninitialize the device
  */
 if(!post_process){
	  try {
		sick_lms_2xx->Uninitialize();
	  }
	  
	  catch(...) {
		cerr << "Uninitialize failed!" << endl;
	  }
  }
  delete sick_lms_2xx;
}
