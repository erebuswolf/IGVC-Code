#include "Lidar.h"
#include "TimeKeeper.h"
#include <string>
#include <iostream>
#include <fstream>

using namespace std;
using namespace SickToolbox;

Lidar::Lidar(string config_filepath){
	//string for the baudrate
	string baud;

	//read the config file in and read the baudrate and device path
	ifstream config_file ("./lidarconfig.conf");
	int dev_string_size_limit=50;
	char str[dev_string_size_limit];
	if (config_file.is_open())
  {
		config_file.width(dev_string_size_limit);
		config_file>>str;
		device_str=str;
		config_file>>str;
		baud=str;
    config_file.close();
  }
  else cerr << "Unable to open lidar config file" << endl; 
	
	num_values=0;
	for(int i=0;i<SickLMS2xx::SICK_MAX_NUM_MEASUREMENTS;i++)
	{
		values[i]=0;
	}

	//create the SickLMS2xx object and initialize various values
	sick_lms_2xx=new SickLMS2xx(device_str);
	desired_baud=SickLMS2xx::SICK_BAUD_38400;
	if ((desired_baud = SickLMS2xx::StringToSickBaud(baud)) == SickLMS2xx::SICK_BAUD_UNKNOWN) {
    cerr << "Invalid baud value! Valid values are: 9600, 19200, 38400, and 500000" << endl;
  }

	/*
   * Initialize the Sick LMS 2xx
   */
  try {
    sick_lms_2xx->Initialize(desired_baud);
  }

  catch(...) {
    cerr << "Initialize failed! Are you using the correct device path?" << endl;
  }
}

void Lidar::getNewValues(){
  try {
		time_stamp=TimeKeeper::GetTime();
		sick_lms_2xx->GetSickScan(values,num_values);
	}
	/* Catch anything else and exit */ 
  catch(...) {
    cerr << "An error occurred!" << endl;
  }
}

Lidar::~Lidar(){
	/*
   * Uninitialize the device
   */
  try {
    sick_lms_2xx->Uninitialize();
  }
  
  catch(...) {
    cerr << "Uninitialize failed!" << endl;
  }

	delete sick_lms_2xx;
}
