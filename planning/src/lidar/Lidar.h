/**
*	Lidar.h
* 
*	class to encapsulate the lidar hardware, this should be virtualized to allow for file spoofing
*	Author: Jesse Fish
*
*/

#ifndef _Lidar_h
#define _Lidar_h

#include <time.h>
#include <string>
#include <sicklms2xx/SickLMS2xx.hh>
#include <fstream>
using namespace std;
using namespace SickToolbox;

class Lidar{
  public:
	Lidar();
	Lidar(  string log_filepath);
	~Lidar();
	//angle step size of the lidar in degrees
	double angle_step;
	//the constructor does nothing this init function loads a config file and sets up the object
	void init(string);
	//gets new values, not sure why it is virtual, was working on file spoofing
	virtual void getNewValues();
	//the time stamp that the last scan was recieved at
	long time_stamp;
	//array for the values for the lidar
	unsigned int values[SickLMS2xx::SICK_MAX_NUM_MEASUREMENTS];
	//the number measurements returned
	unsigned int num_values;                 // Holds the number of measurements returned
	
  private:
	//device path string
	string device_str;        
	//baudrate for the device
	SickLMS2xx::sick_lms_2xx_baud_t desired_baud;
	//pointer to the device object
	SickLMS2xx *sick_lms_2xx;
	
	bool post_process;
	ifstream logfile;
	
	
};

#endif
