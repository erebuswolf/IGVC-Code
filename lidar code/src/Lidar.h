#ifndef _Lidar_h
#define _Lidar_h

#include <time.h>
#include <string>
#include <sicklms2xx/SickLMS2xx.hh>
using namespace std;
using namespace SickToolbox;

class Lidar{
	private:
		string device_str;                      
		SickLMS2xx::sick_lms_2xx_baud_t desired_baud;
		SickLMS2xx *sick_lms_2xx;
	public:
		Lidar(string);
		~Lidar();
		void getNewValues();
		double time_stamp;
		unsigned int values[SickLMS2xx::SICK_MAX_NUM_MEASUREMENTS];
		unsigned int num_values;                                   // Holds the number of measurements returned
};

#endif
