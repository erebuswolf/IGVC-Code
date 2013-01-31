/*!
 * \file main.cc
 * \brief A simple application using the Sick LMS 2xx driver.
 *
 * Code by Jason C. Derenick and Thomas H. Miller.
 * Contact derenick(at)lehigh(dot)edu
 *
 * The Sick LIDAR Matlab/C++ Toolbox
 * Copyright (c) 2008, Jason C. Derenick and Thomas H. Miller
 * All rights reserved.
 *
 * This software is released under a BSD Open-Source License.
 * See http://sicktoolbox.sourceforge.net
 */

#include <string>
#include <iostream>
#include <fstream>
#include <sicklms2xx/SickLMS2xx.hh>
#include "cv.h"
#include "highgui.h"
#include <time.h>
#include <stdio.h>

using namespace std;
using namespace SickToolbox;
using namespace cv;

int main(int argc, char* argv[])
{
//variables to timestamp the scans
	clock_t start=clock();
	clock_t cur_time=clock();
  double timestamp=0;
  
	//variable to describe the angle between scan values
	double angle_step=0.5;
  
  //calibration values for the offset of the lidar to the robot origin
  double angle_offset=0;
  Point3f translation=Point3f(0,0,0);
  
  Rect_<double> danger_box=Rect_<double>(-.5,0,.5,.5);
  
  
  string device_str;                      
  SickLMS2xx::sick_lms_2xx_baud_t desired_baud = SickLMS2xx::SICK_BAUD_38400;

  unsigned int values[SickLMS2xx::SICK_MAX_NUM_MEASUREMENTS] = {0}; // Uses macro defined in SickLMS2xx.hh
  unsigned int num_values = 0;                                   // Holds the number of measurements returned

  /* Check for a device path.  If it's not present, print a usage statement. */
  if ((argc != 2 && argc != 3) || (argc == 2 && strcasecmp(argv[1],"--help") == 0)) {
    cout << "Usage: lms2xx_simple_app PATH [BAUD RATE]" << endl
	 << "Ex: lms2xx_simple_app /dev/ttyUSB0 9600" << endl;
    return -1;
  }

  /* Only device path is given */
  if (argc == 2) {
    device_str = argv[1];
  }

  /* Device path and baud are given */
  if (argc == 3) {    
    device_str = argv[1];
    if ((desired_baud = SickLMS2xx::StringToSickBaud(argv[2])) == SickLMS2xx::SICK_BAUD_UNKNOWN) {
      cerr << "Invalid baud value! Valid values are: 9600, 19200, 38400, and 500000" << endl;
      return -1;
    }
  }

  /*
   * Instantiate an instance
   */
  SickLMS2xx sick_lms_2xx(device_str);

  /*
   * Initialize the Sick LMS 2xx
   */
  try {
    sick_lms_2xx.Initialize(desired_baud);
  }

  catch(...) {
    cerr << "Initialize failed! Are you using the correct device path?" << endl;
    return -1;
  }
	// Create a window to catch keys
  cvNamedWindow( "keycatcher", CV_WINDOW_AUTOSIZE );
  
  /*
   * Acquire a few scans from the Sick LMS
   */
  try {
  
	  while(1) {
      cur_time=clock();
      sick_lms_2xx.GetSickScan(values,num_values);
     	timestamp= ((double)(cur_time-start))/CLOCKS_PER_SEC;
      cout << "\t  Num. Values: " << num_values << endl;
      
      
      /* create matrixies */
      Mat mag=Mat(num_values,1,CV_64F); 
      Mat ang=Mat(num_values,1,CV_64F); 
      
      for(unsigned int j = 0; j < num_values; j++) {
      	ang.at<double>(j,0)=j*angle_step-angle_offset;
      	mag.at<double>(j,0)=values[j]/100.0;
      }

			Mat x=Mat(num_values,1,CV_64F); 
      Mat y=Mat(num_values,1,CV_64F); 
      
      polarToCart(mag,ang,x,y,true);

   	  add(x,translation.x,x);
      add(y,translation.y,y);
      
      for(unsigned int j = 0; j < num_values; j++) {
      	Point2f test_point=Point2f(x.at<double>(j,0),y.at<double>(j,0));
      	
     		cout<<"point: "<<test_point.x<<" "<<test_point.y<<endl;
      	if(danger_box.contains(test_point)){
      			cout<<"danger!"<<test_point.x<<" "<<test_point.y<<" "<<j<<"\n";
      			break;
      	}
      }
	    mag.release();
      ang.release();
      x.release();
      y.release();
      
      if((cvWaitKey(1000) & 255)==27){
      	cout<<"well we tried\n";
	      break;
     	}
      
    }

  }

  /* Catch anything else and exit */ 
  catch(...) {
    cerr << "An error occurred!" << endl;
  }

  /*
   * Uninitialize the device
   */
  try {
    sick_lms_2xx.Uninitialize();
  }
  
  catch(...) {
    cerr << "Uninitialize failed!" << endl;
    return -1;
  }
  
  
  cvDestroyWindow( "keycatcher" );
  /* Success! */
  return 0;

}
    
