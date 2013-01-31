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
  double timestamp=0;
  
  //variable to describe the angle between scan values
  double angle_step=0.5;
  
  //calibration values for the offset of the lidar to the robot origin
  double angle_offset=0;
  Point3f translation=Point3f(250,250,0);
  
  IplImage* img=cvCreateImage(cvSize(500,500),IPL_DEPTH_32F,3);
  
  unsigned int num_values = 361;                                   // Holds the number of measurements returned
  
  
  unsigned int values[]={121,120,120,120,121,120,120,121,121,121,121,121,122,122,123,123,123,124,124,124,124,125, 125,125,125,126,126,126,126,126,126,126,127,127,127,127,128,128,129,129,130,130,131,131,132,133,133,133,133,134, 135,135,136,136,137,138,138,139,140,141,142,142,143,144,145,145,145,146,148,148,149,150,151,152,154,154,155,156, 157,158,159,161,161,162,164,166,167,168,170,171,173,175,176,178,179,182,183,185,187,189,191,192,195,197,200,201, 204,206,209,211,214,217,220,223,225,228,232,236,238,243,246,251,255,260,263,268,272,277,282,287,292,299,304,310, 317,324,330,337,345,354,361,371,382,392,402,414,426,439,452,467,483,499,518,535,556,576,603,628,661,693,730,771, 817,867,922,986,1060,1147,1251,1380,1446,1675,8191,8191,2867,2905,2907,2914,2915,2916,2912,2905,2904,2378,1908, 1658,1417,1290,1119,1049,957,880,812,756,706,663,614,609,609,534,512,486,466,449,426,409,391,375,364,353,342,332, 323,315,306,297,291,283,277,270,264,258,253,248,242,238,236,231,227,223,220,216,212,209,205,201,198,196,192,189, 186,184,181,178,175,174,171,169,167,165,162,161,159,157,155,154,153,150,148,147,145,142,140,140,140,141,141,140, 142,144,145,145,145,145,144,143,142,141,139,138,137,136,135,134,133,133,133,131,131,130,129,128,127,127,126,125, 125,124,124,123,121,120,120,119,119,119,119,118,118,117,117,116,116,116,115,115,114,114,113,112,112,112,111,111, 110,110,110,109,109,109,109,109,109,109,109,109,108, 108,108,108,108,107,107,106,107,107,107,107,108,107,105,105,105,105,104,104,104,103,103,103,103,};
  // Create a window to catch keys
  cvNamedWindow( "keycatcher", CV_WINDOW_AUTOSIZE );
  
  /*
  * Acquire a few scans from the Sick LMS
  */
  try {
	
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
	
	float scaler=3;
	CvMat oldx = x;
	CvMat oldy = y;
	cvScale(&oldx,&oldx,scaler);
	cvScale(&oldy,&oldy,-scaler);
	
	add(x,translation.x,x);
	add(y,translation.y,y);
	
	CvPoint curve1[362];
	for(int i=0;i<361;i++){
	  curve1[i].x=(int)(x.at<double>(i,0));
	  curve1[i].y=(int)(y.at<double>(i,0));
	}
	curve1[361].x=250;
	curve1[361].y=250;
	
	CvPoint* curveArr[1]={curve1};
	int      nCurvePts[1]={362};
	int      nCurves=1;
	cvFillPoly(img,curveArr,nCurvePts,nCurves,cvScalar(255,255,255));
	
	
	cvShowImage("keycatcher", img); 
	cvWaitKey(0);
	
	mag.release();
	ang.release();
	x.release();
	y.release();
	
	
  }
  
  /* Catch anything else and exit */ 
  catch(...) {
	cerr << "An error occurred!" << endl;
  }
  
  cvReleaseImage(&img);
  cvDestroyWindow( "keycatcher" );
  
  
  /* Success! */
  return 0;
  
}

