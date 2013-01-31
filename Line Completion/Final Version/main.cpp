/************************************
*          Aaron Deal               *
*    Test prgram for line class     *
************************************/

#include "Line.h"
#include "time.h"
#include <iostream>
#include <string>

using namespace cv;
using namespace std;

double diffclock(clock_t clock1,clock_t clock2)
{
	double diffticks=clock1-clock2;
	double diffms=(diffticks*1000)/CLOCKS_PER_SEC;
	return diffms;
}

int main(int argc, char *argv[]){
	//load in the test image
	string name = argv[1];
	IplImage* image     = cvLoadImage(name.c_str(), 0);
	IplImage* final		= cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 1);
	
	
	//set the starting location
	Tire initial;
	if(name.compare("test15.PNG")==0)
	{
		initial.heading = 90;
		initial.center = cvPoint(650,400);
	}
	else if(name.compare("test14.PNG")==0)
	{
		initial.heading = -45;
		initial.center = cvPoint(350, 765);
	}
	else if(name.compare("test16.PNG")==0)
	{
		initial.heading = -90;
		initial.center = cvPoint(330, 270);
	}
	else if(name.compare("test17.PNG")==0)
	{
		initial.heading = -135;
		initial.center = cvPoint(650, 300);
	}
	else if(name.compare("test18.PNG")==0)
	{
		initial.heading = 0;
		initial.center = cvPoint(715, 425);
	}
	else if(name.compare("test19.PNG")==0)
	{
		initial.heading = -45;
		initial.center = cvPoint(700, 600);
	}
	else if(name.compare("vision map.jpg")==0)
	{
		cout<<"using vision map\n";
		initial.heading = 135;
		initial.center = cvPoint(930, 1200);
	}
	cvNamedWindow("World Map Controls", CV_WINDOW_AUTOSIZE);
	
	//create the trackbars
	int windowX=0, windowY=0;
	createTrackbar("Window X", "World Map Controls", &windowX, 1000, NULL, NULL);
	createTrackbar("Window Y", "World Map Controls", &windowY, 1000, NULL, NULL);
	//createTrackbar("Window Height", "World Map Controls", &windowHw, worldMap.rows-1, NULL, NULL);
	//createTrackbar("Window Width" , "World Map Controls", &windowWw, worldMap.cols-1, NULL, NULL);
	
	//cvCircle(image,initial.center, 5, cvScalar(100), 3);
	
	int key = 0;
	while(key != 10)
	{
		cvSetImageROI(image, cvRect(windowX,windowY,1000,1000));
		cvShowImage("Orig",image);
		key = cvWaitKey(10);
		key = key&255;
	}
	cvResetImageROI(image);
	
	Line line;
	clock_t begin = clock();
	//for(int i=0; i<50; i++)
		line.main(image, final, initial.center, initial.heading);
	clock_t end = clock();
	cout<<"****Done*****"<<endl;
	//cout << "Time elapsed: " << double(diffclock(end,begin)) << " ms"<< endl;
	key = 0;
	while(key != 27)
	{
		cvSetImageROI(final, cvRect(windowX,windowY,1000,1000));
		cvShowImage("Lines", final);
		key = cvWaitKey(10);
		key = key&255;
	}
	cvResetImageROI(final);
}
