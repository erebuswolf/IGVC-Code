/************************************
*          Aaron Deal               *
*    Test prgram for line class     *
************************************/

#include "Line.h"
#include "time.h"
#include <iostream>
#include <string>

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
	
	IplImage* test = cvCreateImage( cvSize(1000,1000), IPL_DEPTH_8U, 1);
	cvSet(test, cvScalar(255));
	cvSet2D(test, 500, 500, cvScalar(0));
	IplImage* dist = cvCreateImage( cvSize(1000,1000), IPL_DEPTH_32F, 1);
	cvDistTransform( test, dist, CV_DIST_L2);
	cvConvertScale(dist, dist, 1/685.);
	CvScalar s = cvGet2D(dist, 0, 0);
	cout<<s.val[0]<<endl;
	cvShowImage("distance", dist);
	cvWaitKey(0);

	
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
	else if(name.compare("test18hardcore.PNG")==0)
	{
		initial.heading = 0;
		initial.center = cvPoint(715, 425);
	}
	else if(name.compare("test19.PNG")==0)
	{
		initial.heading = -45;
		initial.center = cvPoint(700, 600);
	}
	//cvCircle(image,initial.center, 5, cvScalar(100), 3);
	cvShowImage("Orig",image);
	cvWaitKey(0);
	
	
	Line line;
	clock_t begin = clock();
	//for(int i=0; i<50; i++)
		line.main(image, final, initial.center, initial.heading);
	clock_t end = clock();
	cout<<"****Done*****"<<endl;
	//cout << "Time elapsed: " << double(diffclock(end,begin)) << " ms"<< endl;
	cvShowImage("Lines", final);
	cvWaitKey(0);
}
