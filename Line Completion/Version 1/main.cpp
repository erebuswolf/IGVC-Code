/************************************
*          Aaron Deal               *
*  IGVC - Two wheels and a spring   *
************************************/
#include "highgui.h"
#include "cv.h"
#include "math.h"
#include "Line.h"
#include "time.h"
#include <string>
#include <iostream>

#define PI 3.14159265358979323846264338327950288419716
#define THRESHOLD 40266

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
	
	Gradient initial;
	if(name.compare("test15.PNG")==0)
	{
		initial.angle = -45;
		initial.start = cvPoint(275,875);
	}
	else if(name.compare("test14.PNG")==0)
	{
		initial.angle = -45;
		initial.start = cvPoint(350, 765);
	}
	cvCircle(image, initial.start, 4, cvScalar(200), 2);
	cvShowImage("Orig",image);
	cvWaitKey(0);
	
	IplImage* final = cvCreateImage( cvGetSize(image), IPL_DEPTH_8U, 1 );
	
	clock_t begin=clock();
	//for(int i=0; i<100; i++)
		Line::makeLineMap(image, final, initial);
		
	clock_t end=clock();
	//cout << "Time elapsed: " << double(diffclock(end,begin)) << " ms"<< endl;

	/*IplImage* mask      = cvCreateImage( cvGetSize(image), IPL_DEPTH_8U, 1 );
	IplImage* visionMap     = cvCreateImage( cvGetSize(image), IPL_DEPTH_8U, 1 );
		
	cvCopy(image, visionMap);

	//display the original map
	cvNamedWindow("Orig");
	cvShowImage("Orig", image);
	cvWaitKey(0);
	cvDestroyWindow("Orig");
	cvNamedWindow("Final");

	CvPoint initialL = cvPoint(250,475);
	CvPoint initialR = cvPoint(250,475);
	int initialHeading = 0;
			
	//step size
	//int bigStep    = 10;
	//int littleStep = 6;
	
	//create and initialize the wheel base
	Wheel wheel;
	wheel.state = 0;
	wheel.width = sqrt( (initialL.x - initialR.x)*(initialL.x - initialR.x) + (initialL.y - initialR.y)*(initialL.y - initialR.y) );
	wheel.left.state = 1;
	wheel.right.state = 1;
	wheel.left.last = initialL;
	wheel.right.last = initialR;
	wheel.left.searchWidth = 10;
	wheel.right.searchWidth = 10;
	
	int angleOld = 0, angleDif = 0;
	
	Gradient resultL, resultR;
	resultL.index = resultR.index = -1;
	resultL.sum   = resultR.sum   = 0;
	resultL.angle = resultR.angle = 0;
	resultL.start = initialL;
	resultR.start = initialR;
	
	//find left line
	while(resultL.sum < THRESHOLD)
	{
		//*****step left***** //
		resultL.start.y = resultL.start.y - littleStep*cos((resultL.angle+90)*PI/180.);
		resultL.start.x = resultL.start.x - littleStep*sin((resultL.angle+90)*PI/180.);

		resultL = findGradient(image, mask, resultL.angle, resultL.start, TRANSLATION);
		resultL = findGradient(image, mask, resultL.angle, resultL.start, ROTATION, resultL.sum);
		resultL.angle = resultL.angle*0.4+initialHeading*0.6;
		
		if(resultL.start.x < initialL.x-250)
		{
			//reset the starting location
			resultL.start = initialL;
			//then take a big step in your heading
			resultL.start.y = resultL.start.y - littleStep*cos((resultL.angle)*PI/180.);
			resultL.start.x = resultL.start.x - littleStep*sin((resultL.angle)*PI/180.);
		}		
		cvCircle(visionMap, resultL.start, 2, cvScalar(75), 1);
		cvShowImage("Final", visionMap);
		cvWaitKey(10);
		
	}
	while(resultR.sum < THRESHOLD)
	{
		resultR.start.y = resultR.start.y + littleStep*cos((resultR.angle+90)*PI/180.);
		resultR.start.x = resultR.start.x + littleStep*sin((resultR.angle+90)*PI/180.);
		resultR = findGradient(image, mask, resultR.angle, resultR.start, TRANSLATION);
		resultR = findGradient(image, mask, resultR.angle, resultR.start, ROTATION, resultR.sum);
		resultR.angle = resultR.angle*0.4+initialHeading*0.6;
		if(resultR.start.x > initialR.x+250)
		{
			//reset the starting location
			resultR.start = initialR;
			//then take a big step in your heading
			resultR.start.y = resultR.start.y - littleStep*cos((resultR.angle)*PI/180.);
			resultR.start.x = resultR.start.x - littleStep*sin((resultR.angle)*PI/180.);
			initialR = resultR.start;
		}
		cvCircle(visionMap, resultR.start, 2, cvScalar(75), 1);
		cvShowImage("Final", visionMap);
		cvWaitKey(10);
	}
	//set the last point
	wheel.left.last = resultL.start;
	wheel.right.last = resultR.start;
	cvNamedWindow("Lines");
	//main algorithm that searches for the lines in the images and creates vision map
	while(initialL.x < image->width-70 && initialL.x > 50 && initialR.y > 50 && initialR.x > 50 && initialL.y > 50)
	{	
		//reset everything back to the initial values
		resultL.index = resultR.index = -1;
		resultL.sum   = resultR.sum   =  0;
		
		//set initial positions
		initialL = resultL.start;
		initialR = resultR.start;
		
		//calculate current width
		wheel.width = sqrt( (initialL.x - initialR.x)*(initialL.x - initialR.x) + (initialL.y - initialR.y)*(initialL.y - initialR.y) );
		
		switch(wheel.state)
		{
		case 0:
			cout<<"*********Stepping Left***********\n";
			//*****1. step left***** //
			resultL.start.y = resultL.start.y - bigStep*cos(resultL.angle*PI/180.);
			resultL.start.x = resultL.start.x - bigStep*sin(resultL.angle*PI/180.);
			
			//*****2. optimize left or robust search***** //
			while(resultL.index!=0)
			{
				resultL = findGradient(image, mask, resultL.angle, resultL.start, TRANSLATION, resultL.sum);
			}
			//set index to not 0 so that the loop can be enter
			resultL.index = -1;
			angleOld = resultL.angle;
		
			while(resultL.index!=0)
			{
				resultL = findGradient(image, mask, resultL.angle, resultL.start, ROTATION, resultL.sum);
			}
			if(resultL.sum < THRESHOLD)
			{
				cout<<"Crap! Where is the left line?\n";
				//set state to no line
				wheel.left.state = 0;
				//set search width to 5
				wheel.left.searchWidth = 5;
				//set last known point to initial and set start to initial
				//wheel.left.last = initialL;
				resultL.start = initialL;
				if(wheel.right.state)
					wheel.state=1;
				else
					wheel.state=2;
				break;
			}
			//draw line on final image
			cvLine(final, wheel.left.last, resultL.start, cvScalar(255), 5);
			wheel.left.last = resultL.start;
			//*****3. move right***** //
			angleDif = (angleOld - resultL.angle);
			
			//translate the point
			resultR.start.x = resultR.start.x + abs(sin(resultR.angle*PI/180.))*(resultL.start.x - initialL.x);
			resultR.start.y = resultR.start.y + abs(cos(resultR.angle*PI/180.))*(resultL.start.y - initialL.y);
			
			//rotate the the point
			//resultR.start = rotate( resultL.start, resultR.start, angleDif);
			
			//*****4. optimize right***** //
			if(wheel.right.state)
			{
				while(resultR.index!=0)
				{
					resultR = findGradient(image, mask, resultR.angle, resultR.start, TRANSLATION, resultR.sum);
				}
				//set index to not 0 so that the loop can be enter
				resultR.index = -1;
			
				while(resultR.index!=0)
				{
					resultR = findGradient(image, mask, resultR.angle, resultR.start, ROTATION, resultR.sum);
				}
				wheel.state=1;
				if(resultR.sum < THRESHOLD)
				{
					cout<<"Crap! Where is the right line?\n";
					//set state to no line
					wheel.right.state = 0;
					//set search width to 5
					wheel.right.searchWidth = 5;
					//set last known point to initial and set start to initial
					//wheel.right.last = initialR;
					resultR.start = initialR;
					wheel.state = 0;
					break;
			}
			}
			else
			{
				resultR = robustSearch(image, mask, resultR.angle, resultR.start, wheel.right.searchWidth, THRESHOLD);
				 if(resultR.index)//index == 1 therefore a line was found
				 {
				 	cout<<"Line! Found the right line.\n";
				 	//set state back to line
				 	wheel.right.state = 1;
				 	wheel.state = 1;
				 }
				 else //no line found in robust search
				 {
				 	cout<<"Still no line\n";
				 	resultR.angle = resultL.angle*0.2 + resultR.angle*0.8; 
				 	//increase search width
				 	wheel.right.searchWidth += 5;
				 	if(wheel.right.searchWidth > wheel.width/2.)
						wheel.right.searchWidth = wheel.width/2.;
				 }
			}
			cvCircle(visionMap, resultR.start, 2, cvScalar(125), 2);
			cvCircle(visionMap, resultL.start, 2, cvScalar(125), 2);
			break;
		case 1:
			cout<<"*********Stepping Right**************\n";
			//*****1. step right***** //
			resultR.start.y = resultR.start.y - bigStep*cos(resultR.angle*PI/180.);
			resultR.start.x = resultR.start.x - bigStep*sin(resultR.angle*PI/180.);
			
			//*****2.optimize right***** //
			while(resultR.index!=0)
			{
				resultR = findGradient(image, mask, resultR.angle, resultR.start, TRANSLATION, resultR.sum);
			}
			//set index to not 0 so that the loop can be enter
			resultR.index = -1;
			angleOld = resultR.angle;
		
			while(resultR.index!=0)
			{
				resultR = findGradient(image, mask, resultR.angle, resultR.start, ROTATION, resultR.sum);
			}
			if(resultR.sum < THRESHOLD)
			{
				cout<<"Crap! Where is the right line?\n";
				//set state to no line
				wheel.right.state = 0;
				//set search width to 5
				wheel.right.searchWidth = 5;
				//set last known point to initial and set start to initial
				//wheel.right.last = initialR;
				resultR.start = initialR;
				if(wheel.left.state)
					wheel.state = 0;
				else
					wheel.state=2;
				break;
			}
			//draw map on final image
			cvLine(final, wheel.right.last, resultR.start, cvScalar(255), 5);
			wheel.right.last = resultR.start;
		
			//******3. move left***** //
			angleDif = (angleOld - resultR.angle);
			
			//translate the point
			resultL.start.x = resultL.start.x + abs(sin(resultL.angle*PI/180.))*(resultR.start.x - initialR.x);
			resultL.start.y = resultL.start.y + abs(cos(resultL.angle*PI/180.))*(resultR.start.y - initialR.y);
			
			//rotate the the point
			//resultL.start = rotate( resultR.start, resultL.start, angleDif);
			
			//*****4. optimize left***** //
			if(wheel.left.state)
			{
				while(resultL.index!=0)
				{
					resultL = findGradient(image, mask, resultL.angle, resultL.start, TRANSLATION, resultL.sum);
				}
				//set index to not 0 so that the loop can be enter
				resultL.index = -1;
				angleOld = resultL.angle;
			
				while(resultL.index!=0)
				{
					resultL = findGradient(image, mask, resultL.angle, resultL.start, ROTATION, resultL.sum);
				}
				if(resultL.sum < THRESHOLD)
				{
					cout<<"Crap! Where is the left line?\n";
					//set state to no line
					wheel.left.state = 0;
					//set search width to 5
					wheel.left.searchWidth = 5;
					//set last known point to initial and set start to initial
					//wheel.left.last = initialL;
					resultL.start = initialL;
					wheel.state=1;
					break;
				}
				wheel.state=0;
			}
			else
			{
				resultL = robustSearch(image, mask, resultL.angle, resultL.start, wheel.left.searchWidth, THRESHOLD);
				 if(resultL.index)//index == 1 therefore a line was found
				 {
				 	cout<<"Line! Found the left line.\n";
				 	//set state back to line
				 	wheel.left.state = 1;
				 	wheel.state = 0;
				 }
				 else //no line found in robust search
				 {
				 	cout<<"Still can't find line\n";
				 	resultL.angle = resultR.angle*0.2 + resultL.angle*0.8;
				 	//increase search width
				 	wheel.left.searchWidth += 5;
				 	if(wheel.left.searchWidth > wheel.width/2.)
						wheel.left.searchWidth = wheel.width/2.;
				 }
			}
			//draw point on original image
			cvCircle(visionMap, resultR.start, 2, cvScalar(50), 2);
			cvCircle(visionMap, resultL.start, 2, cvScalar(50), 2);
			break;	
		case 2:
			cout<<"*********We're Lost**********\n";
			//*****1. step right and left***** //
			resultR.start.y = resultR.start.y - bigStep*cos(resultR.angle*PI/180.);
			resultR.start.x = resultR.start.x - bigStep*sin(resultR.angle*PI/180.);
			resultL.start.y = resultL.start.y - bigStep*cos(resultL.angle*PI/180.);
			resultL.start.x = resultL.start.x - bigStep*sin(resultL.angle*PI/180.);
		
			//robust search on each side
			resultL = robustSearch(image, mask, resultL.angle, resultL.start, wheel.left.searchWidth, THRESHOLD);
			if(resultL.index)//index == 1 therefore a line was found
			{
				cout<<"Line! Found the left line.\n";
				//set state back to line
				wheel.left.state = 1;
				wheel.state = 0;
			}
			else //no line found in robust search
			{
				//increase search width
				wheel.left.searchWidth += 5;
				if(wheel.left.searchWidth > wheel.width/2.)
					wheel.left.searchWidth = wheel.width/2.;
			}
			resultR = robustSearch(image, mask, resultR.angle, resultR.start, wheel.right.searchWidth, THRESHOLD);
			if(resultR.index)//index == 1 therefore a line was found
			{
				cout<<"Line! Found the right line.\n";
				//set state back to line
				wheel.right.state = 1;
				wheel.state = 1;
			}
			else //no line found in robust search
			{
				//increase search width
				wheel.right.searchWidth += 5;
				if(wheel.right.searchWidth > wheel.width/2.)
					wheel.right.searchWidth = wheel.width/2.;
			}
			//draw point on original image
			cvCircle(visionMap, resultR.start, 3, cvScalar(200), 2);
			cvCircle(visionMap, resultL.start, 3, cvScalar(200), 2);
		}

		//spit out current status
		cout<<"Left: "<<resultL.start.x<<" "<<resultL.start.y<<" "<<resultL.angle<<" "<<resultL.sum<<"\n";
		cout<<"Right: "<<resultR.start.x<<" "<<resultR.start.y<<" "<<resultR.angle<<" "<<resultR.sum<<"\n";
		cvShowImage("Lines", final);
		cvShowImage("Final", visionMap);
		cvWaitKey(100);
	}
	cout<<"************Done*************\n";
	//create windows to display images*/
	cvShowImage("Lines", final);
	//cvShowImage("Final", visionMap);
	cvWaitKey(0);
	
	cvDestroyWindow("Lines");
	//cvDestroyWindow("Final");
}
