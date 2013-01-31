/************************************
*          Aaron Deal               *
*    Test prgram for line class     *
************************************/

#include "highgui.h"
#include "cv.h"
#include "math.h"
#include <iostream>

using namespace cv;
using namespace std;

#define PI 3.14159265358979323846264338327950288419716

int main(int argc, char *argv[]){
	
	IplImage *image = cvLoadImage("worldmodelvision.jpeg", 0);
	IplImage *copy  = cvCreateImage( cvGetSize(image), 8, 3);
	IplImage *display = cvCreateImage( cvGetSize(image), 8, 3);
	cvCvtColor(image,display,CV_GRAY2RGB);
	
	CvPoint robot = cvPoint(207,490);
	cvCircle(display, robot, 4, cvScalar(0, 0, 255), 2);
	int meterToPixel = 20;
	double track_width  = 3.3;
	double mask_size    = 5*meterToPixel;
	CvPoint rightstart, leftstart, initial, start, end;
	//create the mask
	IplImage *output = cvCreateImage( cvSize(mask_size, mask_size), IPL_DEPTH_8U, 1);
	IplImage *mask   = cvCreateImage( cvSize(mask_size, mask_size), IPL_DEPTH_8U, 1);
	for(double track = 3; track <=4; track += .1){
		for(int heading = 0; heading <= 180; heading+=5){
			//set the mask to zero
			cvSetZero(mask);
			
			//draw lines that are a track width apart
			leftstart  = cvPoint( mask_size/2. - (track/2)*meterToPixel*sin((heading-90)*PI/180.),
								  mask_size/2. - (track/2)*meterToPixel*cos((heading-90)*PI/180.) );
										  
			rightstart  = cvPoint( mask_size/2. + (track/2)*meterToPixel*sin((heading-90)*PI/180.),
								   mask_size/2. + (track/2)*meterToPixel*cos((heading-90)*PI/180.) );
			
			//draw the left line
			start = cvPoint( leftstart.x - mask_size*sin(heading*PI/180.),
							 leftstart.y - mask_size*cos(heading*PI/180.) );
			end   = cvPoint( leftstart.x + mask_size*sin(heading*PI/180.),
							 leftstart.y + mask_size*cos(heading*PI/180.) );	 
			cvLine( mask, start, end, cvScalar(255), 5);
	
			//draw the right line
			start = cvPoint( rightstart.x - mask_size*sin(heading*PI/180.),
							 rightstart.y - mask_size*cos(heading*PI/180.) );
			end   = cvPoint( rightstart.x + mask_size*sin(heading*PI/180.),
							 rightstart.y + mask_size*cos(heading*PI/180.) );
			cvLine( mask, start, end, cvScalar(255), 5);
	
			//cvShowImage("Mask", mask);
			//cvWaitKey(4);
			
			//set the region of interest on the image
			CvRect roi;
			
			for(int i = -4; i<=4; i++){
				//location of the upper left corner of the region of interest
				roi.x = robot.x - (mask_size/2.) + i*(.5*meterToPixel)*sin((heading-90)*PI/180.);
				roi.y = robot.y - (mask_size/2.) + i*(.5*meterToPixel)*cos((heading-90)*PI/180.);
				roi.width = roi.height = mask_size;
				
				//set the region of interest
				cvSetImageROI(image, roi);
				
				//and the images together
				cvAnd(image, mask, output);
	
				cvResetImageROI(image);
				//sum the output image
				CvScalar total = cvSum(output);
				
				if( total.val[0] > 90000 ){
					cout<<"Found Possible line!\n";
					cout<<"SUM: "<<total.val[0]<<endl;
					cvCopy(display, copy);
					
					initial.x = leftstart.x - (mask_size/2.) + i*(.5*meterToPixel)*sin((heading-90)*PI/180.) + robot.x;
					initial.y = leftstart.y - (mask_size/2.) + i*(.5*meterToPixel)*cos((heading-90)*PI/180.) + robot.y;
					
					start.x = initial.x + 50.*sin(heading*PI/180.);
					start.y = initial.y + 50.*cos(heading*PI/180.);
					end.x = initial.x - 50.*sin(heading*PI/180.);
					end.y = initial.y - 50.*cos(heading*PI/180.);
					
					cvLine(copy, start, end, cvScalar(255, 0, 0), 2);
					cvCircle(copy, initial, 2, cvScalar(0, 255, 0), 2);
					
					initial.x = rightstart.x - (mask_size/2.) + i*(.5*meterToPixel)*sin((heading-90)*PI/180.) + robot.x;
					initial.y = rightstart.y - (mask_size/2.) + i*(.5*meterToPixel)*cos((heading-90)*PI/180.) + robot.y;
					start.x = initial.x + 50.*sin(heading*PI/180.);
					start.y = initial.y + 50.*cos(heading*PI/180.);
					end.x = initial.x -50.*sin(heading*PI/180.);
					end.y = initial.y - 50.*cos(heading*PI/180.);
					
					cvLine(copy, start, end, cvScalar(255, 0, 0), 2);
					cvCircle(copy, initial, 2, cvScalar(0, 255, 0), 2);
					
					cvShowImage("match", copy);
					cvWaitKey(0);
				}
			}
		}
	}
}

