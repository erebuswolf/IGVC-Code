//Call:
//  birds-eye board_w board_h instrinics distortion image_file
// ADJUST VIEW HEIGHT using keys 'u' up, 'd' down. ESC to quit.
//
#include <stdio.h>
#include <iostream>
#include "cv.h"
#include "highgui.h"
#include "TimeKeeper.h"

using namespace std;
using namespace cv;

bool quit=false;
pthread_mutex_t image_mutex= PTHREAD_MUTEX_INITIALIZER;

void *camera_grabber(void* voidcap);

int main(int argc, char* argv[]) {

	// LET THE USER ADJUST THE Z HEIGHT OF THE VIEW
	//
	TimeKeeper::start_time();

	pthread_t listenThread;

	CvCapture* capture = cvCaptureFromCAM( CV_CAP_ANY );

  if( !capture ) {
    fprintf( stderr, "ERROR: capture is NULL \n" );
    getchar();
    return -1;
  }


	float Z = 25;
	int key = 0;	
	
	CvMemStorage* storage = cvCreateMemStorage(0);
	CvSeq* lines = 0;
	

	cvNamedWindow("canny");

	IplImage *image = cvCloneImage(cvQueryFrame( capture ));
	IplImage *temp_bw=cvCreateImage(cvSize(image->width,image->height),8,1);

	IplImage *color_canny=cvCloneImage(image);

	IplImage *edges=cvCloneImage(temp_bw);
	
	// LOOP TO ALLOW USER TO PLAY WITH HEIGHT:
	//
	// escape key stops
	//

	int fps=(int)cvGetCaptureProperty(capture,CV_CAP_PROP_FPS);
	//spawn capture thread right before loop
	int thread_id=pthread_create(&listenThread,NULL,&camera_grabber,(void*)capture);

	while(key != 27) {
		long start=TimeKeeper::GetTime();



		//grab a frame from the camera
		//wait for the thread to change modalities
		pthread_mutex_lock(&image_mutex);
		//grab the newest image from the camera
		cvCopy(cvRetrieveFrame(capture),image);
		//cout<<image->height<<" "<<image->width<<endl;
		//let the thread run again
		pthread_mutex_unlock(&image_mutex);

		//convert the converted image back to bw
		cvCvtColor(image,temp_bw,CV_RGB2GRAY);

		cvCanny(temp_bw,edges,30,70,3);

		cvCvtColor(edges,color_canny,CV_GRAY2RGB);
/*
		lines = cvHoughLines2( edges, storage, CV_HOUGH_STANDARD, 1, CV_PI/180, 100, 0, 0 );

		
		
		for( int i = 0; i < lines->total; i++ )
		{
				float* line = (float*)cvGetSeqElem(lines,i);
				float rho = line[0];
				float theta = line[1];
				CvPoint pt1, pt2;
				double a = cos(theta), b = sin(theta);
				if( fabs(a) < 0.001 )
				{
				    pt1.x = pt2.x = cvRound(rho);
				    pt1.y = 0;
				    pt2.y = color_canny->height;
				}
				else if( fabs(b) < 0.001 )
				{
				    pt1.y = pt2.y = cvRound(rho);
				    pt1.x = 0;
				    pt2.x = color_canny->width;
				}
				else
				{
				    pt1.x = 0;
				    pt1.y = cvRound(rho/b);
				    pt2.x = cvRound(rho/a);
				    pt2.y = 0;
				}
				cvLine( color_canny, pt1, pt2, CV_RGB(255,0,0), 3, 8 );
		}

	*/
		cvShowImage( "canny",color_canny );
		


		long dt=TimeKeeper::GetTime()-start;
		cout<<dt<<endl;
	  key = cvWaitKey(2);
	  if(key == 'u') Z += 0.5;
	  if(key == 'd') Z -= 0.5;

  }

	quit=true;
	usleep(100);
//just sleep dont try to join.. it will fail and segfault
	//pthread_join(thread_id,NULL);
	cvReleaseImage(&edges);
	cvReleaseImage(&color_canny);
  cvReleaseImage(&temp_bw);
	cvDestroyWindow("canny");
  return 0;
}

void *camera_grabber(void* voidcap){
	while(!quit){
		pthread_mutex_lock(&image_mutex);
		CvCapture* capture=(CvCapture*)voidcap;
		cvGrabFrame(capture);
		//cout<<"queried image"<<endl;
		pthread_mutex_unlock(&image_mutex);
//can sleep for at least this ammount, camera has an fps of 15, so 1 frame every 66666 usecs
		usleep(36666);
	}
}
