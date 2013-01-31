
#include <stdio.h>
#include <iostream>
#include "cv.h"
#include "highgui.h"
#include "TimeKeeper.h"

using namespace std;
using namespace cv;

bool quit=false;
pthread_mutex_t image1_mutex= PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t image2_mutex= PTHREAD_MUTEX_INITIALIZER;

void *camera1_grabber(void* voidcap);
void *camera2_grabber(void* voidcap);

int main(int argc, char* argv[]) {
  TimeKeeper::start_time();
  
  pthread_t listenThread1;
  pthread_t listenThread2;
  
  
  CvCapture* capture1 = cvCaptureFromCAM( 0 );
  CvCapture* capture2 = cvCaptureFromCAM( 1);
  
  if( !capture1||!capture2 ) {
	fprintf( stderr, "ERROR: capture is NULL \n" );
	getchar();
	return -1;
  }
  
  cvNamedWindow("image1");
  cvNamedWindow("image2");
  
  IplImage *image1 = cvCloneImage(cvQueryFrame( capture1 ));
  IplImage *image2 = cvCloneImage(cvQueryFrame( capture2 ));
  
  IplImage *temp_bw=cvCreateImage(cvSize(image1->width,image1->height),8,1);
  
  // LOOP TO ALLOW USER TO PLAY WITH HEIGHT:
  //
  // escape key stops
  //
  
  int fps=(int)cvGetCaptureProperty(capture1,CV_CAP_PROP_FPS);
  //spawn capture thread right before loop
  int thread_id1=pthread_create(&listenThread1,NULL,&camera1_grabber,(void*)capture1);
  
  int thread_id2=pthread_create(&listenThread2,NULL,&camera2_grabber,(void*)capture2);
  int key=0;
  while((key & 255) != 27) {
	long start=TimeKeeper::GetTime();
	
	
	
	//grab a frame from the camera
	//wait for the thread to change modalities
	pthread_mutex_lock(&image1_mutex);
	pthread_mutex_lock(&image2_mutex);
	//grab the newest image from the camera
	cvCopy(cvRetrieveFrame(capture1),image1);
	cvCopy(cvRetrieveFrame(capture2),image2);
	//cout<<"got image"<<endl;
	//let the thread run again
	pthread_mutex_unlock(&image1_mutex);
	pthread_mutex_unlock(&image2_mutex);
	
	//make the camera frame have 1 color layer
	cvCvtColor(image1,temp_bw,CV_RGB2GRAY);
	//convert the camera frame to color
	cvCvtColor(temp_bw,image1,CV_BayerGB2RGB);
	
	//make the camera frame have 1 color layer
	cvCvtColor(image2,temp_bw,CV_RGB2GRAY);
	//convert the camera frame to color
	cvCvtColor(temp_bw,image2,CV_BayerGB2RGB);
	
	cvShowImage( "image1",image1 );
	cvShowImage( "image2",image2 );
	
	
	
	long dt=TimeKeeper::GetTime()-start;
	cout<<dt<<endl;
	key = cvWaitKey(2);
					  
  }
  
  quit=true;
  usleep(100);
  //just sleep dont try to join.. it will fail and segfault
  //pthread_join(thread_id,NULL);
  
  cvReleaseCapture( &capture1 );
  cvReleaseCapture( &capture2 );
  cvDestroyWindow( "image1" );
  cvDestroyWindow( "image2" );
  cvReleaseImage(&temp_bw);
  return 0;
}

void *camera1_grabber(void* voidcap){
  while(!quit){
	pthread_mutex_lock(&image1_mutex);
	CvCapture* capture=(CvCapture*)voidcap;
	cvGrabFrame(capture);
	//cout<<"queried image"<<endl;
	pthread_mutex_unlock(&image1_mutex);
	//can sleep for at least this ammount, camera has an fps of 15, so 1 frame every 66666 usecs
	usleep(16666);
  }
}

void *camera2_grabber(void* voidcap){
  while(!quit){
	pthread_mutex_lock(&image2_mutex);
	CvCapture* capture=(CvCapture*)voidcap;
	cvGrabFrame(capture);
	//cout<<"queried image"<<endl;
	pthread_mutex_unlock(&image2_mutex);
	//can sleep for at least this ammount, camera has an fps of 15, so 1 frame every 66666 usecs
	usleep(16666);
  }
}

