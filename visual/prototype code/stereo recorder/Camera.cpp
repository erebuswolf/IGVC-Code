/**
*	Camera.cpp
*	Author: Jesse Fish
*
*	wrapper class for threaded camera approach
*
*/


#include "Camera.h"
#include <stdio.h>
using namespace std;
using namespace cv;

Camera::Camera(){

}

Camera::~Camera(){
  killThread();
  //release capture device
  cvReleaseCapture( &capture );
  //release images
  cvReleaseImage(&image);
  cvReleaseImage(&tempBw);
}

void Camera::init(int cam){
  endthread=true;
  imageMutex=PTHREAD_MUTEX_INITIALIZER;
  capture=cvCaptureFromCAM(cam);
  //27000 micro seconds has been determined to be the optimal sleep for the cameras
  threadSleepTime=27000;
  
  if( !capture ) {
	fprintf( stderr, "ERROR: capture is NULL \n" );
  }
  else{
	image = cvCloneImage(cvQueryFrame( capture ));
	tempBw=cvCreateImage(cvSize(image->width,image->height),8,1);
  }
}

int Camera::getCamProp(int prop){
	return (int)cvGetCaptureProperty(capture, prop);
}
void Camera::updateImage(){
  pthread_mutex_lock(&imageMutex);
  cvCopy(cvRetrieveFrame(capture),image);
  pthread_mutex_unlock(&imageMutex);
}

void Camera::colorizeImage(){
  //make the camera frame have 1 color layer
  cvCvtColor(image,tempBw,CV_RGB2GRAY);
  //convert the camera frame to color
  cvCvtColor(tempBw,image,CV_BayerGB2RGB);
}

void Camera::startThread(){
  if(endthread){
	endthread=false;
	thread_id=pthread_create(&listenThread,NULL,&cameraGrabber,(void*)this);
  }
  else{
	printf("Error: camera capture thread has already been started\n");
  }
}

void Camera::killThread(){
  if(!endthread){
	endthread=true;
	pthread_join(listenThread,NULL);
	printf("capture thread killd\n");
  }
}

//thread function for capturing the camera
void  *Camera::cameraGrabber(void *voidcap){
  while(!((Camera*)voidcap)->endthread){
	pthread_mutex_lock(&(((Camera*)voidcap)->imageMutex));
	CvCapture* capture=((Camera*)voidcap)->capture;
	cvGrabFrame(capture);
	pthread_mutex_unlock(&(((Camera*)voidcap)->imageMutex));
	//can sleep for at least this ammount, camera has an fps of 15, so 1 frame every 66666 usecs
	usleep(((Camera*)voidcap)->threadSleepTime);
  }
  pthread_exit(NULL);
}
