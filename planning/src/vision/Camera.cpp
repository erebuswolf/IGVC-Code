/**
*	Camera.cpp
*	Author: Jesse Fish
*
*	wrapper class for threaded camera approach
*
*/


#include "Camera.h"
#include <stdio.h>
#include <iostream>
using namespace std;
using namespace cv;

Camera::Camera(){
	post_process = false;
}

Camera::Camera(string _image_dir, long initial_time_stamp){
	post_process = true;
	image_dir  = _image_dir;
	time_stamp = initial_time_stamp;
}

Camera::~Camera(){
  //release capture device
  if(!post_process){
  	cvReleaseCapture( &capture );
  	killThread();
  }
  //release images
  cvReleaseImage(&image);
  cvReleaseImage(&tempBw);
}

void Camera::init(int cam){
	
	
	if(post_process){
	  //read in image from directory
	  char name[100];
	  sprintf(name, "%soutput%.7ld.bmp", image_dir.c_str(), time_stamp);
	  image = cvLoadImage(name);
	  cout<<"loading "<<name<<endl;
	  if(image==NULL){
	  	printf("first time stamp is wrong\n");
	  	}
	  else{
	  	tempBw=cvCreateImage(cvSize(image->width,image->height),8,1);
		}	
	}
	
	else{
		endthread=true;

		pthread_mutex_init(&(this->imageMutex),NULL);
	//27000 micro seconds has been determined to be the optimal sleep for the cameras
	threadSleepTime=27000;
	  capture=cvCaptureFromCAM(cam);
	  if( !capture ) {
		fprintf( stderr, "ERROR: capture is NULL \n" );
	  }
	  else{
	  image = cvCloneImage(cvQueryFrame( capture ));
	  tempBw=cvCreateImage(cvSize(image->width,image->height),8,1);
	  }
	}
}

int Camera::getCamProp(int prop){
	return (int)cvGetCaptureProperty(capture, prop);
}

void Camera::loadCalib(const char* intrinsic, const char *distortion, const char* H){
  calib.intrinsic = (CvMat*)cvLoad(intrinsic);
  calib.distortion = (CvMat*)cvLoad(distortion);
  calib.H = (CvMat*)cvLoad(H);
  if(calib.intrinsic==NULL || calib.distortion==NULL||calib.H==NULL){
  	printf("error bad camera calibration paths loaded\n");
  }
}
void Camera::updateImage(){
  if(post_process){
    //read in image from directory
	char name[100];
  	
  	  sprintf(name, "%soutput%.7ld.bmp", image_dir.c_str(), time_stamp);
	image = cvLoadImage(name);
  }
  else{
    pthread_mutex_lock(&imageMutex);
    cvCopy(cvRetrieveFrame(capture),image);
    pthread_mutex_unlock(&imageMutex);
  }
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
