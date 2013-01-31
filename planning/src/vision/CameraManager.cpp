 /**
*	CameraManager.cpp
*	Author: Jesse Fish
*
*	Class to sync stereo camera captures
*
*/


#include "CameraManager.h"
#include <stdio.h>
#include <iostream>
using namespace std;
using namespace cv;

CameraManager::CameraManager(){

}

CameraManager::~CameraManager(){
  killThread();
}

void CameraManager::init(){
  endthread=true;
  //27000 micro seconds has been determined to be the optimal sleep for the cameras
  threadSleepTime=27000;
}


void CameraManager::addCam(Camera * add){
  cameras.push_back(add);
}

int CameraManager::getCamProp(int cam,int prop){
	return (int)cvGetCaptureProperty(cameras[cam]->capture, prop);
}

void CameraManager::updateCameras(){
  for(int i=0;i<cameras.size();i++){
  	if(!cameras[i]->post_process){
	  pthread_mutex_lock(&(cameras[i]->imageMutex));
	  cvCopy(cvRetrieveFrame(cameras[i]->capture),cameras[i]->image);
	  pthread_mutex_unlock(&(cameras[i]->imageMutex));
    }
  }
}

void CameraManager::updateCameras(long time_stamp){
  for(int i=0;i<cameras.size();i++){
  	cameras[i]->time_stamp = time_stamp;
  	if(cameras[i]->post_process){
      //read in image from directory
	  char name[100];
  	    sprintf(name, "%soutput%.7ld.bmp", cameras[i]->image_dir.c_str(), time_stamp);
		//cout<<"loading image "<<name<<endl;
  	    IplImage*temp=cvLoadImage(name);
  	    if(temp!=NULL){
		  cameras[i]->image = temp;
		 }
		 else{
		 	cout<<"Error failed to load iamge"<<endl;
		 	}
    }
  }
}
void CameraManager::colorizeImages(){
  for(int i=0;i<cameras.size();i++){
  	if(!cameras[i]->post_process){
	  //make the camera frame have 1 color layer
	  cvCvtColor(cameras[i]->image,cameras[i]->tempBw,CV_RGB2GRAY);
	  //convert the camera frame to color
	  cvCvtColor(cameras[i]->tempBw,cameras[i]->image,CV_BayerGB2RGB);
	}
  }
}

void CameraManager::updateCamera(int cam){
  pthread_mutex_lock(&(cameras[cam]->imageMutex));
  cvCopy(cvRetrieveFrame(cameras[cam]->capture),cameras[cam]->image);
  pthread_mutex_unlock(&(cameras[cam]->imageMutex));
}
void CameraManager::colorizeImage(int cam){
  //make the camera frame have 1 color layer
  cvCvtColor(cameras[cam]->image,cameras[cam]->tempBw,CV_RGB2GRAY);
  //convert the camera frame to color
  cvCvtColor(cameras[cam]->tempBw,cameras[cam]->image,CV_BayerGB2RGB);
}

void CameraManager::startThread(){
  if(endthread){
	endthread=false;
	thread_id=pthread_create(&listenThread,NULL,&allCameraGrabber,(void*)this);
  }
  else{
	printf("Error: camera capture thread has already been started\n");
  }
}

void CameraManager::killThread(){
  if(!endthread){
	endthread=true;
	pthread_join(listenThread,NULL);
	printf("capture thread killd\n");
  }
}

//thread function for capturing the camera
void  *CameraManager::allCameraGrabber(void *voidcap){
 // printf("Thread start\n");
  CameraManager* manager=((CameraManager*)voidcap);
  while(!manager->endthread){
	
	for(int i=0;i<manager->cameras.size();i++){
	  pthread_mutex_lock(&(manager->cameras[i]->imageMutex));
	  cvGrabFrame(manager->cameras[i]->capture);
	  pthread_mutex_unlock(&(manager->cameras[i]->imageMutex));
	}
	//can sleep for at least this ammount, camera has an fps of 15, so 1 frame every 66666 usecs
	usleep(manager->threadSleepTime);
  }
  
//  printf("Thread end\n");
  pthread_exit(NULL);
}
