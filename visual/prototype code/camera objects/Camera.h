/**
*	Camera.h
*	Author: Jesse Fish
*
*	wrapper class for threaded camera approach
*
*/


#ifndef _Camera_h
#define _Camera_h
#include "cv.h"
#include "highgui.h"
//#include "CameraManager.h"

using namespace cv;

struct CameraCalib{
  CvMat*    intrinsic;
  CvMat*    distortion;
  CvMat*    H;
};

class Camera{
  friend class CameraManager;
  public:
	//last image grabbed by updateImage(), or colorizeImage()
	IplImage *image;
	
	//camera calibration values
	CameraCalib calib;
	
	//empty constructor and destructor
	Camera();
	~Camera();
	
	void init(int cam=CV_CAP_ANY);
	void updateImage();
	void colorizeImage();
	void startThread();
	void killThread();
	int getCamProp(int);
	//static thread function
	static void *cameraGrabber(void* voidcap);
	
  private:
	//the tempBw image could be statically shared between all camera objects but to let it be threadsafe it isn't
	IplImage *tempBw;
	
	//camera capture values
	CvCapture* capture;
	
	//thread variables
	pthread_t listenThread;
	int thread_id;
	pthread_mutex_t imageMutex;
	bool endthread;
	unsigned int threadSleepTime;
};

#endif
