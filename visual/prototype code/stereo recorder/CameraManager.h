
/**
*	CameraManager.h
*	Author: Jesse Fish
*
*	wrapper class for threaded camera approach
*
*/


#ifndef _CameraManager_h
#define _CameraManager_h
#include "cv.h"
#include "highgui.h"
#include <vector>
#include "Camera.h"

using namespace cv;
using namespace std;

class CameraManager{
  public:
	//empty constructor and destructor
	CameraManager();
	~CameraManager();
	void init();
	
	void addCam(Camera *);
	
	void updateCameras();
	void colorizeImages();
	
	void updateCamera(int);
	void colorizeImage(int);
	
	
	int getCamProp(int,int);
	
	void startThread();
	void killThread();
	
	//static thread function
	static void *allCameraGrabber(void* voidcap);
	
  private:
	//array of cameras in the manager
	vector <Camera*> cameras;
	
	
	//thread variables
	pthread_t listenThread;
	int thread_id;
	bool endthread;
	unsigned int threadSleepTime;
	
};

#endif
