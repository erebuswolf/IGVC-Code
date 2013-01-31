
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
	
	void calibrateIntrinsics(int);
	void calibrateExtrinsics(int);

	int loadIntrinsics(int,String);
	int loadExtrinsics(int,String);

	int saveIntrinsics(int,String);
	int saveExtrinsics(int,String);


	void startThread();
	void killThread();
	
	//static thread function
	static void *allCameraGrabber(void* voidcap);
	
	//array of cameras in the manager
	vector <Camera*> cameras;
	
  private:
	
	
	//thread variables
	pthread_t listenThread;
	int thread_id;
	bool endthread;
	unsigned int threadSleepTime;
	
};

#endif
