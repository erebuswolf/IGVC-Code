
/**
*	WorldModel.h
*	Author: Jesse Fish
*
* gets lidar data, and position informatoin
* holds all position information
* if we have a preset map uses it and recalcs position
* else just use gps and update the map with lidar and vision for line finding
*/

#ifndef _World_Model_h
#define _World_Model_h

#include "cv.h"
#include "highgui.h"
#include "PositionState.h"
#include "LidarManager.h"
#include "VisionInterpreter.h"
#include "NetworkStack.h"
#include "LidarLocalizer.h"
#include <string>
#include <deque>
#include <fstream>

using namespace std;
using namespace cv;

class WorldModel{
  public:
	WorldModel();
	~WorldModel();
	bool useVision;
	bool useLidar;
	bool usePresetMap;
	bool usePresetMapPSO_ONLY;
	
	bool inDanger;
	//bool collision;//who made this variable??
	
	PositionState state;
	
	IplImage* scanMask;
	IplImage* wallMask;
	IplImage* frontierMask;
	IplImage* gradientAdd;
	IplImage* planner_world;
	IplImage* costMap;
	IplImage* sonarAdd;
	IplImage* sonarSub;
	IplImage* sonarMap;
	IplImage* completePlanWorldMap;
	
	Network* network;
	
	void init(string);
	int updateModel();
	void updateSonar();
	void informHardware();
	
	IplImage* worldModel;
	
	IplImage* worldModelLidar;
	IplImage* worldModelVision;
	IplImage* worldModelVisionAdd;
	IplImage* worldModelVisionSub;
	IplImage* worldModelBuffer;
	IplImage* worldModelCombined;
	IplImage* frontierPixels;
	IplImage* whereIHaveBeen;
	
	int neutral_knowledge;
	
	LidarManager* lidar;
	VisionInterpreter* vision;
	
	LidarLocalizer* lidarLocalizer;
	
	CvPoint2D32f pixeltometer(int x, int y);
	
	float meterToPixel;
	CvPoint2D32f translation;
	
	CvRect getRegionCentered(int height, int width); 
	CvRect correctROI(CvRect roi);
	
	//threadsafe way to get the translation values
	CvPoint2D32f getTranslation();
	
	//threadsafe way to get the state of the robot
	PositionState getstate();
	
	void copyCompletePlanWorldmap(IplImage * image);
	void copyFrontiermap(IplImage * image);
	
	void buildCompletePlanWorldmap();
	int last_danger;
	
  private:
	void findPositionFromMap();
	void fixROI(IplImage* image, IplImage* buffer,CvRect roiImage,CvRect roiImageBuffer,int value);
	//void updateModelLidar();
	void updateModelLidarWithDialation();
	
	void updateModelVision();
	
	//constant for how much to clear from seeing empty space
	float dintensityClear;
	//constant for how much to darken from seeing a wall
	float dintensityWall;
	//depth in meters that walls get darkened in
	float walldepth;
	
	float lidarThreshold;
	
	float objectDialation;
	
	pthread_mutex_t copyfrontiermapMut;
	pthread_mutex_t copyCompletePlanWorldmapMut;
	pthread_mutex_t poseMut;
	pthread_mutex_t TranlsationMut;
	
	
	bool first_shift;
	
	//pixels to move the map if we walk off of it
	int mapMove;
	
	//maximum change in radius that an object can be linked across
	float maxDeltaWall;
	
	//points for visualizing data
	CvPoint curve1[SickLMS2xx::SICK_MAX_NUM_MEASUREMENTS];
	CvPoint curve2[SickLMS2xx::SICK_MAX_NUM_MEASUREMENTS*2];
	
	//method to set up the gradient image
	void setupGradient(int, float);
	
	/* axd - post processing variables*/
	bool post_process;
	//file used to for post processing
	ifstream logfile;
};

#endif

