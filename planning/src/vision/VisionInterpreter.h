/**
*	VisionInterpreter.h
*	Author: Jesse Fish
*
* 
* holds calibrations
* used to find lines and obsticals, using mono and stereo cameras
*
*/

#ifndef _Vision_Interpreter_h
#define _Vision_Interpreter_h

#include <string>
#include <vector>
#include "cv.h"
#include "highgui.h"
#include "CameraManager.h"

using namespace std;
using namespace cv;


//THIS CLASS AND POSSIBLY THE ABOVE STRUCT MAY NEED TO BE REFACTORED WHEN STEREO 3D IS ADDED
class VisionInterpreter{

	public:
		CameraManager* manager;
		
		IplImage* processedImage;
		IplImage* subtractorImage;
		//function to find lines and obsticals in the images from all cameras
		void findLinesAndObsticles(double );

		VisionInterpreter(CameraManager* );
		~VisionInterpreter();
		IplImage* hsv_s;
		IplImage* hsv_v;
		IplImage* subtractorLayer;
		void init(string config_filepath);
	private:
		IplImage* hsv;
		IplImage* dewarped_image;
		
		IplImage* mapx;
		IplImage* mapy;
		
		int s_c;
		float s_k;
		
		int v_c;
		float v_k;
		float lineThreshold;
		
		float black_space;
		
		float subScale;
		
		bool debugVision;
		
		IplImage* rotationImage;
  		int windowSize;
};

#endif

