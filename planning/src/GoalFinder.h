/**
*	goalFinder.h
*	Author: Jesse Fish
*
*/


#ifndef _Goal_Finder_h
#define _Goal_Finder_h
#include "cv.h"
#include "highgui.h"

using namespace cv;
class GoalFinder{
  public:
	GoalFinder(CvPoint imageSize, int heading_rad);
	~GoalFinder();
	IplImage* costMapDistance;
	IplImage* costMapHeading;
	IplImage* costRotMapHeading;
	
	/// returns a point that is the x y point in the image that is the best point
	CvPoint findBestPoint(IplImage* frontier_map,CvPoint robotPosition,float robotHeading,CvPoint goalPosition);

	///removes the given point from the frontier_map
	void removePoint(IplImage* frontier_map,CvPoint point);
	
	void buildDistanceMask();
	void buildHeadingMask(float heading);
	
	IplImage* costFrontiers;
  private:
};

void testGoalFinding();
#endif
