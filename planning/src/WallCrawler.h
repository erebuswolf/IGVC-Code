/**
*	WallCrawler.h
*	Author: Jesse Fish
*
* wall crawling planner
*/


#ifndef _WallCrawler_h
#define _WallCrawler_h

#include <string>
#include "Planner.h"
#include "WorldModel.h"


using namespace std;

enum CrawlState{FRONT_BLOCKED, ON_WALL, CORNER};

class WallCrawler: public Planner{
	public:
		WallCrawler(double, double, WorldModel*,Network*);
		~WallCrawler();
		bool updatePlanner();
	protected:
		CrawlState crawlState;
		void updateState();
		void updateCrawl();
		
		double robot_length;
		int maskWidth;
		int maskHeight;
		IplImage* mathMask;
	private:
		IplImage* modeltouse;
		void buildMasks();
		float findDistToWall();
		//ideal distance we want the left wall
		double left_wall_dist;
		
		//how far left to aim when cornering
		double cornering_Const;
		
		double left_mask_length;
		
		double carrot_length;
		
		Point_<double> nextGoal;
		
		Point_<double> leftMaskTranslation;
		Point_<double> forwardMaskTranslation;
		Point_<double> leftforwardMaskTranslation;
		CvPoint center;
		
		IplImage* leftMask;
		IplImage* leftCenterMask;
		IplImage* leftCloseMask;
		IplImage* forwardMask;
		IplImage* leftForwardMask;
		
		//this is a debugging image
		//IplImage* allMasks;
		
};

#endif

