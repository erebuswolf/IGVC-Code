/**
*	GradientPlanner.h
*	Author: Jesse Fish
*
* gradient planner morgan wrote
*
*/


#ifndef _GradientPlanner_h
#define _GradientPlanner_h

#include <string>
#include "WorldModel.h"
#include "Planner.h"


using namespace std;

inline float min(float a, float b, float c, float d)
{
		return min(min(a,b), min(c,d));
}

class GradientPlanner:public Planner{
	public:
		GradientPlanner(WorldModel*);
		~GradientPlanner();
		int searchMethod;
		WorldModel* model;
		void planPath();
		void UpdateGradient();
	private:
		int gradWidth;
		int gradHeight;
		unsigned char objectThreshold;
		unsigned char safeThreshold;
		void Iterate();
		IplImage* gradientMap;
		
};

#endif

