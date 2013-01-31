/**
*	PointAndClickDisplay.h
*	Author: Jesse Fish
*
*/


#ifndef _PointAndClickDisplay_h
#define _PointAndClickDisplay_h
#include "Display.h"
#include <string.h>

class PointAndClickDisplay: public Display{
	public:
		Point_<float>	goalWayPoint;
		static void mouse_handler(int , int , int , int , void *);
		PointAndClickDisplay(WorldModel*);
		~PointAndClickDisplay();
};

#endif
