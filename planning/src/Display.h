/**
*	Display.h
*	Author: Jesse Fish
*
*/


#ifndef _Display_h
#define _Display_h
#include "WorldModel.h"
#include "Planner.h"
#include <string.h>

struct WindowData{
	int xmin;
	int ymin;
	int width;
	int height;
};

class Display{
	public:
		Display(WorldModel*);
		~Display();
		WorldModel* model;
		Planner* planner;
		IplImage* displayImage;	
		IplImage* displayImage2;
	
		//this variable should have an accessor, but im not going to do it right now
		string windowname;
		string windowname2;
		string visionwindow;
		void displayModel();
		
	private:

	protected:
		WindowData windata;
};

#endif
