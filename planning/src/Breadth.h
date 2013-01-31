#ifndef _Breadth_h
#define _Breadth_h

#include "WorldModel.h"

class Breadth{
	public:
		Breadth();
		
		void buildInitialMap();
		void buildMap();
		
		WorldModel *model;
	private:
		//map of available locations to update
		IplImage* open;
		
		int width, height;
};

#endif
