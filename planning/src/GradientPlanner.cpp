/**
*	GradientPlanner.cpp
*	Author: Jesse Fish
*
*/

#include "Planner.h"
#include <math.h>

GradientPlanner::GradientPlanner(WorldModel* arg_model){
	model = arg_model;
	gradWidth = model->worldModel->width;
	gradHeight = model->worldModel->height;
	gradientMap = cvCreateImage(cvSize(gradWidth,gradHeight),8,1);
}

void GradientPlanner::planPath(){

//ask the model to update
	model->updateModel();

//	access the world model image and plan a path

//	model->worldModel

//send path out to crio, need to know what format to plan the path in, vectors? speeds between points?
//network code here
}

void GradientPlanner::Iterate()
{
	/*
	 * for element in our image
	 * if cost!= 0 and cost!=255
	 * cost = lowest adjacent plus one
	 */
	/*
	 * Note that we don't iterate on the outermost cells in order to prevent pointer overflowing
	 */
	for(int i=1; i<(gradHeight-1); i++)
	{
		for(int j=1; j<(gradWidth-1); j++)
		{
			int step = gradientMap->widthStep/sizeof(float);
			float* data = (float *)gradientMap->imageData;
			float value = data[i*step+j];
			if( value > 0.1 && value < 100000.)
			{
				float up = data[(i+1)*step + j];
				float down = data[(i-1)*step + j];
				float left = data[i*step + j - 1];
				float right = data[i*step + j + 1];

				float ul = data[(i+1)*step + j - 1];
				float ur = data[(i+1)*step + j + 1];
				float dl = data[(i-1)*step + j - 1];
				float dr = data[(i-1)*step + j + 1];
				data[i*step+j] = min( (1. + min(up,down,left,right)), (1.41421356 + min(ul,ur,dl,dr)));
			}
		}
	}
}

void GradientPlanner::UpdateGradient()
{
	IplImage* p_world;
//	cvSetImageROI(gradientMap, model->worldModel->roi);
	/*
	 * We are going to assume that obstacles are dangerous, and everything else is fair game
	 */
	IplImage* temp_mask;

}

GradientPlanner::~GradientPlanner(){

}
