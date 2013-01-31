/*************************************
*              Line.h                *
*        Author: Aaron Deal          *
*      Class for classifying lines   *
*************************************/

#ifndef _Line_h
#define _Line_h

#include "WorldModel.h"
#include "highgui.h"
#include "cv.h"
#include "math.h"
#include <vector>

using namespace std;

#define LEFT 0
#define RIGHT 1

#define PI 3.14159265358979323846264338327950288419716

typedef struct{
	//used for gradient descent and robust search
	int index;
	int sum;
	//heading of tire
	int heading;
	//state = 1 -- on a line
	//state = 0 -- not on a line
	int state;
	//last known point where the wheel was on a line
	CvPoint last;
	//center of the tire
	CvPoint center;
	//current search width -- maxes out at half the track width
	int search_width;
} Tire;

typedef struct{
	//state = 0 -- stepping left first
	//state = 1 -- stepping right first
	//state = 2 -- searching on both sides
	int state;
	//diffenrence between left and right position
	int width;
	//tires on the axle
	Tire tires[2];
}Axle;

typedef struct{
	float a;
	float b;
	float c;
} Coefficients;
	

class Line
{
	public:
		//constructor and deconstructor
		Line();
		~Line();
		
		//initializes the line class -- reads in config file
		void init(string config_path);
		
		vector <CvPoint> main(IplImage *image, IplImage *map, CvPoint start, int heading);
		
		//takes in a 0-255 image of suggested lines and outputs a 0 and 255 map of lines
		vector <CvPoint> makeLineMap(IplImage *image, IplImage *map, CvPoint start, int heading);
		vector <CvPoint> makeLineMap(IplImage *image, IplImage *map, Axle axle);
		
		//find lines in an image
		void findLines(IplImage *image, CvPoint robot, Tire *tires);
		
		//finds the position of the left and right wheel on an unknow map
		void startUp(IplImage *image, CvPoint start, int heading, Tire *tires);
		
		//sets the initial heading of the robot
		void setInitialHeading(int heading);
		
		//returns midpoints to be followed by planner
		vector<CvPoint2D32f> qualsLineFinder(IplImage *image, CvPoint robot);
		
		//startup for qualification run
		vector<CvPoint> qualsStartUp(IplImage *image, CvPoint robot);
		
		//translates the region of interest and finds the best fit
		void translation(IplImage* image, Tire *tire);
		
		//rotates the mask to find the best fit
		void rotation(IplImage *image, Tire *tire);
		
		//gradient descent that finds the best fit for a line
		void gradientDescent(IplImage* image, Tire *tire);
		
		//robust search to find the line
		void robustSearch(IplImage *image, Tire *tire, int threshold);
		
		//and's the current mask and returns the sum of the output
		int sum(IplImage* image, IplImage* mask);
		
		Coefficients genLine(float x1, float y1, float x2, float y2);
		
		//finds the angle between the normal and the unit vector between the points
		double dotproduct(Tire *tire, int side);
		
		//world model which holds previous line masks
		WorldModel *model;
		
	private:
		//width of the square mask
		int mask_width;
		
		//width of the line on mask
		int line_width;
		
		//two inverse masks
		IplImage *line_mask, *no_line_mask, *output, *vMap, *final;
		
		//config variables
		int THRESHOLD;
		int TRACKWIDTH;
		int littleStep;
		int bigStep;
		double percent_step;
		int map_line_width;
		
		//do we have a start point
		bool start_point;
		int threshold;
		
		//current start point for wheels and heading
		CvPoint start[2];
		CvPoint2D32f init_location[2];
		int initial_heading;
		int line_heading;
		double axle_width;
		
		//stores previous points for left and right wheel
		CvPoint previous[2][5];
};

#endif
