/*************************************
*              Line.h                *
*        Author: Aaron Deal          *
*      Class for classifying lines   *
*************************************/

#ifndef _Line_h
#define _Line_h

#include "highgui.h"
#include "cv.h"
#include "math.h"

#define ROTATION    1
#define TRANSLATION 0

#define LEFT 0
#define RIGHT 1

#define PI 3.14159265358979323846264338327950288419716

#define littleStep 10
#define bigStep 15
#define TRACKWIDTH 250
#define THRESHOLD 40266

typedef struct{
	int index;
	int sum;
	int angle;
	CvPoint start;
} Gradient;

typedef struct{
	//state = 1 -- on a line
	//state = 0 -- not on a line
	int state;
	//last known point where the wheel was on a line
	CvPoint last;
	//current search width -- maxes out at half the track width
	int searchWidth;
} Status;

typedef struct{
	//state = 0 -- stepping left first
	//state = 1 -- stepping right first
	//state = 2 -- searching on both sides
	int state;
	//diffenrence between left and right position
	int width;
	//status of each wheel
	Status lines[2];
	Status left;
	Status right;
} Wheel;

class Line
{
	public:
		Line();
		~Line();
	
		//make a line map
		static void makeLineMap(IplImage* image, IplImage* final, Gradient initial);
		
		//line functions
		static int sum(IplImage* image, IplImage* mask);
		
		//rotate wheel base about origin
		static CvPoint rotate(CvPoint origin, CvPoint point, int angle);
		
	private:
		IplImage *line_mask, *no_line_mask;
		
		static void startUp(IplImage* image, IplImage* mask, Gradient initial, Gradient* lines);
		
		static Gradient gradientDescent(IplImage* image, IplImage* mask, Gradient line);
		
		//finds maximum number of pixels
		static Gradient findGradient(IplImage *image, IplImage*mask, int angle, CvPoint start, int type, int total=0);
		
		//finds the maximum neighbor
		static Gradient findNeighbors(IplImage *image, IplImage *mask, int angle, CvPoint start, int type);

		//does a linear robost search - Translation only
		static Gradient robustSearch(IplImage *image, IplImage *mask, int angle, CvPoint start, int 	searchWidth, int threshold = THRESHOLD);
};

#endif
