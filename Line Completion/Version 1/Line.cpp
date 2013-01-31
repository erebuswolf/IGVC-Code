/*******************************
*         Line.cpp             *
*     Author: Aaron Deal       *
*******************************/

#include "Line.h"
#include <iostream>

using namespace std;

Line::Line()
{
	//create the mask that is use in most functions
	line_mask    = cvCreateImage(cvSize(50, 50), IPL_DEPTH_8U, 1 );
	no_line_mask = cvCreateImage(cvSize(50, 50), IPL_DEPTH_8U, 1 );
}


CvPoint Line::rotate(CvPoint origin, CvPoint point, int angle)
{
	CvPoint result;
	
	//translate back to origin
	point.x = point.x - origin.x;
	point.y = point.y - origin.y;
	
	//rotate about origin
	result.x = ((double)point.x * cos(angle*PI/180.)) - ((double)point.y * sin(angle*PI/180.));
	result.y = ((double)point.x * sin(angle*PI/180.)) + ((double)point.y * cos(angle*PI/180.));
	
	//add back on the translation to the origin
	result.x = result.x + origin.x;
	result.y = result.y + origin.y;
	
	return result;
}

Gradient Line::findGradient(IplImage *image, IplImage*mask, int angle, CvPoint start, int type, int total)
{
	//end point of line for the mask
	CvPoint end;
	
	if( type == TRANSLATION )
	{
		//check to see if there is a sum, if not find the sum for the current position
		if(total == 0)
		{
			//find normal to angle
			int normal = angle + 90;
		
			//set mask image to zero
			cvSetZero(mask);
		
			//find new x cordinates for mask
			end.x = start.x - 50*sin(angle*PI/180.);
			end.y = start.y - 50*cos(angle*PI/180.);
		
			//draw line on mask
			cvLine(mask, start, end, cvScalar(255), 10);
		
			total = sum(image, mask);
		}
		
		Gradient neighbors = findNeighbors(image, mask, angle, start, type);
		
		Gradient result;
		if(neighbors.sum > total)
		{
			result = neighbors;
		}
		else
		{
			result.index = 0;
			result.sum   = total;
			result.angle = angle;
			result.start = start;
		}
		return result;
	}
	else
	{
		Gradient neighbors = findNeighbors(image, mask, angle, start, type);
		
		Gradient result;
		if(neighbors.sum > total)
		{
			result = neighbors;
		}
		else
		{
			result.index = 0;
			result.sum   = total;
			result.angle = angle;
			result.start = start;
		}
		return result;
	}	
}

Gradient Line::findNeighbors(IplImage *image, IplImage *mask, int angle, CvPoint start, int type)
{
	//points
	CvPoint left, right, end;
		
	if( type == TRANSLATION )
	{
		//find normal to angle
		int normal = angle + 90;
		
		//calculate left mask
		//set mask image to zero
		cvSetZero(mask);
		
		//find new x and y cordinates for mask
		left.x = start.x-2*sin(normal*PI/180.);
		left.y = start.y-2*cos(normal*PI/180.); 
		end.x  = left.x - 50*sin(angle*PI/180.);
		end.y  = left.y - 50*cos(angle*PI/180.);
		
		//draw line on mask
		cvLine(mask, end, left, cvScalar(255), 10);
		
		//calculate left sum
		int sumLeft = sum(image, mask);
		
		//calculate right mask
		//set mask image to zero
		cvSetZero(mask);
		
		//find cordinates for mask
		right.x = start.x+2*sin(normal*PI/180.);
		right.y = start.y+2*cos(normal*PI/180.); 
		end.x  = right.x - 50*sin(angle*PI/180.);
		end.y  = right.y - 50*cos(angle*PI/180.);
		
		//draw line on mask
		cvLine(mask, end, right, cvScalar(255), 10);
		
		//calculate right sum
		int sumRight = sum(image, mask);
		
		Gradient result;
		if( sumRight > sumLeft)
		{
			result.index = 1;
			result.sum   = sumRight;
			result.angle = angle;
			result.start = right;
		}
		else
		{
			result.index = -1;
			result.sum   = sumLeft;
			result.angle = angle;
			result.start = left;
		}
		return result;
	}
	else
	{
		//add 5 degrees to angle
		int Angle = angle + 5;
			
		//set mask image to zero
		cvSetZero(mask);
		
		//find new x cordinates for mask
		end.x = start.x - 50*sin(Angle*PI/180.);
		end.y = start.y - 50*cos(Angle*PI/180.);
		
		//draw line on mask
		cvLine(mask, start, end, cvScalar(255), 10);
		
		int sumPositive = sum(image, mask);
		
		//subtract 5 degrees from angle
		Angle = angle - 5;
		
		//set mask image to zero
		cvSetZero(mask);
		
		//find new x cordinates for mask
		end.x = start.x - 50*sin(Angle*PI/180.);
		end.y = start.y - 50*cos(Angle*PI/180.);
		
		//draw line on mask
		cvLine(mask, start, end, cvScalar(255), 10);
		
		int sumNegative = sum(image, mask);
		
		Gradient result;
		if( sumPositive > sumNegative )
		{
			result.index = -1;
			result.sum   = sumPositive;			
			result.angle = angle+5;
			result.start = start;
		}
		else
		{
			result.index = 1;
			result.sum   = sumNegative;
			result.angle = angle-5;
			result.start = start;
		}
		return result;
	}
}

Gradient Line::robustSearch(IplImage *image, IplImage *mask, int angle, CvPoint start, int searchWidth, int threshold)
{	
	//points
	CvPoint move, end;
	
	//find normal to angle
	int normal = angle + 90;
	
	int total;
	Gradient result;
	result.start = start;
	result.angle = angle;
	result.sum = 0;
	
	for(int i=-searchWidth; i<=searchWidth; i++)
	{
		//calculate left mask
		//set mask image to zero
		cvSetZero(mask);
	
		//find new x and y cordinates for mask
		move.x = start.x-2*i*sin(normal*PI/180.);
		move.y = start.y-2*i*cos(normal*PI/180.); 
		end.x  = move.x - 50*sin(angle*PI/180.);
		end.y  = move.y - 50*cos(angle*PI/180.);
	
		//draw line on mask
		cvLine(mask, end, move, cvScalar(255), 10);
	
		//calculate left sum
		total = sum(image, mask);
		
		if(total > threshold)
		{
			//set new threshold to beat
			threshold = total;
			//store the point
			result.start = move;
			//set index to 1 to signal that a line was found
			result.index = 1;
			//set sum
			result.sum = total;
		}
	}
	return result;
}

//and's the mask and image then returns the sum
int Line::sum(IplImage *image, IplImage *mask)
{
	//and the images together
	cvAnd(image, mask, mask);
	
	//sum the andImage
	CvScalar total = cvSum(mask);
	
	return total.val[0];
}

void Line::makeLineMap(IplImage* image, IplImage* map, Gradient robot)
{
	//used to determined the line location in the array
	int primary, secondary, color, step, normal;
	
	double dist_to_last = 0, dist[2], x, y, angle, diff, angle1;
	
	//create the mask image
	IplImage* mask = cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 1);
	IplImage* vMap = cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 1);
	cvCopy(image, vMap);
	
	//initialize a gradient for each line --- 0=left 1=right
	Gradient* lines = new Gradient[2];
	lines[0].index   = lines[1].index   = -1;
	lines[0].sum     = lines[1].sum     = 1;
	lines[0].angle   = lines[1].angle   = robot.angle;
	lines[0].start   = lines[1].start   = robot.start;
	
	//Array of initial positions
	CvPoint* initial = new CvPoint[2];
	initial[0] = initial[1] = robot.start;
	
	//find the starting position of both the left and right wheels
	startUp(image, mask, robot, lines);
	
	//initialize the wheels
	Wheel wheel;
	wheel.state = 0;  //start stepping with the left side
	wheel.lines[LEFT].state = wheel.lines[RIGHT].state = 1; //set the states of both wheels
	wheel.lines[LEFT].last  = lines[LEFT].start;
	wheel.lines[RIGHT].last = lines[RIGHT].start;
	
	//main algorithm that searches for the lines in the image and creates vision map
	while( dist_to_last < 250 )
	{
		//reset everything back to initial values
		lines[LEFT].index = lines[RIGHT].index = -1;
		lines[LEFT].sum   = lines[RIGHT].sum   =  0;
		
		//set left and right initial positions
		initial[LEFT]  = lines[LEFT].start;
		initial[RIGHT] = lines[RIGHT].start;
		
		//calculate current wheel width
		wheel.width = sqrt( (lines[LEFT].start.x - lines[RIGHT].start.x)*
							(lines[LEFT].start.x - lines[RIGHT].start.x)+ 
							(lines[LEFT].start.y - lines[RIGHT].start.y)*
							(lines[LEFT].start.y - lines[RIGHT].start.y) );
							
		//state 0 and 1 we do normal steps and for state 2 we do robust search
		if( wheel.state == 2 )
		{
			//**** 1. Step the left and right wheel ****//
			lines[RIGHT].start.y = lines[RIGHT].start.y - bigStep*cos(lines[RIGHT].angle*PI/180.);
			lines[RIGHT].start.x = lines[RIGHT].start.x - bigStep*sin(lines[RIGHT].angle*PI/180.);
			lines[LEFT].start.y = lines[LEFT].start.y - bigStep*cos(lines[LEFT].angle*PI/180.);
			lines[LEFT].start.x = lines[LEFT].start.x - bigStep*sin(lines[LEFT].angle*PI/180.);
			
			//**** 2. Robust search on the left and right side ****//
			for(int i=0; i<2; i++)
			{
				lines[i] = robustSearch(image, mask, lines[i].angle, lines[i].start, wheel.lines[i].searchWidth, THRESHOLD);
				if(lines[i].index) //index == 1 therefore a line was found
				{
					wheel.lines[i].state = 1;
					wheel.state = 0;
				}
				else
				{
					//increase the search width
					wheel.lines[i].searchWidth += 5;
					//check to make sure the search width is not to large
					if(wheel.lines[i].searchWidth > wheel.width/2.)
						wheel.lines[i].searchWidth = wheel.width/2.;
				}
			}
			//calculate the distance to last on each side and average
			dist[LEFT]  = sqrt( (lines[LEFT].start.x - wheel.lines[LEFT].last.x)*
								(lines[LEFT].start.x - wheel.lines[LEFT].last.x)+
								(lines[LEFT].start.y - wheel.lines[LEFT].last.y)*
								(lines[LEFT].start.y - wheel.lines[LEFT].last.y) );
			dist[RIGHT] = sqrt( (lines[RIGHT].start.x - wheel.lines[RIGHT].last.x)*
								(lines[RIGHT].start.x - wheel.lines[RIGHT].last.x)+
								(lines[RIGHT].start.y - wheel.lines[RIGHT].last.y)*
								(lines[RIGHT].start.y - wheel.lines[RIGHT].last.y) );
			dist_to_last = (dist[LEFT]+dist[RIGHT])/2.0;
			cout<<dist_to_last<<"\n";
			cvCircle(vMap, lines[LEFT].start, 3, cvScalar(175), 2);
			cvCircle(vMap, lines[RIGHT].start, 3, cvScalar(175), 2);
			
		}
		else
		{	
			//determine the primary and secondary wheels
			if(wheel.state == 0)
			{
				//cout<<"**STEPPING LEFT**\n";
				primary   = 0;
				secondary = 1;
				color = 75;
				normal = -90;
			}
			else
			{
				//cout<<"*STEPPING RIGHT*\n";
				primary   = 1;
				secondary = 0;
				color = 150;
				normal = -90;
			}
			
			//**** 1. Take a step with the primary wheel ****//
			lines[primary].start.y = lines[primary].start.y - bigStep*cos(lines[primary].angle*PI/180.);
			lines[primary].start.x = lines[primary].start.x - bigStep*sin(lines[primary].angle*PI/180.);
			
			//**** 2. Optimize using the gradient descent method ****//
			lines[primary] = gradientDescent(image, mask, lines[primary]);
			
			//check to make sure the threshold is met
			if(lines[primary].sum < THRESHOLD)
			{
				wheel.lines[primary].state = 0; //set state to no line
				wheel.lines[primary].searchWidth = 5;
				lines[primary].start = initial[primary];
				
				//check the state
				if(wheel.lines[secondary].state)
					wheel.state = secondary;
				else
					wheel.state = 2;
			}
			else
			{
				//**** 3. Draw line on the final image ****//
				cvLine(map, wheel.lines[primary].last, lines[primary].start, cvScalar(255), 5);
				wheel.lines[primary].last = lines[primary].start;
			
				//switch the wheel state
				wheel.state = secondary;
			
				//**** 4. Translate right along its heading ****//
				lines[secondary].start.x = lines[secondary].start.x + abs(sin(lines[secondary].angle*PI/180.))*(lines[primary].start.x - initial[primary].x);
				lines[secondary].start.y = lines[secondary].start.y + abs(cos(lines[secondary].angle*PI/180.))*(lines[primary].start.y - initial[primary].y);
			
				//** 5. Optimize the secondarying using gradient or robust depending on state **//
				if(wheel.lines[secondary].state)
				{
					//Do a gradient descent because we are on a line
					lines[secondary] = gradientDescent(image, mask, lines[secondary]);
				
					//check to make sure the threshold is metu
					if(lines[secondary].sum < THRESHOLD)
					{
						wheel.lines[secondary].state = 0; //set state to no line
						wheel.lines[secondary].searchWidth = 5;
						lines[secondary].start = initial[secondary];
						wheel.state = primary;
					}
					else
					{
						//draw in the good line
						cvLine(map, wheel.lines[secondary].last, lines[secondary].start, cvScalar(255), 5);
						wheel.lines[secondary].last = lines[secondary].start;
						
						cvCircle(vMap, lines[LEFT].start, 2, cvScalar(color), 2);
						cvCircle(vMap, lines[RIGHT].start, 2, cvScalar(color), 2);
						cvLine(vMap, lines[secondary].start, lines[primary].start, cvScalar(color), 1);
					
						//check to see if wheels are perpendicular
						x = -abs( lines[LEFT].start.x - lines[RIGHT].start.x);
						y = lines[LEFT].start.y - lines[RIGHT].start.y;
						angle = -90 - (atan(y/x)*180./PI);
						normal+=lines[primary].angle;
						diff = abs(normal - angle);
						//set step to zero
						step = 0;
						cout<<primary<<" Angle: "<<angle<<" Current: "<<normal<<" "<<x<<" "<<y<<endl;
						while(45 != angle && step < 3)
						{
							initial[secondary] = lines[secondary].start;
							//**** 1. Take a step with the primary wheel **** //
							lines[secondary].start.y = lines[secondary].start.y - bigStep*cos(lines[secondary].angle*PI/180.);
							lines[secondary].start.x = lines[secondary].start.x - bigStep*sin(lines[secondary].angle*PI/180.);
			
							//**** 2. Optimize using the gradient descent method **** //
							lines[secondary] = gradientDescent(image, mask, lines[secondary]);
			
							//check to make sure the threshold is met
							if(lines[secondary].sum < THRESHOLD)
							{
								wheel.lines[secondary].state = 0; //set state to no line
								wheel.lines[secondary].searchWidth = 5;
								lines[secondary].start = initial[secondary];
								step = 3;
							
								//check the state
								if(wheel.lines[primary].state)
									wheel.state = primary;
								else
									wheel.state = 2;
							}
							else
							{
								
								//recalculate the differences and angle
								x = -abs(lines[LEFT].start.x - lines[RIGHT].start.x);
								y = lines[LEFT].start.y - lines[RIGHT].start.y;
								angle = -90 - (atan(y/x)*180./PI);
								
								if(abs(normal-angle) < diff && abs(diff - abs(normal-angle)) > 2)
								{
									//draw in the good line
									cvLine(map, wheel.lines[secondary].last, lines[secondary].start, cvScalar(255), 5);
									wheel.lines[secondary].last = lines[secondary].start;
									
									diff = abs(normal-angle);
									step++;
									cvLine(vMap, lines[secondary].start, lines[primary].start, cvScalar(color), 1);
									cvCircle(vMap, lines[secondary].start, 2, cvScalar(color), 2);
									cvShowImage("Orig",vMap);
									cvWaitKey(0);
								}
								else
								{
									step = 3;
									lines[secondary].start = initial[secondary];
								}
							}
						}
					}
				}
				else
				{
					lines[secondary] = robustSearch(image, mask, lines[secondary].angle, lines[secondary].start, wheel.lines[secondary].searchWidth, THRESHOLD);
				
					if(lines[secondary].index) //index == 1 therefore a line was found
					{
						//set the state back to "line"
						wheel.lines[secondary].state = 1;
					}
					else
					{
						//Adjust the heading back towards the other wheel
						lines[secondary].angle = lines[primary].angle*0.2 + lines[secondary].angle*0.8;
						//increase the searchWidth
						wheel.lines[secondary].searchWidth += 5;
						wheel.state = primary;
						//check to ensure that the search width is not too large
						if(wheel.lines[secondary].searchWidth > wheel.width/2.)
							wheel.lines[secondary].searchWidth = wheel.width/2.0;
					}
					
					cvCircle(vMap, lines[LEFT].start, 2, cvScalar(color), 2);
					cvCircle(vMap, lines[RIGHT].start, 2, cvScalar(color), 2);
				}
				cvWaitKey(0);
			}
		}
		
		cout<<"START L: "<<lines[LEFT].start.x<<" "<<lines[LEFT].start.y<<" "<<lines[LEFT].angle<<endl;
		cout<<"INIT L: "<<initial[LEFT].x<<" "<<initial[LEFT].y<<endl;
		cout<<"START R: "<<lines[RIGHT].start.x<<" "<<lines[RIGHT].start.y<<" "<<lines[RIGHT].angle<<endl;
		cout<<"INIT R: "<<initial[RIGHT].x<<" "<<initial[RIGHT].y<<"\n\n";
		cvShowImage("lines", map);
		cvShowImage("Orig", vMap);
		cvWaitKey(0);
	}
}


void Line::startUp(IplImage* image, IplImage* mask, Gradient initial, Gradient* lines)
{
	/*
	TODO: Need to add initial checking to verify that there are lines to find on the map
	*/

	//define the normal angle to the heading
	int heading = initial.angle;
	int normal  = initial.angle + 90;
	//stores magnitude from initial position
	double mag;
	//Gradient to for output of descent
	Gradient result;
	
	//search until a line is found on both the left and right side
	for(int i=0; i<2; i++)
	{
		//reset the initial position
		initial.start = lines[i].start;
		
		//set the sum to zero to enter loop
		result.sum = 0;
		
		while(result.sum < THRESHOLD)
		{
			//Take a step normal to the current heading
			lines[i].start.y = lines[i].start.y - littleStep*cos(normal*PI/180.);
			lines[i].start.x = lines[i].start.x - littleStep*sin(normal*PI/180.);
			
			//optimize at the current location
			result = gradientDescent(image, mask, lines[i]);
			
			//adjust heading to keep the wheels heading close to robot heading
			lines[i].angle = result.angle*0.4 + heading*0.6;
			
			//check to see if the wheel is too far away
			mag = sqrt( (lines[i].start.x - initial.start.x)*(lines[i].start.x - initial.start.x) + (lines[i].start.y - initial.start.y)*(lines[i].start.y - initial.start.y) );
			
			if(mag > TRACKWIDTH )
			{
				//reset the starting location
				lines[i].start = initial.start;
				
				//take a big step in the direction of your heading
				lines[i].start.y = lines[i].start.y - bigStep*cos(heading*PI/180.);
				lines[i].start.x = lines[i].start.x - bigStep*sin(heading*PI/180.);
				
				//set initial as the starting location
				initial.start = lines[i].start;
			}
		}
		//store the results
		lines[i].start = result.start;
		lines[i].angle = result.angle;
		
		//set the normal to the negative
		normal -= 180;
	}
}

Gradient Line::gradientDescent(IplImage* image, IplImage* mask, Gradient line)
{
	//set the index to not 0 to enter the while loop
	line.index = -1;
	
	//set sum to zero to start off
	line.sum = 0;

	//Do a translation first
	while(line.index != 0)
	{
		line = findGradient(image, mask, line.angle, line.start, TRANSLATION, line.sum);
	}
	
	//set the index to not 0 to enter the while loop
	line.index = -1;
	
	//Now do a rotation
	while(line.index != 0)
	{
		line = findGradient(image, mask, line.angle, line.start, ROTATION, line.sum);
	}
	
	return line;
}

Line::~Line()
{

}
