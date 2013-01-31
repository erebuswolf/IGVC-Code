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
	output       = cvCreateImage(cvSize(50, 50), IPL_DEPTH_8U, 1 );
}

int Line::sum(IplImage* image, IplImage* mask)
{
	//and the images together
	cvAnd(image, mask, output);
	
	//sum the output image
	CvScalar total = cvSum(output);
	
	return total.val[0];
}

//generates the parameters of the line that passes through the points (x1,y1) to (x2,y2)
Coefficients Line::genLine(float x1, float y1, float x2, float y2){
	//find the a and b coeffiecents
	float a = y1-y2;
	float b = x2-x1;
	
	//normalize the coefficents
	float norm = sqrt(a*a + b*b);
	
	//normalize a and b
	a /= norm;
	b /= norm;
	
	//calculate c
	float c = -a*x2 - b*y2;
	
	//pack into LineParam and return
	Coefficients line = {a,b,c};
	
	return line;
}

double Line::dotproduct(Tire *tire, int side)
{
	//find the normal to the current stepping side
	int normal = tire[side].heading - 90;
	
	//find the unit vector of the normal
	double y_norm = -cos(normal*PI/180.);
	double x_norm = -sin(normal*PI/180.);
	
	//find the x and y difference between points
	double x_tan = tire[RIGHT].center.x - tire[LEFT].center.x;
	double y_tan = tire[RIGHT].center.y - tire[LEFT].center.y;
	
	//find magnitude of vector between points
	double mag = sqrt( x_tan*x_tan + y_tan*y_tan );
	
	//find the unit vector between the points
	x_tan /= mag;
	y_tan /= mag;
	
	return acos( (x_norm*x_tan) + (y_norm*y_tan) )*180./PI;
}

void Line::translation(IplImage* image, Tire *tire)
{
	CvPoint start, end;
	Tire initial = *tire;
	int line, no_line, total;
	
	//set tire back to 0 to say the original is best right now
	tire->index = 0;
	
	//find the starting point of the mask
	start.x = 25 - 100*sin(tire->heading*PI/180.);
	start.y = 25 - 100*cos(tire->heading*PI/180.);
	
	//find the ending point of the mask
	end.x = 25 + 100*sin(tire->heading*PI/180.);
	end.y = 25 + 100*cos(tire->heading*PI/180.);
	
	//draw line on the mask and invert for another mask
	cvSetZero(line_mask);
	cvSet(no_line_mask, cvScalar(255));
	cvLine(line_mask, start, end, cvScalar(255), 10);
	cvSub(no_line_mask, line_mask, no_line_mask);
	
	for(int i=-1; i<=1; i++)
	{
		//location of the upper left corner of the region of interest
		start.x = initial.center.x - 25 + i*3*sin((initial.heading+90)*PI/180.);
		start.y = initial.center.y - 25 + i*3*cos((initial.heading+90)*PI/180.);
		
		if(start.x < 0 || start.y < 0 || start.x+50 > 1000 || start.y+50 > 1000)
		{
			cout<<"Translatoin: "<<start.x<<"  "<<start.y<<"  "<<start.x+50<<"  "<<start.y+50<<endl;
		}
		else
		{
			//set the region of interest on the image
			cvSetImageROI(image, cvRect(start.x, start.y, 50, 50));
		
			//And and sum the two images
			line    = sum(image, line_mask);
			no_line = sum(image, no_line_mask);
			total   = line - no_line;
		
			//reset the ROI
			cvResetImageROI(image);
		
			if(total > tire->sum)
			{
				//store the center of the mask
				tire->center.x = start.x + 25;
				tire->center.y = start.y + 25;
			
				//store the sum of this mask
				tire->sum = total;
			
				//store index
				tire->index = i;
			}
		}
	}
}

void Line::rotation(IplImage *image, Tire *tire)
{
	CvPoint start, end;
	Tire initial = *tire;
	int line, no_line, total;
	
	//set tire back to 0 to say the original is best right now
	tire->index = 0;
	
	//location of the upper left corner of the region of interest
	start.x = initial.center.x - 25;
	start.y = initial.center.y - 25;
	
	if(start.x < 0 || start.y < 0 || start.x+50 > 1000 || start.y+50>1000)
	{
		cout<<"Rotation: "<<start.x<<"  "<<start.y<<"  "<<start.x+50<<"  "<<start.y+50<<endl;
	}
	else
	{
		//set the region of interest on the image
		cvSetImageROI(image, cvRect(start.x, start.y, 50, 50));
	
		for(int i=-3; i<=3; i++)
		{
			//find the starting point of the mask
			start.x = 25 - 100*sin((initial.heading-3*i)*PI/180.);
			start.y = 25 - 100*cos((initial.heading-3*i)*PI/180.);
	
			//find the ending point of the mask
			end.x = 25 + 100*sin((initial.heading-3*i)*PI/180.);
			end.y = 25 + 100*cos((initial.heading-3*i)*PI/180.);
	
			//draw line on the mask and invert for another mask
			cvSetZero(line_mask);
			cvSet(no_line_mask, cvScalar(255));
			cvLine(line_mask, start, end, cvScalar(255), 10);
			cvSub(no_line_mask, line_mask, no_line_mask);
		
			//And and sum the two images
			line    = sum(image, line_mask);
			no_line = sum(image, no_line_mask);
			total   = line - no_line;
		
			if(total > tire->sum)
			{
				//store the center of the mask
				tire->heading = initial.heading - (3*i);
			
				//store the sum of this mask
				tire->sum = total;
			
				//store index
				tire->index = i;
			}
		}
		//reset the ROI
		cvResetImageROI(image);
	}
}		
		
void Line::gradientDescent(IplImage* image, Tire *tire)
{
	CvPoint start;
	int angle;
	
	for(int i=0; i<3; i++)
	{
		//set are start point before gradient descent
		start = tire->center;
		angle = tire->heading;
		
		//set the index to -1 to enter the loop
		tire->index = -1;
	
		//set the sum to a big negative number to start off with
		tire->sum = -100000;
	
		//Do a translation first
		while(tire->index != 0)
			translation(image, tire);
	
		//set the index to not 0 to enter the while loop
		tire->index = -1;
	
		//Do a translation first
		while(tire->index != 0)
			rotation(image, tire);
		
		if(start.x == tire->center.x && start.y == tire->center.y && angle == tire->heading)
			break;
	}
}

void Line::robustSearch(IplImage *image, Tire *tire, int threshold)
{
	CvPoint start, end;
	Tire initial = *tire;
	int line, no_line, total;
	
	//set tire index to -1 to signify that no line was found -- 1 = line found
	tire->index = -1;
	
	//find the starting point of the mask
	start.x = 25 - 100*sin(tire->heading*PI/180.);
	start.y = 25 - 100*cos(tire->heading*PI/180.);
	
	//find the ending point of the mask
	end.x = 25 + 100*sin(tire->heading*PI/180.);
	end.y = 25 + 100*cos(tire->heading*PI/180.);
	
	//draw line on the mask and invert for another mask
	cvSetZero(line_mask);
	cvLine(line_mask, start, end, cvScalar(255), 10);
	
	for(int i=-tire->search_width; i<=tire->search_width; i++)
	{
		//location of the upper left corner of the region of interest
		start.x = initial.center.x - 25 + i*5*sin((initial.heading+90)*PI/180.);
		start.y = initial.center.y - 25 + i*5*cos((initial.heading+90)*PI/180.);
		
		if(start.x < 0 || start.y < 0 || start.x+50 > 1000 || start.y+50 > 1000)
		{
			cout<<"Robust Search: "<<start.x<<"  "<<start.y<<"  "<<start.x+50<<"  "<<start.y+50<<endl;
		}
		else
		{
			//set the region of interest on the image
			cvSetImageROI(image, cvRect(start.x, start.y, 50, 50));
		
			//And and sum the two images
			line    = sum(image, line_mask);
			no_line = sum(image, no_line_mask);
			total   = line - no_line;
		
			//reset the ROI
			cvResetImageROI(image);
		
			if(total > threshold)
			{
				//set the new threshold to beat
				threshold = total;
			
				//store the center of the mask
				tire->center.x = start.x + 25;
				tire->center.y = start.y + 25;
			
				//store the sum of this mask
				tire->sum = total;
			
				//store index
				tire->index = 1;
			}
		}
	}
	//perform a gradient descent if a line was found
	if(tire->index == 1)
		gradientDescent(image, tire);
}

void Line::startUp(IplImage *image, CvPoint start, int heading, Tire *tires)
{
	//normal vector to the wheel
	int normal = heading + 90;
	
	double magnitude;
	CvPoint location;
	//stores results from the gradient search
	Tire result;
	result.heading = heading;
	
	//set sums to zero
	tires[LEFT].sum = tires[RIGHT].sum = 0;
	tires[LEFT].center = tires[RIGHT].center = start;
	
	for(int i=0; i<2; i++)
	{
		//reset location to beginning
		location = start;
		result.sum = 0;
		
		while(result.sum < THRESHOLD)
		{
			//take a step normal to the current heading
			tires[i].center.y = tires[i].center.y - 10*cos(normal*PI/180.);
			tires[i].center.x = tires[i].center.x - 10*sin(normal*PI/180.);
			result.center = tires[i].center;
			
			//perform a gradient descent here
			gradientDescent(image, &result);
			
			//adjust heading to keep wheels heading close to robot heading
			result.heading = result.heading*0.4 + heading*0.6;
			
			//check to see if the tire is too far away
			magnitude = sqrt( (tires[i].center.x - location.x)*
							  (tires[i].center.x - location.x) +
							  (tires[i].center.y - location.y)*
							  (tires[i].center.y - location.y) );
							  
			if(magnitude > TRACKWIDTH)
			{
				//reset tire to starting location
				tires[i].center = location;
				
				//take a big step in the direction of your heading
				tires[i].center.y = tires[i].center.y - bigStep*cos(heading*PI/180.);
				tires[i].center.x = tires[i].center.x - bigStep*sin(heading*PI/180.);
				
				//set starting location as the current center of tires
				location = tires[i].center;
			}
			//debugging code
			cvCircle(vMap, tires[i].center, 2, cvScalar(75), 1);
			cvShowImage("Orig", vMap);
			cvWaitKey(10);
		}
		//set tires to the result
		tires[i].center = result.center;
		tires[i].heading= result.heading;
		
		//set the normal to the opposite
		normal -= 180;
	}
}

void Line::main(IplImage *image, IplImage *map, CvPoint start, int heading)
{
	//TODO: verify that there are lines to be found
	
	//take a step in the heading to make sure the lines overlap
	start.y = start.y + bigStep*cos(heading*PI/180.);
	start.x = start.x + bigStep*sin(heading*PI/180.);
	
	makeLineMap(image, map, start, heading);
	CvPoint run1L = initialL;
	CvPoint run1R = initialR;
	
	//take a step in the heading to make sure the lines overlap
	start.y = start.y - 2*bigStep*cos(heading*PI/180.);
	start.x = start.x - 2*bigStep*sin(heading*PI/180.);
	
	makeLineMap(image, map, start, heading-180);
	cvLine(map, run1L, initialR, cvScalar(255), 5);
	cvLine(map, run1R, initialL, cvScalar(255), 5);
}

void Line::makeLineMap(IplImage *image, IplImage *map, CvPoint start, int heading)
{
	int primary, secondary, color, normal, norm;
	double dist_to_last = 0, dist[2], angle_orig, angle_new, x, y, det, unit_x, unit_y, norm_x, norm_y;
	
	//create a debugging image
	vMap = cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 1);
	cvCopy(image, vMap);
	
	//initialize an axle for the line map
	Axle axle;
	axle.state = 0;
	axle.tires[LEFT].heading = axle.tires[RIGHT].heading = heading;
	axle.tires[RIGHT].center  = axle.tires[RIGHT].center  = start;
	
	//run startup to find the initial position of the tires
	startUp(image, start, heading, axle.tires);
	initialL = axle.tires[LEFT].center;
	initialR = axle.tires[RIGHT].center;
	
	//debugging code
	cvCircle(vMap, axle.tires[LEFT].center, 2, cvScalar(125), 2);
	cvCircle(vMap, axle.tires[RIGHT].center, 2, cvScalar(125), 2);
	cvLine(vMap, axle.tires[LEFT].center, axle.tires[RIGHT].center, cvScalar(125), 1);
	
	//initialize the rest of the axle
	axle.tires[LEFT].state = axle.tires[RIGHT].state = 1;
	axle.tires[LEFT].last  = axle.tires[LEFT].center;
	axle.tires[RIGHT].last = axle.tires[RIGHT].center;	
	
	//stores tire location before the tire steps
	CvPoint* initial = new CvPoint[2];
	
	//main algorithm that searches for the lines in the image and creates a vision map
	while( dist_to_last < 75 )
	{
		//set left and right tire initial position
		initial[LEFT]  = axle.tires[LEFT].center;
		initial[RIGHT] = axle.tires[RIGHT].center;
		
		//calculate the current width of the axle
		axle.width = sqrt( (axle.tires[LEFT].center.x - axle.tires[RIGHT].center.x)*	
						   (axle.tires[LEFT].center.x - axle.tires[RIGHT].center.x) +
						   (axle.tires[LEFT].center.y - axle.tires[RIGHT].center.y)*
						   (axle.tires[LEFT].center.y - axle.tires[RIGHT].center.y) );
						  
		//axle state 0 and 1 we do nomral steps and for state 2 we do robust search
		if( axle.state == 2 )
		{
			// **** 1.Step the left and right wheel **** //
			axle.tires[RIGHT].center.y = initial[RIGHT].y - bigStep*cos(axle.tires[RIGHT].heading*PI/180.);
			axle.tires[RIGHT].center.x = initial[RIGHT].x - bigStep*sin(axle.tires[RIGHT].heading*PI/180.);
			axle.tires[LEFT].center.y = initial[LEFT].y - bigStep*cos(axle.tires[LEFT].heading*PI/180.);
			axle.tires[LEFT].center.x = initial[LEFT].x - bigStep*sin(axle.tires[LEFT].heading*PI/180.);
			
			// **** 2. Robust search on the left and right side **** //
			for(int i=0; i<2; i++)
			{	
				//perform the robust search
				robustSearch(image, &axle.tires[i], THRESHOLD);
				
				//check to see if a line was found
				if(axle.tires[i].index==0)
				{
					//cout<<i<<" tire found the line!\n";
					
					//set tire state back to "line"
					axle.tires[i].state = 1;
					
					//set axle state to step with this tire
					axle.state = i;
					
					//debugging code
					cvCircle(vMap, axle.tires[i].center, 3, cvScalar(100), 2);
				}
				else
				{
					//increase the search width for the next step
					axle.tires[i].search_width += 5;
					
					//check to make sure the search width is not to large
					if( axle.tires[i].search_width*5 > axle.width/2. )
						axle.tires[i].search_width = axle.width/10.;
						
					//calculate the distance to last known point on each side and average
					dist[LEFT] = sqrt( (axle.tires[LEFT].center.x - axle.tires[LEFT].last.x)*
									   (axle.tires[LEFT].center.x - axle.tires[LEFT].last.x) +
									   (axle.tires[LEFT].center.y - axle.tires[LEFT].last.y)*
									   (axle.tires[LEFT].center.y - axle.tires[LEFT].last.y) );
					dist[RIGHT] = sqrt( (axle.tires[RIGHT].center.x - axle.tires[RIGHT].last.x)*
										(axle.tires[RIGHT].center.x - axle.tires[RIGHT].last.x) +
										(axle.tires[RIGHT].center.y - axle.tires[RIGHT].last.y)*
										(axle.tires[RIGHT].center.y - axle.tires[RIGHT].last.y) );
					//average the two distances
					dist_to_last = (dist[LEFT]+dist[RIGHT])/2.;
			
					//debugging code
					cvCircle(vMap, axle.tires[LEFT].center, 3, cvScalar(175), 2);
					cvCircle(vMap, axle.tires[RIGHT].center, 3, cvScalar(175), 2);
				}
			}
		}
		else
		{
			//determine the primary and secondary tires
			if(axle.state == 0)
			{
				cout<<"****STEPPING LEFT****\n";
				primary   = 0;
				secondary = 1;
				color     = 75;
				norm      = -90;
			}
			else
			{
				cout<<"***STEPPING RIGHT***\n";
				primary   = 1;
				secondary = 0;
				color     = 150;
				norm      = 90;
			}
			
			// **** 1. Take a step with the primary wheel **** /
			axle.tires[primary].center.y = initial[primary].y - bigStep*cos(axle.tires[primary].heading*PI/180.);
			axle.tires[primary].center.x = initial[primary].x - bigStep*sin(axle.tires[primary].heading*PI/180.);
			
			// **** 2. Optimize using the gradient descent method **** //
			gradientDescent(image, &axle.tires[primary]);
			
			//check to make sure the threshold is met
			if( axle.tires[primary].sum <THRESHOLD)
			{
				//cout<<primary<<" tire fell off the line!\n";
				//set state to "no line"
				axle.tires[primary].state = 0;
				
				//set the search_width used in the robust search
				axle.tires[primary].search_width = 5;
				
				//reset to position before step
				axle.tires[primary].center = initial[primary];
				
				//check the state of the other tire
				if(axle.tires[secondary].state)
					axle.state = secondary;
				else
					axle.state = 2;
			}
			else
			{
				// **** 3. Draw line on the final image **** //
				//Draw line on the map from current to last known
				cvLine(map, axle.tires[primary].center, axle.tires[primary].last, cvScalar(255), 5);
				//set last know to the current point
				axle.tires[primary].last = axle.tires[primary].center;
				
				// **** 4. Translate secondary along its heading **** //
				/*axle.tires[secondary].center.y = axle.tires[secondary].center.y + abs(cos(axle.tires[secondary].heading*PI/180.))*(axle.tires[primary].center.y - initial[primary].y);
				axle.tires[secondary].center.x = axle.tires[secondary].center.x + abs(sin(axle.tires[secondary].heading*PI/180.))*(axle.tires[primary].center.x - initial[primary].x);*/
				
				//form m*x + b lines from normal of primary and heading of secondary
				unit_x = -sin(axle.tires[secondary].heading*PI/180.);
				unit_y = -cos(axle.tires[secondary].heading*PI/180.);
				Coefficients minor = genLine(axle.tires[secondary].center.x,
											 axle.tires[secondary].center.y,
											 axle.tires[secondary].center.x + unit_x,
											 axle.tires[secondary].center.y + unit_y );
				
				norm_x = -1000*sin((axle.tires[primary].heading+norm)*PI/180.);
				norm_y = -1000*cos((axle.tires[primary].heading+norm)*PI/180.);
				norm_x += axle.tires[primary].center.x;
				norm_y += axle.tires[primary].center.y;
				Coefficients main = genLine(axle.tires[primary].center.x, axle.tires[primary].center.y, norm_x, norm_y );
				
				//cvLine(vMap, axle.tires[primary].center, cvPoint(norm_x,norm_y), cvScalar(150), 1);
				//cvLine(vMap, axle.tires[secondary].center, cvPoint(axle.tires[secondary].center.x + 100*unit_x, axle.tires[secondary].center.y + 100*unit_y), cvScalar(150), 1);
				
				//find the intersection of the two points
				det = minor.a*main.b - minor.b*main.a;
				
				if(det == 0)
				{
					//Lines are parallel
				}
				else
				{
					x = (main.b*-minor.c - minor.b*-main.c)/det;
					y = (minor.a*-main.c - main.a*-minor.c)/det;
					
					x = x - axle.tires[secondary].center.x;
					y = y - axle.tires[secondary].center.y;
					
					if( (x*x + y*y) < ((x+unit_x)*(x+unit_x) + (y+unit_y)*(y+unit_y)) )
					{
						axle.tires[secondary].center.x += percent_step * x;
						axle.tires[secondary].center.y += percent_step * y;
					}
				}
				// **** 5. Optimize the secondary tire **** //
				if(axle.tires[secondary].state)
				{
					//do a gradient descent because we are on a line
					gradientDescent(image, &axle.tires[secondary]);
					
					//check to make sure the threshold is met
					if(axle.tires[secondary].sum < THRESHOLD)
					{
						//cout<<secondary<<" tire fell off the line!\n";
						
						//set state to "no line"
						axle.tires[secondary].state = 0;
						
						//set tire back to position before step
						axle.tires[secondary].center = initial[secondary];
						
						//set the search width for the robust search
						axle.tires[secondary].search_width = 5;
						
						//set axle state to step with current tire again
						axle.state = primary;
					}
					else
					{
						//switch the axle state
						axle.state = secondary;
						
						//draw good line on final map
						cvLine(map, axle.tires[secondary].last, axle.tires[secondary].center, cvScalar(255), 5);
						//set last known to current center
						axle.tires[secondary].last = axle.tires[secondary].center;
						
						//debugging code
						cvCircle(vMap, axle.tires[LEFT].center, 2, cvScalar(color), 2);
						cvCircle(vMap, axle.tires[RIGHT].center, 2, cvScalar(color), 2);
						cvLine(vMap, axle.tires[LEFT].center, axle.tires[RIGHT].center, cvScalar(color), 1);
						
						//find the angle between primary heading and tangent line
						angle_orig = dotproduct(axle.tires, primary);
						//cout<<"Before DP: "<<angle_orig<<endl;
						for(int step=0; step<3; step++)
						{	
							//set initial location to current center
							initial[secondary] = axle.tires[secondary].center;
							
							//take a step with the secondary wheel
							axle.tires[secondary].center.y = initial[secondary].y - bigStep*cos(axle.tires[secondary].heading*PI/180.);
							axle.tires[secondary].center.x = initial[secondary].x - bigStep*sin(axle.tires[secondary].heading*PI/180.);
							
							//optimize at current location
							gradientDescent(image, &axle.tires[secondary]);
							
							//check to make sure the threshold is met
							if(axle.tires[secondary].sum < THRESHOLD)
							{
								//cout<<secondary<<" tire fell off the line!\n";
								//set state to "no line"
								axle.tires[secondary].state = 0;
						
								//set the search width for the robust search
								axle.tires[secondary].search_width = 5;
						
								//set axle state to step with current tire again
								axle.state = primary;
								
								//reset back to location before step
								axle.tires[secondary].center = initial[secondary];
							}
							else
							{
								//recalculate the angle between the wheels
								angle_new = dotproduct(axle.tires, primary);
								//cout<<"Step and New DP: "<<angle_new<<endl;
								if(angle_new < angle_orig)
								{
									//set the new difference
									angle_orig = angle_new;
									
									//draw in the good line on the map
									cvLine(map, axle.tires[secondary].last, axle.tires[secondary].center, cvScalar(255), 5);
									//set last known to current center
									axle.tires[secondary].last = axle.tires[secondary].center;
									
									//debugging code
									cvCircle(vMap, axle.tires[secondary].center, 2, cvScalar(color), 2);
									cvLine(vMap, axle.tires[LEFT].center, axle.tires[RIGHT].center, cvScalar(color), 1);
								}
								else
								{
									//reset back to locatin before step
									axle.tires[secondary].center = initial[secondary];
									break;
								}
							}
						}
					}
				}
				else
				{
					//not on a line so do a robust search
					//cout<<"Using robust search\n";
					robustSearch(image, &axle.tires[secondary], THRESHOLD);
					
					if(axle.tires[secondary].index==0)
					{
						//cout<<secondary<<" tire found the line!\n";
						//set the state back to "line"
						axle.tires[secondary].state = 1;
						//set the state to step with secondary wheel next
						axle.state = secondary;
					}
					else
					{
						//adjust heading of secondary towards primary tire
						axle.tires[secondary].heading = axle.tires[secondary].heading*0.7 + axle.tires[primary].heading*0.3;				
						
						//increase the search width for the next step
						axle.tires[secondary].search_width += 5;
						
						//check to ensure that the search width is not too big
						if( axle.tires[secondary].search_width*5 > axle.width/2. )
							axle.tires[secondary].search_width = axle.width/10.;
					}
					//debugging code
					cvCircle(vMap, axle.tires[LEFT].center, 2, cvScalar(color), 2);
					cvCircle(vMap, axle.tires[RIGHT].center, 2, cvScalar(color), 2);
					cvLine(vMap, axle.tires[LEFT].center, axle.tires[RIGHT].center, cvScalar(color), 1);
				}
			}
		}
		cout<<"Left: "<<axle.tires[LEFT].center.x<<" "<<axle.tires[LEFT].center.y<<" "<<axle.tires[LEFT].heading<<" State: "<<axle.tires[LEFT].state<<endl;
		cout<<"Right: "<<axle.tires[RIGHT].center.x<<" "<<axle.tires[RIGHT].center.y<<" "<<axle.tires[RIGHT].heading<<" State: "<<axle.tires[RIGHT].state<<endl<<endl;
		cvShowImage("Orig", vMap);
		cvWaitKey(0);
	}
}
				

Line::~Line()
{
	cvReleaseImage(&line_mask);
	cvReleaseImage(&no_line_mask);
	cvReleaseImage(&output);
}
