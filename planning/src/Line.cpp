/*******************************
*         Line.cpp             *
*     Author: Aaron Deal       *
*******************************/

#include "Line.h"
#include "fstream"
#include <iostream>

using namespace std;

Line::Line()
{
	mask_width = 30;
	line_width = 5;
	
	//create the mask that is use in most functions
	line_mask    = cvCreateImage(cvSize(mask_width, mask_width), IPL_DEPTH_8U, 1 );
	no_line_mask = cvCreateImage(cvSize(mask_width, mask_width), IPL_DEPTH_8U, 1 );
	output       = cvCreateImage(cvSize(mask_width, mask_width), IPL_DEPTH_8U, 1 );
	
	//variables for the quals line finder
	start_point = false;
	threshold = THRESHOLD;
	final = cvCreateImage( cvSize(1500,1500), 8, 3);
	vMap = cvCreateImage(cvSize(1500,1500), IPL_DEPTH_8U, 3);
	
	for( int i=0; i<5; i++){
		previous[0][i] = cvPoint(0,0);
		previous[0][i] = cvPoint(0,0);
	}
}

void Line::init(string config_path){
	ifstream config_file( config_path.c_str() );
	
	if(config_file.is_open()){
		//read in the config variables
		config_file>>THRESHOLD;
		//cout<<THRESHOLD<<endl;
		config_file.ignore(300,'\n');
		config_file>>TRACKWIDTH;
		//cout<<TRACKWIDTH<<endl;
		config_file.ignore(300,'\n');
		config_file>>littleStep;
		//cout<<littleStep<<endl;
		config_file.ignore(300,'\n');
		config_file>>bigStep;
		//cout<<bigStep<<endl;
		config_file.ignore(300,'\n');
		//config_file>>percent_step;
		cout<<percent_step<<endl;
		config_file.ignore(300,'\n');
		config_file>>map_line_width;
		//cout<<map_line_width<<endl;
		config_file.close();
	}
	else{
		cout<<"Invalid path for line config file\n";
	}
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
	start.x = (mask_width/2.) - 100*sin(tire->heading*PI/180.);
	start.y = (mask_width/2.) - 100*cos(tire->heading*PI/180.);
	
	//find the ending point of the mask
	end.x = (mask_width/2.) + 100*sin(tire->heading*PI/180.);
	end.y = (mask_width/2.) + 100*cos(tire->heading*PI/180.);
	
	//draw line on the mask and invert for another mask
	cvSetZero(line_mask);
	cvSet(no_line_mask, cvScalar(255));
	cvLine(line_mask, start, end, cvScalar(255), line_width);
	cvSub(no_line_mask, line_mask, no_line_mask);
	
	for(int i=-1; i<=1; i++)
	{
		//location of the upper left corner of the region of interest
		start.x = initial.center.x - (mask_width/2.) + i*2*sin((initial.heading-90)*PI/180.);
		start.y = initial.center.y - (mask_width/2.) + i*2*cos((initial.heading-90)*PI/180.);
		
		if(start.x < 0 || start.y < 0 || start.x+mask_width > image->width || start.y+mask_width > image->height)
		{
			cout<<"Translatoin: "<<start.x<<"  "<<start.y<<"  "<<start.x+mask_width<<"  "<<start.y+mask_width<<endl;
		}
		else
		{
			//set the region of interest on the image
			cvSetImageROI(image, cvRect(start.x, start.y, mask_width, mask_width));
		
			//And and sum the two images
			line    = sum(image, line_mask);
			no_line = sum(image, no_line_mask);
			total   = line - no_line/5.;
		
			//reset the ROI
			cvResetImageROI(image);
		
			if(total > tire->sum)
			{
				//store the center of the mask
				tire->center.x = start.x + (mask_width/2.);
				tire->center.y = start.y + (mask_width/2.);
				//cout<<"New: "<<tire->center.x<<" "<<tire->center.y<<endl; 
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
	start.x = initial.center.x - (mask_width/2.);
	start.y = initial.center.y - (mask_width/2.);
	
	if(start.x < 0 || start.y < 0 || start.x+mask_width > image->width || start.y+mask_width > image->height)
	{
		cout<<"Rotation: "<<start.x<<"  "<<start.y<<"  "<<start.x+mask_width<<"  "<<start.y+mask_width<<endl;
	}
	else
	{
		//set the region of interest on the image
		cvSetImageROI(image, cvRect(start.x, start.y, mask_width, mask_width));
	
		for(int i=-3; i<=3; i++)
		{
			//find the starting point of the mask
			start.x = (mask_width/2.) - 100*sin((initial.heading-3*i)*PI/180.);
			start.y = (mask_width/2.) - 100*cos((initial.heading-3*i)*PI/180.);
	
			//find the ending point of the mask
			end.x = (mask_width/2.) + 100*sin((initial.heading-3*i)*PI/180.);
			end.y = (mask_width/2.) + 100*cos((initial.heading-3*i)*PI/180.);
	
			//draw line on the mask and invert for another mask
			cvSetZero(line_mask);
			cvSet(no_line_mask, cvScalar(255));
			cvLine(line_mask, start, end, cvScalar(255), line_width);
			cvSub(no_line_mask, line_mask, no_line_mask);
		
			//And and sum the two images
			line    = sum(image, line_mask);
			no_line = sum(image, no_line_mask);
			total   = line - no_line/5.;
		
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
	
	//for(int i=0; i<1; i++)
	//{
		//set are start point before gradient descent
		start = tire->center;
		angle = tire->heading;
		
		//set the index to -1 to enter the loop
		tire->index = -1;
	
		//set the sum to a big negative number to start off with
		tire->sum = -100000;
	
		//Do a translation first
		while(tire->index != 0){
			translation(image, tire);
		}
	
		//set the index to not 0 to enter the while loop
		tire->index = -1;
	
		//Do a translation first
		while(tire->index != 0){
			rotation(image, tire);
		}
		
		//if(start.x == tire->center.x && start.y == tire->center.y && angle == tire->heading)
		//	break;
	//}
}

void Line::robustSearch(IplImage *image, Tire *tire, int threshold)
{
	CvPoint start, end;
	Tire initial = *tire;
	int line, no_line,total;
	
	//set tire index to -1 to signify that no line was found -- 1 = line found
	tire->index = -1;
	
	//find the starting point of the mask
	start.x = (mask_width/2.) - 100*sin(tire->heading*PI/180.);
	start.y = (mask_width/2.) - 100*cos(tire->heading*PI/180.);
	
	//find the ending point of the mask
	end.x = (mask_width/2.) + 100*sin(tire->heading*PI/180.);
	end.y = (mask_width/2.) + 100*cos(tire->heading*PI/180.);
	
	//draw line on the mask and invert for another mask
	cvSetZero(line_mask);
	cvSet(no_line_mask, cvScalar(255));
	cvLine(line_mask, start, end, cvScalar(255), line_width);
	cvSub(no_line_mask, line_mask, no_line_mask);
	
	for(int i=-tire->search_width; i<=tire->search_width; i++)
	{
		//location of the upper left corner of the region of interest
		start.x = initial.center.x - (mask_width/2.) + i*(line_width/2.)*sin((initial.heading+90)*PI/180.);
		start.y = initial.center.y - (mask_width/2.) + i*(line_width/2.)*cos((initial.heading+90)*PI/180.);
		
		if(start.x < 0 || start.y < 0 || start.x+mask_width > image->width || start.y+mask_width > image->height)
		{
			cout<<"Robust Search: "<<start.x<<"  "<<start.y<<"  "<<start.x+mask_width<<"  "<<start.y+mask_width<<endl;
		}
		else
		{
			//set the region of interest on the image
			cvSetImageROI(image, cvRect(start.x, start.y, mask_width, mask_width));
		
			//And and sum the two images
			line    = sum(image, line_mask);
			no_line = sum(image, no_line_mask);
			total   = line - no_line/5.;
		
			//reset the ROI
			cvResetImageROI(image);
		
			if(total > threshold)
			{
				//set the new threshold to beat
				threshold = total;
			
				//store the center of the mask
				tire->center.x = start.x + (mask_width/2.);
				tire->center.y = start.y + (mask_width/2.);
			
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

void Line::findLines(IplImage *image, CvPoint robot, Tire *tires)
{
	//variables that should go in a config file or class variables
	double track_width = 3.3;
	double mask_size = 5*(1/model->meterToPixel);
	IplImage *output = cvCreateImage( cvSize(mask_size, mask_size), IPL_DEPTH_8U, 1);
	IplImage *line   = cvCreateImage( cvSize(mask_size, mask_size), IPL_DEPTH_8U, 1);
	IplImage *no_line= cvCreateImage( cvSize(mask_size, mask_size), IPL_DEPTH_8U, 1);
	
	//variables used in function
	CvPoint leftstart, rightstart, start, end;
	CvRect roi;
	
	for(double track = 2.6; track<=3.6; track += .1){
		for(int heading = 0; heading <= 180; heading += 5){
			//set the mask to zero
			cvSetZero(line);
			cvSet(no_line, cvScalar(255));
			
			//draw lines that are a track width apart
			leftstart  = cvPoint( mask_size/2. - (track/2)*(1/model->meterToPixel)*sin((heading-90)*PI/180.), mask_size/2. - (track/2)*(1/model->meterToPixel)*cos((heading-90)*PI/180.) );
										  
			rightstart  = cvPoint( mask_size/2. + (track/2)*(1/model->meterToPixel)*sin((heading-90)*PI/180.), mask_size/2. + (track/2)*(1/model->meterToPixel)*cos((heading-90)*PI/180.) );
								   
			//draw the left line
			start = cvPoint( leftstart.x - mask_size*sin(heading*PI/180.),
							 leftstart.y - mask_size*cos(heading*PI/180.) );
			end   = cvPoint( leftstart.x + mask_size*sin(heading*PI/180.),
							 leftstart.y + mask_size*cos(heading*PI/180.) );	 
			cvLine( line, start, end, cvScalar(255), 5);
	
			//draw the right line
			start = cvPoint( rightstart.x - mask_size*sin(heading*PI/180.),
							 rightstart.y - mask_size*cos(heading*PI/180.) );
			end   = cvPoint( rightstart.x + mask_size*sin(heading*PI/180.),
							 rightstart.y + mask_size*cos(heading*PI/180.) );
			cvLine( line, start, end, cvScalar(255), 5);
			cvSub( no_line, line, no_line);
			//cvShowImage("line", line);
			//cvShowImage("no line", no_line);
			//cvWaitKey(0);
			for(int i = -4; i<=4; i++){
				//location of the upper left corner of the region of interest
				roi.x = robot.x - (mask_size/2.) + i*(.25*(1/model->meterToPixel))*sin((heading-90)*PI/180.);
				roi.y = robot.y - (mask_size/2.) + i*(.25*(1/model->meterToPixel))*cos((heading-90)*PI/180.);
				roi.width = roi.height = mask_size;
				
				//set the region of interest
				cvSetImageROI(image, roi);
				
				//and the images together
				cvAnd(image, line, output);
				
				//sum the output image
				CvScalar tot_line = cvSum(output);
				
				//and the images together
				cvAnd(image, no_line, output);
				
				CvScalar tot_no_line = cvSum(output);
				
				//reset the region of interest
				cvResetImageROI(image);
				
				int total = tot_line.val[0] - tot_no_line.val[0]/9.;
				//cout<<total<<endl;
				if( total > threshold ){
					//set the new threshold to beat
					threshold = total;
					
					//set the axle width
					axle_width = track*(1/model->meterToPixel);
					
					tires[0].index = 1;
					
					//set left wheel position
					tires[LEFT].center.x = leftstart.x - (mask_size/2.) + i*(.25*(1/model->meterToPixel))*sin((heading-90)*PI/180.) + robot.x;
					tires[LEFT].center.y = leftstart.y - (mask_size/2.) + i*(.25*(1/model->meterToPixel))*cos((heading-90)*PI/180.) + robot.y;
					tires[LEFT].heading = heading;
					
					//set the right wheel position
					tires[RIGHT].center.x = rightstart.x - (mask_size/2.) + i*(.25*(1/model->meterToPixel))*sin((heading-90)*PI/180.) + robot.x;
					tires[RIGHT].center.y = rightstart.y - (mask_size/2.) + i*(.25*(1/model->meterToPixel))*cos((heading-90)*PI/180.) + robot.y;
					tires[RIGHT].heading = heading;
				}
			}
		}
	}
}

void Line::setInitialHeading(int heading){
	initial_heading = heading;
}

vector<CvPoint2D32f> Line::qualsLineFinder(IplImage *image, CvPoint robot)
{
	vector<CvPoint> new_start = qualsStartUp(image, robot);
	
	vector<CvPoint2D32f> unit_vector;
	/*
	if(new_start.size()!=0){
		for(int i=0; i<3; i++){
			midPoints.push_back(model->pixeltometer(new_start[i].x, new_start[i].y));
		}
	}*/
	threshold = 2*THRESHOLD;
	if( new_start.size() != 0 )
	{
		//set start point to try saying we have a start point
		start_point = true;
		
		//make a vector from the robot to each mid point
		double v1X = new_start[0].x - new_start[1].x;
		double v1Y = new_start[0].y - new_start[1].y;
		double v2X = new_start[2].x - new_start[1].x;
		double v2Y = new_start[2].y - new_start[1].y;
		
		//find the magnitude of each vector
		double mag1 = sqrt(v1X*v1X + v1Y*v1Y);
		double mag2 = sqrt(v2X*v2X + v2Y*v2Y);
		
		//find heading unit vector
		double uvX = -sin(initial_heading*PI/180);
		double uvY = -cos(initial_heading*PI/180);
		
		//make unit vectors to each point
		v1X /= mag1;
		v1Y /= mag1;
		v2X /= mag2;
		v2Y /= mag2;
		
		cvSetZero(model->worldModelVisionAdd);
		/*CvPoint start = cvPoint(1000*v1X, 1000*v1Y);
		CvPoint end = cvPoint(1000*v2X, 1000*v2Y);
		
		CvPoint start1 = cvPoint(1000*v1X, 1000*v1Y);
		CvPoint end1 = cvPoint(1000*v2X, 1000*v2Y);
		CvRect roi=model->getRegionCentered(300,300);
		cvSetImageROI(model->worldModelVisionAdd,roi);
		line_heading = -90 - atan2(v1Y, v1X)*180/PI;
		CvPoint2D32f n_vec;
			
		n_vec.x = -sin((line_heading+90)*PI/180);
		n_vec.y = -cos((line_heading+90)*PI/180);

		start.x+=(axle_width/2.)*n_vec.x;
		start.y+=(axle_width/2.)*n_vec.y;
		end.x+=- (axle_width/2.)*n_vec.x;
		end.y+=- (axle_width/2.)*n_vec.y;
			
		cvLine(model->worldModelVisionAdd, start, end, cvScalar(255), 20);
		start1.x-= (axle_width/2.)*n_vec.x;
		start1.y-= (axle_width/2.)*n_vec.y;
		end1.x-= (axle_width/2.)*n_vec.x;
		end1.y-= (axle_width/2.)*n_vec.y;
		
		cvLine(model->worldModelVisionAdd, start1, end1, cvScalar(255), 20);
		cvResetImageROI(model->worldModelVisionAdd);*/		
		
		if( acos(v1X*uvX + v1Y*uvY) < acos(v2X*uvX + v2Y*uvY) ){
			
		//cout<<"fuckup1\n";
			//set the new heading
			line_heading = -90 - atan2(v1Y, v1X)*180/PI;
			CvPoint2D32f n_vec;
			unit_vector.push_back(cvPoint2D32f(v1X,v1Y));
			n_vec.x = -sin((line_heading+90)*PI/180);
			n_vec.y = -cos((line_heading+90)*PI/180);
			//cvLine(final, new_start[1], cvPoint( new_start[1].x + v1X*20, new_start[1].y + v1Y*20 ), cvScalar(0, 255, 0), 2);
			//set the new start points for the tires
			cvCircle(final, new_start[0], 4, cvScalar(0,255, 0), 2);
			start[0].x = new_start[0].x + (axle_width/2.)*n_vec.x;
			start[0].y = new_start[0].y + (axle_width/2.)*n_vec.y;
			CvPoint start1 = cvPoint(150*v1X+start[0].x, 150*v1Y+start[0].y);
			CvPoint end1 = cvPoint(-150*v1X+start[0].x, -150*v1Y+start[0].y);
			cvLine(model->worldModelVisionAdd, start1, end1, cvScalar(255), 10);
			cvCircle(final, start[0], 4, cvScalar(255, 0, 0), 2);
			start[1].x = new_start[0].x - (axle_width/2.)*n_vec.x;
			start[1].y = new_start[0].y - (axle_width/2.)*n_vec.y;
			start1 = cvPoint(150*v1X+start[1].x, 150*v1Y+start[1].y);
			end1 = cvPoint(-150*v1X+start[1].x, -150*v1Y+start[1].y);
			cvLine(model->worldModelVisionAdd, start1, end1, cvScalar(255), 10);
			cvCircle(final, start[1], 4, cvScalar(0, 0, 255), 2);
			//cout<<"left: "<<start[0].x<<" "<<start[0].y<<endl;
			//cout<<"right: "<<start[1].x<<" "<<start[1].y<<endl;
		}
		else{
		//cout<<"fuckup2\n";
			//set the new heading
			line_heading = 90 -atan2(v1Y, v1X)*180/PI;
			//cvLine(final, new_start[1], cvPoint( new_start[1].x + v2X*20, new_start[1].y + v2Y*20 ), cvScalar(0, 255, 0), 2);
			unit_vector.push_back(cvPoint2D32f(v2X,v2Y));
			CvPoint2D32f n_vec;
			n_vec.x = -sin((line_heading+90)*PI/180);
			n_vec.y = -cos((line_heading+90)*PI/180);
			//set the new points for the tires
			cvCircle(final, new_start[2], 4, cvScalar(0,255, 0), 2);
			start[0].x = new_start[2].x + (axle_width/2.)*n_vec.x;
			start[0].y = new_start[2].y + (axle_width/2.)*n_vec.y;
			CvPoint start1 = cvPoint(150*v1X+start[0].x, 150*v1Y+start[0].y);
			CvPoint end1 = cvPoint(-150*v1X+start[0].x, -150*v1Y+start[0].y);
			cvLine(model->worldModelVisionAdd, start1, end1, cvScalar(255), 10);
			cvCircle(final, start[0], 4, cvScalar(255, 0, 0), 2);
			start[1].x = new_start[2].x - (axle_width/2.)*n_vec.x;
			start[1].y = new_start[2].y - (axle_width/2.)*n_vec.y;
			start1 = cvPoint(150*v1X+start[1].x, 150*v1Y+start[1].y);
			end1 = cvPoint(-150*v1X+start[1].x, -150*v1Y+start[1].y);
			cvLine(model->worldModelVisionAdd, start1, end1, cvScalar(255), 10);
			cvCircle(final, start[1], 4, cvScalar(0, 0, 255), 2);
			//cout<<"left: "<<start[0].x<<" "<<start[0].y<<endl;
			//cout<<"right: "<<start[1].x<<" "<<start[1].y<<endl;
		}
		cout<<"Found new start with heading: "<<line_heading<<" Threshold: "<<threshold<<endl;
   		//cvSetImageROI(final,cvRect(robot.x-250, robot.y-250, 500, 500));
		//cvShowImage("Lines", final);
		//cvWaitKey(4);
		//cvResetImageROI(final);
	}
	//midpoints of the axle
	/*vector<CvPoint> midpoints;
	vector<CvPoint2D32f> points;
	
	if(start_point){
		//create an axle
		Axle axle;
		axle.tires[LEFT].heading = axle.tires[RIGHT].heading = line_heading;
		axle.tires[LEFT].state = axle.tires[RIGHT].state = 1;
		axle.tires[LEFT].last = axle.tires[LEFT].center = start[0];
		axle.tires[RIGHT].last = axle.tires[RIGHT].center = start[1];
		
		//search for lines in map
		midpoints = makeLineMap(image, model->worldModelVisionAdd, axle);
		for(int i = 0; i<midpoints.size(); i++){
			points.push_back( model->pixeltometer(midpoints[i].x,midpoints[i].y) );
			cvCircle(final, midpoints[i], 4, cvScalar( 0, 128, 0), 2);
		}
   		cvSetImageROI(final,cvRect(robot.x-250, robot.y-250, 500, 500));
		cvShowImage("Lines", final);
		//cvWaitKey(0);
		cvResetImageROI(final);
	}*/
	return unit_vector;

}

vector<CvPoint> Line::qualsStartUp(IplImage *image, CvPoint robot)
{
	//initialize tires to send to the find lines algorithm
   	Tire tires[2];
   	tires[0].index = tires[1].index = 0;
	
	CvPoint initial, start, end;
	
	//call the find lines function
	findLines(image, robot, tires);
	
	//midpoints between lines if found
	vector <CvPoint> midpoints;
	
	//cvCvtColor(image,final,CV_GRAY2RGB);
	
	if( tires[0].index ){
		//debugging code
		
		double heading = tires[0].heading;
   		
		initial = tires[0].center;
		start.x = initial.x + 50.*sin(heading*PI/180.);
		start.y = initial.y + 50.*cos(heading*PI/180.);
		end.x = initial.x - 50.*sin(heading*PI/180.);
		end.y = initial.y - 50.*cos(heading*PI/180.);
		cvLine(final, start, end, cvScalar(255, 0, 0), 2);
		cvCircle(final, initial, 2, cvScalar(0, 255, 0), 2);
		
		initial = tires[1].center;
		start.x = initial.x + 50.*sin(heading*PI/180.);
		start.y = initial.y + 50.*cos(heading*PI/180.);
		end.x = initial.x -50.*sin(heading*PI/180.);
		end.y = initial.y - 50.*cos(heading*PI/180.);
		cvLine(final, start, end, cvScalar(255, 0, 0), 2);
		cvCircle(final, initial, 2, cvScalar(0, 255, 0), 2);
		
		//find the robots new position between lines
		robot.x = tires[LEFT].center.x + (tires[RIGHT].center.x - tires[LEFT].center.x)/2.;
		robot.y = tires[LEFT].center.y + (tires[RIGHT].center.y - tires[LEFT].center.y)/2.;
		
		start.x = robot.x + 50.*sin(heading*PI/180.);
		start.y = robot.y + 50.*cos(heading*PI/180.);
		end.x = robot.x -50.*sin(heading*PI/180.);
		end.y = robot.y - 50.*cos(heading*PI/180.);
		//cvCircle(final, start, 2, cvScalar(0, 255, 0), 2);
		//cvCircle(final, end, 2, cvScalar(0, 255, 0), 2);
		
		//push the two center points into midpoints
		midpoints.push_back(start);
		midpoints.push_back(robot);
		midpoints.push_back(end);
		
   		cvSetImageROI(final,cvRect(robot.x-250, robot.y-250, 500, 500));
		//cvShowImage("Lines", final);
	//	cvWaitKey(0);
		cvResetImageROI(final);
	}
	
	return midpoints;
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
		int steps = 0;
		int check = normal;
		while(result.sum < THRESHOLD && steps < 10)
		{
			//take a step normal to the current heading
			tires[i].center.y = tires[i].center.y - round(line_width*cos(normal*PI/180.));
			tires[i].center.x = tires[i].center.x - round(line_width*sin(normal*PI/180.));
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
				
				//increase steps
				steps++;
				
				//cvWaitKey(4);
			}
			
			
			//debugging code
			//cvCircle(vMap, tires[i].center, 2, cvScalar(75), 1);
			//cvSetImageROI(vMap, cvRect(start.x - 250, start.y-250, 500, 500));
			//cvShowImage("Orig", vMap);
			//cvResetImageROI(vMap);
		}
		//set tires to the result
		tires[i].center = result.center;
		tires[i].heading= result.heading;
		
		//set the normal to the opposite
		normal += 180;
	}
}

vector<CvPoint> Line::main(IplImage *image, IplImage *map, CvPoint start, int heading)
{
	//TODO: verify that there are lines to be found
	//cvCvtColor(image, vMap, CV_GRAY2RGB);
	
	//set the line map to black
	cvSetZero(map);
	
	//take a step in the heading to make sure the lines overlap
	start.y = start.y + bigStep*cos(heading*PI/180.);
	start.x = start.x + bigStep*sin(heading*PI/180.);
	
	return makeLineMap(image, map, start, heading);
	
	//take a step in the heading to make sure the lines overlap
	//start.y = start.y - 2*bigStep*cos(heading*PI/180.);
	//start.x = start.x - 2*bigStep*sin(heading*PI/180.);
	
	//makeLineMap(image, map, start, heading-180);
}

vector<CvPoint> Line::makeLineMap(IplImage *image, IplImage *map, CvPoint robot, int heading)
{
	//cvCvtColor(image, vMap, CV_GRAY2RGB);
	
	//initialize an axle for the line map
	Axle axle;
	axle.state = 0;
	axle.tires[LEFT].center = axle.tires[RIGHT].center = robot;
	axle.tires[LEFT].heading = axle.tires[RIGHT].heading = heading;
	
	startUp(image, robot, heading, axle.tires);
	
	//initialize the rest of the axle
	axle.tires[LEFT].state = axle.tires[RIGHT].state = 1;
	axle.tires[LEFT].last  = axle.tires[LEFT].center;
	axle.tires[RIGHT].last = axle.tires[RIGHT].center;
	
	return makeLineMap(image, map, axle);
	
	/*if(!start_point){
		threshold = 3*THRESHOLD;
		vector<CvPoint> new_start = qualsStartUp(image, robot);
	
		if(new_start.size()!=0){
			//set start point to try saying we have a start point
			start_point = true;
	
			//make a vector from the robot to each mid point
			double v1X = new_start[0].x - new_start[1].x;
			double v1Y = new_start[0].y - new_start[1].y;
			double v2X = new_start[2].x - new_start[1].x;
			double v2Y = new_start[2].x - new_start[1].y;
	
			//find the magnitude of each vector
			double mag1 = sqrt(v1X*v1X + v1Y*v1Y);
			double mag2 = sqrt(v2X*v2X + v2Y*v2Y);
	
			//find heading unit vector
			double uvX = -sin(heading*PI/180);
			double uvY = -cos(heading*PI/180);
	
			//make unit vectors to each point
			v1X /= mag1;
			v1Y /= mag1;
			v2X /= mag2;
			v2Y /= mag2;
	
			if( acos(v1X*uvX + v1Y*uvY) < acos(v2X*uvX + v2Y*uvY) ){
				//set the new heading
				line_heading = -90 - atan2(v1Y, v1X)*180/PI;
				//cvLine(final, new_start[1], cvPoint( new_start[1].x + v1X*20, new_start[1].y + v1Y*20 ), cvScalar(0, 255, 0), 2);
				//set the new start points for the tires
				start[0].x = new_start[0].x + (axle_width/2.)*sin((line_heading-90)*PI/180.);
				start[0].y = new_start[0].y + (axle_width/2.)*cos((line_heading-90)*PI/180.);
				//cvCircle(final, start[0], 4, cvScalar(255, 0, 0), 2);
				start[1].x = new_start[0].x - (axle_width/2.)*sin((line_heading-90)*PI/180.);
				start[1].y = new_start[0].y - (axle_width/2.)*cos((line_heading-90)*PI/180.);
				init_location[0] = model->pixeltometer(start[0].x,start[0].y);
				init_location[1] = model->pixeltometer(start[1].x,start[1].y);
				//cout<<init_location[0].x<<"  "<<init_location[0].y<<endl;
				//cvCircle(final, start[1], 4, cvScalar(0, 0, 255), 2);
				//cout<<"left: "<<start[0].x<<" "<<start[0].y<<endl;
				//cout<<"right: "<<start[1].x<<" "<<start[1].y<<endl;
			}
			else{
				//set the new heading
				line_heading = -90 -atan2(v2Y, v2X)*180/PI + 90;
				//cvLine(final, new_start[1], cvPoint( new_start[1].x + v2X*20, new_start[1].y + v2Y*20 ), cvScalar(0, 255, 0), 2);
		
				//set the new points for the tires
				start[0].x = new_start[2].x + (axle_width/2.)*sin((line_heading-90)*PI/180.);
				start[0].y = new_start[2].y + (axle_width/2.)*cos((line_heading-90)*PI/180.);
				//cvCircle(final, start[0], 4, cvScalar(255, 0, 0), 2);
				start[1].x = new_start[2].x - (axle_width/2.)*sin((line_heading-90)*PI/180.);
				start[1].y = new_start[2].y - (axle_width/2.)*cos((line_heading-90)*PI/180.);
				init_location[0] = model->pixeltometer(start[0].x,start[0].y);
				init_location[1] = model->pixeltometer(start[1].x,start[1].y);
				//cvCircle(final, start[1], 4, cvScalar(0, 0, 255), 2);
				//cout<<"left: "<<start[0].x<<" "<<start[0].y<<endl;
				//cout<<"right: "<<start[1].x<<" "<<start[1].y<<endl;
			}
			//cout<<"Found new start with heading: "<<line_heading<<" Threshold: "<<threshold<<endl;
	   		//cvSetImageROI(final,cvRect(robot.x-250, robot.y-250, 500, 500));
			//cvShowImage("Lines", final);
			//cvWaitKey(4);
			//cvResetImageROI(final);
		}
	}
	if(start_point){	
		axle.tires[LEFT].heading  = axle.tires[RIGHT].heading = line_heading;
		axle.tires[LEFT].center.x   = init_location[0].x*(1/model->meterToPixel) + model->translation.x;
		axle.tires[LEFT].center.y   = -init_location[0].y*(1/model->meterToPixel) + model->translation.y;
		axle.tires[RIGHT].center.x  = init_location[1].x*(1/model->meterToPixel) + model->translation.x;
		axle.tires[RIGHT].center.y  = -init_location[1].y*(1/model->meterToPixel) + model->translation.y;
	
		//initialize the rest of the axle
		axle.tires[LEFT].state = axle.tires[RIGHT].state = 1;
		axle.tires[LEFT].last  = axle.tires[LEFT].center;
		axle.tires[RIGHT].last = axle.tires[RIGHT].center;	

		return makeLineMap(image, map, axle);
	}
	else
	{
		vector<CvPoint> nothing;
		return nothing;
	}*/
}
	
vector<CvPoint> Line::makeLineMap(IplImage* image, IplImage *map, Axle axle)
{
	int primary, secondary, normal, norm, index[2] = {0,0};
	double dist_to_last = 0, dist[2], angle_orig, angle_new, x, y, det, unit_x, unit_y, norm_x, norm_y;
	vector<CvPoint>midpoints;
	CvScalar color;
	
	//set map to zero
	//cvSetZero(map);
	
	//debugging code
	cvCircle(vMap, axle.tires[LEFT].center, 2, cvScalar(125), 2);
	cvCircle(vMap, axle.tires[RIGHT].center, 2, cvScalar(125), 2);
	cvLine(vMap, axle.tires[LEFT].center, axle.tires[RIGHT].center, cvScalar(125), 1);

	//stores tire location before the tire steps
	CvPoint* initial = new CvPoint[2], after_step;
	
	//main algorithm that searches for the lines in the image and creates a vision map
	while( dist_to_last < 125 )
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
			
			//cout<<"****STEPPING BOTH****"<<endl;
			
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
					
					//check to see if the axle is below the minimum width
					if( ((axle.tires[LEFT].center.x - axle.tires[RIGHT].center.x)*	
						 (axle.tires[LEFT].center.x - axle.tires[RIGHT].center.x) +
						 (axle.tires[LEFT].center.y - axle.tires[RIGHT].center.y)*
						 (axle.tires[LEFT].center.y - axle.tires[RIGHT].center.y)) < 750 )
					{
						int heading_diff = axle.tires[LEFT].heading - axle.tires[RIGHT].heading;
						axle.tires[LEFT].heading -= heading_diff/2.;
						axle.tires[RIGHT].heading += heading_diff/2.;
					}
			
					//debugging code
					cvCircle(vMap, axle.tires[LEFT].center, 3, cvScalar(0,128,0), 2);
					cvCircle(vMap, axle.tires[RIGHT].center, 3, cvScalar(0,128,0), 2);
				}
			}
		}
		else
		{
			//determine the primary and secondary tires
			if(axle.state == 0)
			{
				//cout<<"****STEPPING LEFT****\n";
				primary   = 0;
				secondary = 1;
				color     = cvScalar(0,255,0);
				norm      = -90;
			}
			else
			{
				//cout<<"***STEPPING RIGHT***\n";
				primary   = 1;
				secondary = 0;
				color     = cvScalar(0, 0, 255);
				norm      = 90;
			}
			// **** 1. Take a step with the primary wheel **** /
			after_step.x = axle.tires[primary].center.y = initial[primary].y - round(bigStep*cos(axle.tires[primary].heading*PI/180.));
			after_step.y = axle.tires[primary].center.x = initial[primary].x - round(bigStep*sin(axle.tires[primary].heading*PI/180.));
			
			//cout<<"Before: "<<axle.tires[primary].center.x<<" "<<axle.tires[primary].center.y<<" "<<axle.tires[primary].heading<<" State: "<<axle.tires[primary].state<<endl;
			// **** 2. Optimize using the gradient descent method **** //
			gradientDescent(image, &axle.tires[primary]);
			//cout<<"After: "<<axle.tires[primary].center.x<<" "<<axle.tires[primary].center.y<<" "<<axle.tires[primary].heading<<" State: "<<axle.tires[primary].state<<endl;
			
			//check to make sure that the tire is not going back on previous positions
			bool stepped_on_previous = false;
			for(int i = 0; i<5; i++){
				double mag = (previous[primary][i].x-axle.tires[primary].center.x)*
							 (previous[primary][i].x-axle.tires[primary].center.x)+
							 (previous[primary][i].y-axle.tires[primary].center.y)*
							 (previous[primary][i].y-axle.tires[primary].center.y);
							 
				if( mag < 3 ){
					//cout<<"Reset the position!"<<endl;
					axle.tires[primary].center = after_step;
					stepped_on_previous = true;
					break;
				}
			}		
			
			//check to make sure the threshold is met
			if( axle.tires[primary].sum <THRESHOLD || stepped_on_previous)
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
				//store current in the previous
				previous[primary][index[primary]]  = axle.tires[primary].center;
				index[primary]++;
				if(index[primary] >= 5)
					index[primary] = 0;
				
				// **** 3. Draw line on the final image **** //
				//Draw line on the map from current to last known
				cvLine(map, axle.tires[primary].center, axle.tires[primary].last, cvScalar(255), map_line_width);
				cvLine(vMap, axle.tires[primary].last, axle.tires[primary].center, cvScalar(255, 0, 0), 3);
				//set last know to the current point
				axle.tires[primary].last = axle.tires[primary].center;
				
				// **** 4. Translate the secondary wheel **** //
				//form m*x + b lines from normal of primary and heading of secondary
				unit_x = -sin(axle.tires[secondary].heading*PI/180.);
				unit_y = -cos(axle.tires[secondary].heading*PI/180.);
				Coefficients minor = genLine(axle.tires[secondary].center.x,
											 axle.tires[secondary].center.y,
											 axle.tires[secondary].center.x + unit_x,
											 axle.tires[secondary].center.y + unit_y );
				
				norm_x = -sin((axle.tires[primary].heading+norm)*PI/180.);
				norm_y = -cos((axle.tires[primary].heading+norm)*PI/180.);
				Coefficients main = genLine(axle.tires[primary].center.x, 
											axle.tires[primary].center.y, 
											axle.tires[primary].center.x + norm_x, 
											axle.tires[primary].center.y + norm_y );
				
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
						//push midpoint into vector
						midpoints.push_back(cvPoint(axle.tires[primary].center.x - (axle.tires[primary].center.x - axle.tires[secondary].center.x)/2., axle.tires[primary].center.y - (axle.tires[primary].center.y - axle.tires[secondary].center.y)/2.));
						//switch the axle state
						axle.state = secondary;
						
						//draw good line on final map
						cvLine(map, axle.tires[secondary].last, axle.tires[secondary].center, cvScalar(255), map_line_width);
						cvLine(vMap, axle.tires[secondary].last, axle.tires[secondary].center, cvScalar(255, 0, 0), 3);
						//set last known to current center
						axle.tires[secondary].last = axle.tires[secondary].center;
						
						//debugging code
						cvCircle(vMap, axle.tires[LEFT].center, 2, color, 2);
						cvCircle(vMap, axle.tires[RIGHT].center, 2, color, 2);
						cvLine(vMap, axle.tires[LEFT].center, axle.tires[RIGHT].center, color, 1);
						
						//find the angle between primary heading and tangent line
						angle_orig = dotproduct(axle.tires, primary);
						//cout<<"Before DP: "<<angle_orig<<endl;
						for(int step=0; step<3; step++)
						{
							//set initial location to current center
							initial[secondary] = axle.tires[secondary].center;
							
							//take a step with the secondary wheel
							after_step.x = axle.tires[secondary].center.y = initial[secondary].y - bigStep*cos(axle.tires[secondary].heading*PI/180.);
							after_step.y = axle.tires[secondary].center.x = initial[secondary].x - bigStep*sin(axle.tires[secondary].heading*PI/180.);
							
							//optimize at current location
							gradientDescent(image, &axle.tires[secondary]);
							
							/*stepped_on_previous = false;
							for(int i = 0; i<5; i++){
								double mag = (previous[secondary][i].x-axle.tires[secondary].center.x)*
											 (previous[secondary][i].x-axle.tires[secondary].center.x)+
											 (previous[secondary][i].y-axle.tires[secondary].center.y)*
											 (previous[secondary][i].y-axle.tires[secondary].center.y);
											 
								if( mag < 3 ){
									//cout<<"Reset the position!"<<endl;
									axle.tires[secondary].center = after_step;
									stepped_on_previous = true;
									break;
								}
							}	*/
							
							//check to make sure the threshold is met
							if(axle.tires[secondary].sum < THRESHOLD )//|| stepped_on_previous)
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
								break;
							}
							else
							{	
								//store current in the previous
								//previous[secondary][index[secondary]]  = axle.tires[secondary].center;
								//index[secondary]++;
								//if(index[secondary] >= 5)
								//	index[secondary] = 0;
								
								//recalculate the angle between the wheels
								angle_new = dotproduct(axle.tires, primary);
								//cout<<"Step and New DP: "<<angle_new<<endl;
								if(angle_new < angle_orig)
								{
									//set the new difference
									angle_orig = angle_new;
									
									//draw in the good line on the map
									cvLine(map, axle.tires[secondary].last, axle.tires[secondary].center, cvScalar(255), map_line_width);
									cvLine(vMap, axle.tires[secondary].last, axle.tires[secondary].center, cvScalar(255, 0, 0), 3);
									//set last known to current center
									axle.tires[secondary].last = axle.tires[secondary].center;
									
									//pop back old point
									midpoints.pop_back();
									//push in new midpoint
									midpoints.push_back(cvPoint(axle.tires[primary].center.x - (axle.tires[primary].center.x - axle.tires[secondary].center.x)/2., axle.tires[primary].center.y - (axle.tires[primary].center.y - axle.tires[secondary].center.y)/2.));
									
									//debugging code
									cvCircle(vMap, axle.tires[secondary].center, 2, color, 2);
									cvLine(vMap, axle.tires[LEFT].center, axle.tires[RIGHT].center, color, 1);
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
					cvCircle(vMap, axle.tires[LEFT].center, 2, color, 2);
					cvCircle(vMap, axle.tires[RIGHT].center, 2, color, 2);
					cvLine(vMap, axle.tires[LEFT].center, axle.tires[RIGHT].center, color, 1);
				}
			}
		}
		//cout<<"Left: "<<axle.tires[LEFT].center.x<<" "<<axle.tires[LEFT].center.y<<" "<<axle.tires[LEFT].heading<<" State: "<<axle.tires[LEFT].state<<endl;
		//cout<<"Right: "<<axle.tires[RIGHT].center.x<<" "<<axle.tires[RIGHT].center.y<<" "<<axle.tires[RIGHT].heading<<" State: "<<axle.tires[RIGHT].state<<endl<<endl;
		//cvSetImageROI(vMap, cvRect(axle.tires[LEFT].center.x - 250, axle.tires[LEFT].center.y - 250, 500, 500));
		//cvShowImage("Lines", map);
		//if(axle.tires[RIGHT].center.x==637 && axle.tires[RIGHT].center.y==1293){
			//cvShowImage("Orig", vMap);
			//while((cvWaitKey(10)&255)!=32);
			//cvWaitKey(10);
		//}
		//cvResetImageROI(vMap);
	}
	return midpoints;
}
				

Line::~Line()
{
	cvReleaseImage(&line_mask);
	cvReleaseImage(&no_line_mask);
	cvReleaseImage(&output);
}
