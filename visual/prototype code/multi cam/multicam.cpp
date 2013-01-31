#include <stdio.h>
#include <time.h>
#include "cv.h"
#include "highgui.h"

int main( int argc, char** argv )
{
  CvCapture* capture0 = cvCaptureFromCAM( 0 );
  CvCapture* capture1 = cvCaptureFromCAM( 1 );
  if( !capture1 || !capture0) {
	fprintf( stderr, "ERROR: capture is NULL \n" );
	getchar();
	return -1;
  }
  
  
  IplImage* frame0 = cvQueryFrame( capture0 );
 // IplImage* frame1 = cvQueryFrame( capture1 );
  IplImage* frame1=NULL;
  
  /* create a window */ 
  cvNamedWindow( "img0", CV_WINDOW_AUTOSIZE );   
  cvNamedWindow( "img1", CV_WINDOW_AUTOSIZE );   
  
  IplImage *temp_bw=cvCreateImage(cvSize(frame0->width,frame0->height),8,1);
  IplImage *colored_image=cvCreateImage(cvSize(frame0->width,frame0->height),8,3);
  IplImage *temp_bw1=cvCreateImage(cvSize(frame0->width,frame0->height),8,1);
  IplImage *colored_image1=cvCreateImage(cvSize(frame0->width,frame0->height),8,3);
  
  // Show the image captured from the camera in the window and repeat
  while( 1 ) {
	// Get one frame
	frame0 = cvQueryFrame( capture0 );
	cvCopy(frame0,colored_image);
	
	frame1 = cvQueryFrame( capture1 );
	cvCopy(frame1,colored_image1);
	
	if( !frame0 ||!frame1) {
	  fprintf( stderr, "ERROR: frame is null...\n" );
	  printf( "ERROR: frame is null...\n" );
	  getchar();
	  break;
	}
	//make the camera frame have 1 color layer
//	cvCvtColor(frame0,temp_bw,CV_RGB2GRAY);
	//convert the camera frame to color
//	cvCvtColor(temp_bw,colored_image,CV_BayerGB2RGB);
	
	/* display the image */  
//	cvShowImage( "img0", colored_image );
	cvShowImage( "img0", colored_image );
	//make the camera frame have 1 color layer
//	cvCvtColor(frame1,temp_bw1,CV_RGB2GRAY);
	//convert the camera frame to color
//	cvCvtColor(temp_bw1,colored_image1,CV_BayerGB2RGB);
	
	/* display the image */  
//	cvShowImage( "img1", colored_image1 );
	
	cvShowImage( "img1", colored_image1 );
	
	//If ESC key pressed, Key=0x10001B under OpenCV 0.9.7(linux version),
	//remove higher bits using AND operator
 	int key=cvWaitKey(3); 
	printf("weee, %d\n",key);
	if( (key & 255) == 27 ) break;
	
  }
  
  /* free memory */
  cvDestroyWindow( "img0" );
  cvDestroyWindow( "img1" );
  cvReleaseImage( &colored_image );
  cvReleaseImage( &temp_bw );
  
  cvReleaseImage( &colored_image1 );
  cvReleaseImage( &temp_bw1);
  
  cvReleaseCapture( &capture0 );
  cvReleaseCapture( &capture1 );
  return 0;
}

