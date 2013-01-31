#include <stdio.h>
#include <time.h>
#include "cv.h"
#include "highgui.h"

int main( int argc, char** argv )
{
	
	CvCapture* capture = cvCaptureFromCAM( CV_CAP_ANY );
  if( !capture ) {
    fprintf( stderr, "ERROR: capture is NULL \n" );
    getchar();
    return -1;
  }
  
  
	IplImage* frame = cvQueryFrame( capture );
	
	/* create a window */ 
	cvNamedWindow( "Colored image", CV_WINDOW_AUTOSIZE );   
	
	IplImage *temp_bw=cvCreateImage(cvSize(frame->width,frame->height),8,1);
	IplImage *colored_image=cvCreateImage(cvSize(frame->width,frame->height),8,3);

	
	// Show the image captured from the camera in the window and repeat
  while( 1 ) {
  
    // Get one frame
    IplImage* frame = cvQueryFrame( capture );
    if( !frame ) {
      fprintf( stderr, "ERROR: frame is null...\n" );
      getchar();
      break;
    }
    
    //make the camera frame have 1 color layer
		cvCvtColor(frame,temp_bw,CV_RGB2GRAY);
		//convert the camera frame to color
		cvCvtColor(temp_bw,colored_image,CV_BayerGB2RGB);


		/* display the image */  
		cvShowImage( "Colored image", colored_image );

   
    // Do not release the frame!
		
    //If ESC key pressed, Key=0x10001B under OpenCV 0.9.7(linux version),
    //remove higher bits using AND operator
    int key=cvWaitKey(2);   
    if( (key & 255) == 27 ) break;
    
  }


	/* free memory */
	cvDestroyWindow( "Colored image" );
	cvReleaseImage( &colored_image );
	
	
  cvReleaseCapture( &capture );
	return 0;
}

