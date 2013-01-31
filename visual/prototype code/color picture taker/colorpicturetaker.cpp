#include "cv.h"
#include "highgui.h"
#include <stdio.h>

// A Simple Camera Capture Framework
int main() {

  CvCapture* capture = cvCaptureFromCAM( CV_CAP_ANY );
  if( !capture ) {
    fprintf( stderr, "ERROR: capture is NULL \n" );
    getchar();
    return -1;
  }

  // Create a window in which the captured images will be presented
  cvNamedWindow( "mywindow", CV_WINDOW_AUTOSIZE );


	int picturecount=1;

  char buffer [50];

  // Show the image captured from the camera in the window and repeat
  while( 1 ) {
    // Get one frame
    IplImage* frame = cvQueryFrame( capture );
		IplImage *temp_bw=cvCreateImage(cvSize(frame->width,frame->height),8,1);
		IplImage *colored_image=cvCreateImage(cvSize(frame->width,frame->height),8,3);
		//make the camera frame have 1 color layer
		cvCvtColor(frame,temp_bw,CV_RGB2GRAY);
		//convert the camera frame to color
		cvCvtColor(temp_bw,colored_image,CV_BayerGB2RGB);
		
    if( !frame ) {
      fprintf( stderr, "ERROR: frame is null...\n" );
      getchar();
      break;
    }

    cvShowImage( "mywindow", colored_image );
    // Do not release the frame!

    //If ESC key pressed, Key=0x10001B under OpenCV 0.9.7(linux version),
    //remove higher bits using AND operator
    int key=cvWaitKey(10);
    if( (key & 255) == 27 ) break;
    else if((key & 255)==32) {
			sprintf (buffer, "test%02d.jpg", picturecount++);
			if(!cvSaveImage(buffer,colored_image,0)) printf("Could not save: %s\n",buffer);
			else printf("picture taken!!!\n");
    }
    cvReleaseImage(&colored_image);
  }


	
  // Release the capture device housekeeping
  cvReleaseCapture( &capture );
  cvDestroyWindow( "mywindow" );
  return 0;
}
