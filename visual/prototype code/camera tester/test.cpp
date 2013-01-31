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
    if( !frame ) {
      fprintf( stderr, "ERROR: frame is null...\n" );
      getchar();
      break;
    }

    cvShowImage( "mywindow", frame );
    // Do not release the frame!

    //If ESC key pressed, Key=0x10001B under OpenCV 0.9.7(linux version),
    //remove higher bits using AND operator
    int key=cvWaitKey(10);
    if( (key & 255) == 27 ) break;
    else if((key & 255)==32) {
			IplImage* img = 0; 
			
			img=cvQueryFrame(capture);           // retrieve the captured frame
			sprintf (buffer, "test%d.jpg", picturecount++);

			if(!cvSaveImage(buffer,img,0)) printf("Could not save: %s\n",buffer);
			else printf("picture taken!!!\n");
    }
    
  }


	
  // Release the capture device housekeeping
  cvReleaseCapture( &capture );
  cvDestroyWindow( "mywindow" );
  return 0;
}
