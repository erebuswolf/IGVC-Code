//Call:
//  birds-eye board_w board_h instrinics distortion image_file
// ADJUST VIEW HEIGHT using keys 'u' up, 'd' down. ESC to quit.
//
#include <stdio.h>
#include "cv.h"
#include "highgui.h"
int main(int argc, char* argv[]) {
  if(argc != 4){
   printf("too few args\n");
   return -1;
	}
  // INPUT PARAMETERS:
  //
  CvMat*    intrinsic  = (CvMat*)cvLoad(argv[1]);
  CvMat*    distortion = (CvMat*)cvLoad(argv[2]);
  CvMat*    H = (CvMat*)cvLoad(argv[3]);
  
	CvCapture* capture = cvCaptureFromCAM( CV_CAP_ANY );
  if( !capture ) {
    fprintf( stderr, "ERROR: capture is NULL \n" );
    getchar();
    return -1;
  }


	// LET THE USER ADJUST THE Z HEIGHT OF THE VIEW
	//
	
	float Z = 25;
	int key = 0;	
	
	cvNamedWindow("Birds_Eye");

	IplImage *image = cvQueryFrame( capture );
	IplImage *temp_bw=cvCreateImage(cvSize(image->width,image->height),8,1);
	IplImage *birds_image=cvCloneImage(image);
	
	// LOOP TO ALLOW USER TO PLAY WITH HEIGHT:
	//
	// escape key stops
	//
	while(key != 27) {
		image = cvQueryFrame( capture );
		//make the camera frame have 1 color layer
		cvCvtColor(image,temp_bw,CV_RGB2GRAY);
		//convert the camera frame to color
		cvCvtColor(temp_bw,image,CV_BayerGB2RGB);
		
		cvCopy( image, birds_image);
		// Set the height
		//
		CV_MAT_ELEM(*H,float,2,2) = Z;

		// COMPUTE THE FRONTAL PARALLEL OR BIRD'S-EYE VIEW:
		// USING HOMOGRAPHY TO REMAP THE VIEW
		//
		cvWarpPerspective(
		  image,
		  birds_image,
		  H,
		  CV_INTER_LINEAR | CV_WARP_INVERSE_MAP | CV_WARP_FILL_OUTLIERS
		);
		cvShowImage( "Birds_Eye", birds_image );

	  key = cvWaitKey(2);
	  if(key == 'u') Z += 0.5;
	  if(key == 'd') Z -= 0.5;
  }
	cvReleaseImage(&birds_image);
  cvReleaseImage(&temp_bw);
  return 0;
}
