// intrinsiccalibrator.cpp
// Calling convention:
// calib board_w board_h number_of_views
//
// Hit 'p' to pause/unpause, ESC to quit
//

//changes were made to this file that are yet untested
//i made the find chessboard corners 
#include <cv.h>
#include <highgui.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include "intrinsiccalibrator.h"

const int board_dt = 20; //Wait 20 frames per chessboard view

using namespace std;

int intrinsiccalibrator(int board_w, int  board_h, int n_boards, float cm_grid_width) {
  
  int board_n  = board_w * board_h;
  CvSize board_sz = cvSize( board_w, board_h );
  CvCapture* capture = cvCreateCameraCapture( 0 );
  assert( capture );
  float grid_width=cm_grid_width;
  
  cvNamedWindow( "Calibration" );
  //ALLOCATE STORAGE
  CvMat* image_points      = cvCreateMat(n_boards*board_n,2,CV_64FC1);
  CvMat* object_points     = cvCreateMat(n_boards*board_n,3,CV_64FC1);
  CvMat* point_counts      = cvCreateMat(n_boards,1,CV_32SC1);
  CvMat* intrinsic_matrix  = cvCreateMat(3,3,CV_64FC1);
  CvMat* distortion_coeffs = cvCreateMat(5,1,CV_64FC1);
  
  CvPoint2D32f* corners = new CvPoint2D32f[ board_n ];
  int corner_count;
  int successes = 0;
  int step, frame = 0;
  IplImage *image = cvQueryFrame( capture );
  IplImage *temp_bw=cvCreateImage(cvSize(image->width,image->height),8,1);
  IplImage *gray_image = cvCreateImage(cvGetSize(image),8,1);//subpixel
  
  //make the camera frame have 1 color layer
  cvCvtColor(image,temp_bw,CV_RGB2GRAY);
  //convert the camera frame to color
  cvCvtColor(temp_bw,image,CV_BayerGB2RGB);
  
  cvCvtColor(image,gray_image,CV_RGB2GRAY);
  
  // CAPTURE CORNER VIEWS LOOP UNTIL WE'VE GOT n_boards
  // SUCCESSFUL CAPTURES (ALL CORNERS ON THE BOARD ARE FOUND)
  //
  while(successes < n_boards) {
	
	//Skip every board_dt frames to allow user to move chessboard
	if(frame++ % board_dt == 0) {
	  //Find chessboard corners:
	  int found = cvFindChessboardCorners(
	  gray_image, board_sz, corners, &corner_count,CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
	  
	  cvFindCornerSubPix(gray_image, corners, corner_count,
						 cvSize(11,11),cvSize(-1,-1), cvTermCriteria(
						 CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
						 
	  //Draw it
	  cvDrawChessboardCorners(image, board_sz, corners,
							  corner_count, found);
							  cvShowImage( "Calibration", image );
							  
	  // If we got a good board, add it to our data
	  if( corner_count == board_n ) {
		step = successes*board_n;
		for( int i=step, j=0; j<board_n; ++i,++j ) {
		  CV_MAT_ELEM(*image_points, float,i,0) = corners[j].x;
		  CV_MAT_ELEM(*image_points, float,i,1) = corners[j].y;
		  CV_MAT_ELEM(*object_points,float,i,0) =grid_width*( j/board_w);
		  //  cout<<j/board_w<<" "<<j%board_w<<endl;
		  CV_MAT_ELEM(*object_points,float,i,1) = grid_width*(j%board_w);
		  CV_MAT_ELEM(*object_points,float,i,2) = 0.0f;
		}
		CV_MAT_ELEM(*point_counts, int,successes,0) = board_n;
		successes++;
	  }
	} //end skip board_dt between chessboard capture
	
	//Handle pause/unpause and ESC
	int c = cvWaitKey(15);
	if(c == 'p'){
	  c = 0;
	  while(c != 'p' && c != 27){
		c = cvWaitKey(250);
	  }
	}
	if(c == 27)
	  return 0;
	image = cvQueryFrame( capture ); //Get next image
	//make the camera frame have 1 color layer
	cvCvtColor(image,temp_bw,CV_RGB2GRAY);
	//convert the camera frame to color
	cvCvtColor(temp_bw,image,CV_BayerGB2RGB);
	//store a true black and white version of it
	cvCvtColor(image,gray_image,CV_RGB2GRAY);
	
	
  } //END COLLECTION WHILE LOOP.
  
  //ALLOCATE MATRICES ACCORDING TO HOW MANY CHESSBOARDS FOUND
  CvMat* object_points2   =  cvCreateMat(successes*board_n,3,CV_32FC1);
  CvMat* image_points2    =  cvCreateMat(successes*board_n,2,CV_32FC1);
  CvMat* point_counts2    =  cvCreateMat(successes,1,CV_32SC1);
  //TRANSFER THE POINTS INTO THE CORRECT SIZE MATRICES
  //Below, we write out the details in the next two loops. We could
  //instead have written:
  //image_points->rows = object_points->rows = \
  //successes*board_n; point_counts->rows = successes;
  //
  for(int i = 0; i<successes*board_n; ++i) {
	CV_MAT_ELEM( *image_points2, float, i, 0) =
	CV_MAT_ELEM( *image_points, float, i, 0);
	CV_MAT_ELEM( *image_points2, float,i,1) =
	CV_MAT_ELEM( *image_points, float, i, 1);
	CV_MAT_ELEM(*object_points2, float, i, 0) =
	CV_MAT_ELEM( *object_points, float, i, 0) ;
	CV_MAT_ELEM( *object_points2, float, i, 1) =
	CV_MAT_ELEM( *object_points, float, i, 1) ;
	CV_MAT_ELEM( *object_points2, float, i, 2) =
	CV_MAT_ELEM( *object_points, float, i, 2) ;
  }
  for(int i=0; i<successes; ++i){ //These are all the same number
	CV_MAT_ELEM( *point_counts2, int, i, 0) =
	CV_MAT_ELEM( *point_counts, int, i, 0);
  }
  cvReleaseMat(&object_points);
  cvReleaseMat(&image_points);
  cvReleaseMat(&point_counts);
  
  // At this point we have all of the chessboard corners we need.
  // Initialize the intrinsic matrix such that the two focal
  // lengths have a ratio of 1.0
  //
  CV_MAT_ELEM( *intrinsic_matrix, float, 0, 0 ) = 1.0f;
  CV_MAT_ELEM( *intrinsic_matrix, float, 1, 1 ) = 1.0f;
  
  //CALIBRATE THE CAMERA!
  cvCalibrateCamera2(object_points2, image_points2,
					 point_counts2, cvGetSize( image ),
					 intrinsic_matrix, distortion_coeffs,
					 NULL, NULL,0 //CV_CALIB_FIX_ASPECT_RATIO
					 );
					 
  // SAVE THE INTRINSICS AND DISTORTIONS
  cvSave("vision/calibration/Intrinsic.xml",intrinsic_matrix);
  cvSave("vision/calibration/Distortion.xml",distortion_coeffs);
  // EXAMPLE OF LOADING THESE MATRICES BACK IN:
  CvMat *intrinsic = (CvMat*)cvLoad("vision/calibration/Intrinsic.xml");
  CvMat *distortion = (CvMat*)cvLoad("vision/calibration/Distortion.xml");
  
  // Build the undistort map that we will use for all
  // subsequent frames.
  //
  IplImage* mapx = cvCreateImage( cvGetSize(image), IPL_DEPTH_32F, 1 );
  IplImage* mapy = cvCreateImage( cvGetSize(image), IPL_DEPTH_32F, 1 );
  cvInitUndistortMap(intrinsic,
					distortion,
					mapx,
					mapy
					);
  // Just run the camera to the screen, now showing the raw and
  // the undistorted image.
  //
  cvNamedWindow( "Undistort" );
  IplImage *t = cvCloneImage(image);
  while(image) {
	cvCopy(image,t);
	//t = cvCloneImage(image);
	cvShowImage( "Calibration", image ); // Show raw image
	cvRemap( t, image, mapx, mapy );     // Undistort image
	cvShowImage("Undistort", image);     // Show corrected image
	
	//Handle pause/unpause and ESC
	int c = cvWaitKey(15);
	if(c == 'p') {
	  c = 0;
	  while(c != 'p' && c != 27) {
		c = cvWaitKey(250);
	  }
	}
	if((c&255) == 27)
	  break;
	image = cvQueryFrame( capture );
	//make the camera frame have 1 color layer
	cvCvtColor(image,temp_bw,CV_RGB2GRAY);
	//convert the camera frame to color
	cvCvtColor(temp_bw,image,CV_BayerGB2RGB);
  }
  
  cvReleaseImage(&temp_bw); 
  cvReleaseImage(&t); 
  cvReleaseImage(&gray_image); 
  return 0;
}


