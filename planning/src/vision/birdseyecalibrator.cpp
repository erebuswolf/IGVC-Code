//Call:
//  birds-eye board_w board_h instrinics distortion image_file
// this assumes we use a calibration board with squares of 11.1 cm and put it 8 feet in front of the robot
//
#include <stdio.h>
#include "cv.h"
#include "highgui.h"
#include "Camera.h"
#include "birdseyecalibrator.h"

int birdseyecalibrator(int board_w, int board_h, const char * intrinsic_path, const char * distortion_path, float cm_grid_width, float cm_per_pixel, float cm_in_front_of_robot, int image_width, int image_height) {  
  
  int       board_n    = board_w * board_h;
  CvSize    board_sz   = cvSize( board_w, board_h );
  CvMat*    intrinsic  = (CvMat*)cvLoad(intrinsic_path);
  CvMat*    distortion = (CvMat*)cvLoad(distortion_path);
  
  Camera cam;
  cam.init(0);
  cam.startThread();
  
  
  cvNamedWindow("Chessboard");
  cam.updateImage();
  cam.colorizeImage();
  
  IplImage* image      = cvCloneImage(cam.image);
  IplImage* gray_image = 0;
  
  
  float grid_width=cm_grid_width/cm_per_pixel;
  float xoffset=image_width/2.;
  float yoffset=image_height/2.-cm_in_front_of_robot/cm_per_pixel;
  
  
  printf("press space when you like the image for calibration\n");

  int key = 0;
  while(key<=0){
	cam.updateImage();
	cam.colorizeImage();
	
	cvCopy(cam.image,image);
	cvShowImage( "Chessboard", image );
	
	key = cvWaitKey(3);
  }
  
  
  //take image and make it black and white
  gray_image = cvCreateImage( cvGetSize(image), 8, 1 );
  cvCvtColor(image, gray_image, CV_BGR2GRAY );

  // UNDISTORT OUR IMAGE
  //
  IplImage* mapx = cvCreateImage( cvGetSize(image), IPL_DEPTH_32F, 1 );
  IplImage* mapy = cvCreateImage( cvGetSize(image), IPL_DEPTH_32F, 1 );
  //This initializes rectification matrices
  //
  cvInitUndistortMap(
	intrinsic,
	distortion,
	mapx,
	mapy
  );
  IplImage *t = cvCloneImage(gray_image);
  // Rectify our image
  //
  cvRemap( t, gray_image, mapx, mapy );


  IplImage *t2 = cvCloneImage(image);
  // Rectify our image
  //
  cvRemap( t2, image, mapx, mapy );

  // GET THE CHESSBOARD ON THE PLANE
  //
  CvPoint2D32f* corners = new CvPoint2D32f[ board_n ];
  int corner_count = 0;
  int found = cvFindChessboardCorners(
	gray_image,
	board_sz,
	corners,
	&corner_count,
	CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS
  );
  
  if(!found){
	printf("Couldn't aquire chessboard on image, "
	  "only found %d of %d corners\n",corner_count,board_n
	);
	return -1;
  }
  
  //Get Subpixel accuracy on those corners:
  cvFindCornerSubPix(
	gray_image,
	corners,
	corner_count,
	cvSize(11,11),
	cvSize(-1,-1),
	cvTermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1 )
  );

  //GET THE IMAGE AND OBJECT POINTS:
  // We will choose chessboard object points as (r,c):
  // (0,0), (board_w-1,0), (0,board_h-1), (board_w-1,board_h-1).
  //
  
  //this was changed to add actual units and measurements to the values
  CvPoint2D32f objPts[4], imgPts[4];
  objPts[0].x = 0+xoffset;          objPts[0].y = 0+yoffset;
  objPts[1].x = grid_width*(board_w-1)+xoffset;  objPts[1].y = 0+yoffset;
  objPts[2].x = 0+xoffset;          objPts[2].y = grid_width*(board_h-1)+yoffset;
  objPts[3].x = grid_width*(board_w-1)+xoffset;  objPts[3].y = grid_width*(board_h-1)+yoffset;
  imgPts[0]   = corners[0];
  imgPts[1]   = corners[board_w-1];
  imgPts[2]   = corners[(board_h-1)*board_w];
  imgPts[3]   = corners[(board_h-1)*board_w + board_w-1];

  // DRAW THE POINTS in order: B,G,R,YELLOW
  //
  cvCircle( image, cvPointFrom32f(imgPts[0]), 9, CV_RGB(0,0,255),   3);
  cvCircle( image, cvPointFrom32f(imgPts[1]), 9, CV_RGB(0,255,0),   3);
  cvCircle( image, cvPointFrom32f(imgPts[2]), 9, CV_RGB(255,0,0),   3);
  cvCircle( image, cvPointFrom32f(imgPts[3]), 9, CV_RGB(255,255,0), 3);

  // DRAW THE FOUND CHESSBOARD
  //
  cvDrawChessboardCorners(
	image,
	board_sz,
	corners,
	corner_count,
	found
  );
  cvShowImage( "Chessboard", image );

  // FIND THE HOMOGRAPHY
  //
  CvMat *H = cvCreateMat( 3, 3, CV_32F);
  cvGetPerspectiveTransform( objPts, imgPts, H);

  // LET THE USER ADJUST THE Z HEIGHT OF THE VIEW
  //
  //float Z = 5.5 ;
  IplImage *birds_image = cvCreateImage( cvSize(image_width,image_height), 8, 3 );
  cvNamedWindow("Birds_Eye"); 

  // LOOP TO ALLOW USER TO PLAY WITH HEIGHT:
  //
  // escape key stops
  //
  
  while((key&255) != 27) {
	cam.updateImage();
	cam.colorizeImage();
	cvCopy(cam.image,image);
	
	cvRemap(image,t2, mapx, mapy );

	// COMPUTE THE FRONTAL PARALLEL OR BIRD'S-EYE VIEW:
	// USING HOMOGRAPHY TO REMAP THE VIEW
	//
	cvWarpPerspective(
	t2,
	birds_image,
	H,
	CV_INTER_LINEAR | CV_WARP_INVERSE_MAP | CV_WARP_FILL_OUTLIERS
	);
	cvShowImage( "Birds_Eye", birds_image );

	key = cvWaitKey(10);
  }
  
  //this should be fixed when we get to stereo vision
  cvSave("vision/calibration/H.xml",H); //We can reuse H for the same camera mounting
  return 0;
}
