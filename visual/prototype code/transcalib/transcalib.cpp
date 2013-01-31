//Call:
//  birds-eye board_w board_h instrinics distortion image_file
// ADJUST VIEW HEIGHT using keys 'u' up, 'd' down. ESC to quit.
//
#include <stdio.h>
#include "cv.h"
#include "highgui.h"
int main(int argc, char* argv[]) {
  if(argc != 6){
	printf("too few args\n");
	return -1;
  }
  // INPUT PARAMETERS:
  //
  int       board_w    = atoi(argv[1]);
  int       board_h    = atoi(argv[2]);
  int       board_n    = board_w * board_h;
  CvSize    board_sz   = cvSize( board_w, board_h );
  CvMat*    intrinsic  = (CvMat*)cvLoad(argv[3]);
  CvMat*    distortion = (CvMat*)cvLoad(argv[4]);
  IplImage* image      = 0;
  IplImage* gray_image = 0;
  if( (image = cvLoadImage(argv[5])) == 0 ) {
    printf("Error: Couldn't load %s\n",argv[5]);
    return -1;
  }
  
  CvMat* image_points      = cvCreateMat(1*board_n,2,CV_32FC1);
  CvMat* object_points     = cvCreateMat(1*board_n,3,CV_32FC1);
  
  CvMat* objdrawpoints = cvCreateMat(1,1,CV_32FC3);
  CvMat* imgdrawpoints = cvCreateMat(1,1,CV_32FC2);
  float x=0;
  float y=0;
  float z=0;
  
  double grid_width=2.85;
  gray_image = cvCreateImage( cvGetSize(image), 8, 1 );
  cvCvtColor(image, gray_image, CV_BGR2GRAY );

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
	printf("Couldn't aquire chessboard on %s, "
	  "only found %d of %d corners\n",
	  argv[5],corner_count,board_n
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

// If we got a good board, add it to our data
  for( int i=0, j=0; j<board_n; ++i,++j ) {
	CV_MAT_ELEM(*image_points, float,i,0) = corners[j].x;
	CV_MAT_ELEM(*image_points, float,i,1) = corners[j].y;
	CV_MAT_ELEM(*object_points,float,i,0) =grid_width*( j/board_w);
	//  cout<<j/board_w<<" "<<j%board_w<<endl;
	CV_MAT_ELEM(*object_points,float,i,1) = grid_width*(j%board_w);
	CV_MAT_ELEM(*object_points,float,i,2) = 0.0f;
  }

  // DRAW THE FOUND CHESSBOARD
  //
  cvDrawChessboardCorners(
	image,
	board_sz,
	corners,
	corner_count,
	found
  );

  // FIND THE HOMOGRAPHY
  //
  CvMat *trans = cvCreateMat( 1, 3, CV_32F);
  CvMat *rot = cvCreateMat( 1, 3, CV_32F);

  // LET THE USER ADJUST THE Z HEIGHT OF THE VIEW
  //
  cvFindExtrinsicCameraParams2(object_points,image_points,intrinsic,distortion,rot,trans);
  
//  cvSave("trans.xml",trans); 
//  cvSave("rot.xml",rot); 
  int key = 0;
  IplImage *drawn_image = cvCloneImage(image);
  cvNamedWindow("translation");

  // LOOP TO ALLOW USER TO PLAY WITH HEIGHT:
  //
  // escape key stops
  //
  
//  cvSetZero(trans);
//  cvSetZero(rot);
  while(key != 27) {
	cvCopy(image,drawn_image);
	
	if(key==97)x--;
	else if(key==113)x++;
	else if(key==115)y--;
	else if(key==119)y++;
	else if(key==100)z--;
	else if(key==101)z++;
	
	((float*)(objdrawpoints->data.ptr))[0]=x;
	((float*)(objdrawpoints->data.ptr))[1]=y;
	((float*)(objdrawpoints->data.ptr))[2]=z;
	printf("%f %f %f\n",x,y,z);
	cvProjectPoints2(objdrawpoints,rot,trans,intrinsic,distortion,imgdrawpoints);
	cvCircle(drawn_image,cvPoint(((float*)(imgdrawpoints->data.ptr))[0],((float*)(imgdrawpoints->data.ptr))[1]),5,cvScalar(255,0,0),-1);
	printf("%f %f\n",((float*)(imgdrawpoints->data.ptr))[0],((float*)(imgdrawpoints->data.ptr))[1]);
	cvShowImage( "translation", drawn_image );
	key = cvWaitKey(3);
  }
  cvDestroyWindow( "translation" );
  //must add a lot of memory releasing here
  return 0;
}


