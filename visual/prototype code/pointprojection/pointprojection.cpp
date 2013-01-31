//Call:
//  birds-eye board_w board_h instrinics distortion image_file
// ADJUST VIEW HEIGHT using keys 'u' up, 'd' down. ESC to quit.
//
#include <stdio.h>
#include "cv.h"
#include "highgui.h"
int main(int argc, char* argv[]) {
  if(argc != 3){
   printf("too few args\n");
   return -1;
	}
  // INPUT PARAMETERS:
  //
  CvMat*    intrinsic  = (CvMat*)cvLoad(argv[1]);
  CvMat*    distortion = (CvMat*)cvLoad(argv[2]);
  
  CvMat*    trans = cvCreateMat(3,1,CV_32FC1);
  CvMat*    rot = cvCreateMat(3,1,CV_32FC1);
  
  cvSetZero(trans);
  cvSetZero(rot);
  
  CvMat* objpoints = cvCreateMat(1,1,CV_32FC3);
  CvMat* imgpoints = cvCreateMat(1,1,CV_32FC2);
  
  
  CvCapture* capture = cvCaptureFromCAM( CV_CAP_ANY );
  if( !capture ) {
    fprintf( stderr, "ERROR: capture is NULL \n" );
    getchar();
    return -1;
  }
  
  int key = 0;
  float x=0;
  float y=0;
  float z=0;
  
  cvNamedWindow("awesomevision");

  IplImage *image = cvQueryFrame( capture );
  IplImage *temp_bw=cvCreateImage(cvSize(image->width,image->height),8,1);
  
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
	if(key==97)x--;
	else if(key==113)x++;
	else if(key==115)y--;
	else if(key==119)y++;
	else if(key==100)z--;
	else if(key==101)z++;
	int i=0;
	((float*)(objpoints->data.ptr+objpoints->step*i))[0]=x;
	((float*)(objpoints->data.ptr+objpoints->step*i))[1]=y;
	((float*)(objpoints->data.ptr))[2]=z;
	printf("%f %f %f\n",x,y,z);
	printf("%f %f\n",((float*)(imgpoints->data.ptr))[0],((float*)(imgpoints->data.ptr))[1]);

//	*((float*)CV_MAT_ELEM_PTR(*objpoints,0,0))=x;
//	*((float*)CV_MAT_ELEM_PTR(*objpoints,0,1))=y;
//	*((float*)CV_MAT_ELEM_PTR(*objpoints,0,2))=z;
	
	cvProjectPoints2(objpoints,rot,trans,intrinsic,distortion,imgpoints);
	cvCircle(image,cvPoint(((float*)(imgpoints->data.ptr+imgpoints->step*0))[0],((float*)(imgpoints->data.ptr+imgpoints->step*0))[1]),10,cvScalar(255,0,0),-1);
	cvShowImage( "awesomevision", image );
	key = cvWaitKey(2);
  }

  cvDestroyWindow( "awesomevision" );
  cvReleaseImage(&temp_bw);
  cvReleaseMat(&intrinsic);
  cvReleaseMat(&distortion);
  cvReleaseMat(&objpoints);
  cvReleaseMat(&imgpoints);
  cvReleaseMat(&rot);
  cvReleaseMat(&trans);
  cvReleaseCapture( &capture );
  return 0;
}
