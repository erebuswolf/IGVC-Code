//Call:
//  birds-eye board_w board_h instrinics distortion image_file
// ADJUST VIEW HEIGHT using keys 'u' up, 'd' down. ESC to quit.
//
#include <stdio.h>
#include <iostream>
#include "cv.h"
#include "highgui.h"
#include "TimeKeeper.h"

using namespace std;
using namespace cv;

bool quit=false;
pthread_mutex_t image_mutex= PTHREAD_MUTEX_INITIALIZER;

void *camera_grabber(void* voidcap);

int main(int argc, char* argv[]) {
  TimeKeeper::start_time();
  
  pthread_t listenThread;
  
  // INPUT PARAMETERS:
  //
  CvMat*    intrinsic  = (CvMat*)cvLoad("C4L1Intrinsics.xml");
  CvMat*    distortion = (CvMat*)cvLoad("C4L1Distortion.xml");
  CvMat*    H = (CvMat*)cvLoad("H.xml");
  
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
  
  CvMemStorage* storage = cvCreateMemStorage(0);
  CvSeq* lines = 0;
  
  
  cvNamedWindow("Birds_Eye");
  
  IplImage *image = cvCloneImage(cvQueryFrame( capture ));
  IplImage *temp_bw=cvCreateImage(cvSize(image->width,image->height),8,1);
  
  IplImage *color_canny=cvCloneImage(image);
  
  IplImage *edges=cvCloneImage(temp_bw);
  //IplImage *birds_image=cvCloneImage(temp_bw);
  IplImage *birds_image=cvCloneImage(image);
  
  // LOOP TO ALLOW USER TO PLAY WITH HEIGHT:
  //
  // escape key stops
  //
  
  int fps=(int)cvGetCaptureProperty(capture,CV_CAP_PROP_FPS);
  //spawn capture thread right before loop
  int thread_id=pthread_create(&listenThread,NULL,&camera_grabber,(void*)capture);
  
  while(key != 27) {
	long start=TimeKeeper::GetTime();
	
	
	
	//grab a frame from the camera
	//wait for the thread to change modalities
	pthread_mutex_lock(&image_mutex);
	//grab the newest image from the camera
	cvCopy(cvRetrieveFrame(capture),image);
	//cout<<"got image"<<endl;
	//let the thread run again
	pthread_mutex_unlock(&image_mutex);
	
	//make the camera frame have 1 color layer
	cvCvtColor(image,temp_bw,CV_RGB2GRAY);
	//convert the camera frame to color
	cvCvtColor(temp_bw,image,CV_BayerGB2RGB);
	
	
	
	//		cvCopy( image, birds_image);
	// Set the height
	//
	CV_MAT_ELEM(*H,float,2,2) = Z;
	
	// COMPUTE THE FRONTAL PARALLEL OR BIRD'S-EYE VIEW:
	// USING HOMOGRAPHY TO REMAP THE VIEW
	//
	cvWarpPerspective(image,
					  birds_image,
					  H,
					  CV_INTER_LINEAR | CV_WARP_INVERSE_MAP | CV_WARP_FILL_OUTLIERS
					  );
					  
	//convert the converted image back to bw
	cvCvtColor(birds_image,temp_bw,CV_RGB2GRAY);
	
	cvCanny(temp_bw,edges,30,50,3);
	
	cvCvtColor(edges,color_canny,CV_GRAY2RGB);
	
	
	lines = cvHoughLines2( edges, storage, CV_HOUGH_STANDARD, 1, CV_PI/180, 150, 0, 0 );
	
	
	
	for( int i = 0; i < lines->total; i++ )
	{
	  float* line = (float*)cvGetSeqElem(lines,i);
	  float rho = line[0];
	  float theta = line[1];
	  CvPoint pt1, pt2;
	  double a = cos(theta), b = sin(theta);
	  if( fabs(a) < 0.001 )
	  {
		pt1.x = pt2.x = cvRound(rho);
		pt1.y = 0;
		pt2.y = color_canny->height;
	  }
	  else if( fabs(b) < 0.001 )
	  {
		pt1.y = pt2.y = cvRound(rho);
		pt1.x = 0;
		pt2.x = color_canny->width;
	  }
	  else
	  {
		pt1.x = 0;
		pt1.y = cvRound(rho/b);
		pt2.x = cvRound(rho/a);
		pt2.y = 0;
	  }
	  cvLine( color_canny, pt1, pt2, CV_RGB(255,0,0), 3, 8 );
	}
	
	
	cvShowImage( "Birds_Eye",color_canny );
	
	
	
	long dt=TimeKeeper::GetTime()-start;
	cout<<dt<<endl;
	key = cvWaitKey(2);
	if(key == 'u') Z += 0.5;
	if(key == 'd') Z -= 0.5;
					  
  }
  
  quit=true;
  usleep(100);
  //just sleep dont try to join.. it will fail and segfault
  //pthread_join(thread_id,NULL);
  cvReleaseImage(&edges);
  cvReleaseImage(&color_canny);
  cvReleaseImage(&birds_image);
  cvReleaseImage(&temp_bw);
  return 0;
}

void *camera_grabber(void* voidcap){
  while(!quit){
	pthread_mutex_lock(&image_mutex);
	CvCapture* capture=(CvCapture*)voidcap;
	cvGrabFrame(capture);
	//cout<<"queried image"<<endl;
	pthread_mutex_unlock(&image_mutex);
	//can sleep for at least this ammount, camera has an fps of 15, so 1 frame every 66666 usecs
	usleep(16666);
  }
}
