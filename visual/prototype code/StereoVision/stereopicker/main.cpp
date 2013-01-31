 
 #include "cv.h"
 #include "highgui.h"
 #include <stdio.h>
 #include <iostream>
 using namespace cv;
 using namespace std;
 
 int main(int argc, char* argv[]) {
   
   printf("starting\n");
   
  CvCapture* capLeft=cvCreateFileCapture("cam1.avi");
  CvCapture* capRight=cvCreateFileCapture("cam2.avi");
  
//   TimeKeeper::start_time();
 
   try{
	 printf("inited\n");
	
	 int key=0;
	 cvNamedWindow( "image1", CV_WINDOW_AUTOSIZE );
	 cvNamedWindow( "image2", CV_WINDOW_AUTOSIZE );
	 IplImage* leftim;
	 IplImage* rightim;
	 int picturecount=1;
	 char buffer [50];
	 bool pause=false;
	 while((key&255)!=27){
	   if(!pause){
		leftim=cvQueryFrame(capLeft);
		rightim=cvQueryFrame(capRight);
	   }
	   
	   cvShowImage("image1",leftim);
	   cvShowImage("image2",rightim);

	   
//	   long dt=TimeKeeper::GetTime()-start;
//	   cout<<dt<<endl;
	   key=cvWaitKey(50);
	   if((key&255)==32){
		pause=!pause; 
	   }
	   if((key&255)==115){
		//save images
			sprintf (buffer, "left%02d.jpg", picturecount);
			if(!cvSaveImage(buffer,leftim,0)) printf("Could not save: %s\n",buffer);
			else printf("picture taken!!!\n");
			
			
			sprintf (buffer, "right%02d.jpg", picturecount);
			if(!cvSaveImage(buffer,rightim,0)) printf("Could not save: %s\n",buffer);
			else printf("picture taken!!!\n");
			
			picturecount++;
	   }
	   
	 }
   }
   catch(...){printf("somthing bad happened\n"); }
   cvReleaseCapture(&capLeft);
   cvReleaseCapture(&capRight);
   cvDestroyWindow( "image1" );
   cvDestroyWindow( "image2" );
   
 }
 
