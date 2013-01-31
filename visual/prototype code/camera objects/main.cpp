 
 #include "Camera.h"
 #include "CameraManager.h"
 #include "cv.h"
 #include "highgui.h"
 #include "TimeKeeper.h"
 #include <stdio.h>
 #include <iostream>
 
 using namespace std;
 
 int main(int argc, char* argv[]) {
   
   printf("starting\n");
   Camera a;
   a.init(0);
   
   Camera b;
   b.init(1);
   
   CameraManager manager;
   manager.init();
   
   manager.addCam(&a);
   manager.addCam(&b);
   
   
   TimeKeeper::start_time();
   try{
	 printf("inited\n");
//	 a.startThread();
//	 b.startThread();

	 manager.startThread();
	 int key=0;
	 cvNamedWindow( "image1", CV_WINDOW_AUTOSIZE );
	 cvNamedWindow( "image2", CV_WINDOW_AUTOSIZE );
	 while((key&255)!=27){
	   long start=TimeKeeper::GetTime();
	   //printf("looping\n");
	   manager.updateCameras();
	   manager.colorizeImages();
	   
	   //a.updateImage();
	   //b.updateImage();
	   
	   //a.colorizeImage();
	   //b.colorizeImage();
	   
	   cvShowImage("image1",a.image);
	   cvShowImage("image2",b.image);
	   long dt=TimeKeeper::GetTime()-start;
	   cout<<dt<<endl;
	   key=cvWaitKey(3);
	 }
   }
   catch(...){printf("somthing bad happened\n"); }
   
   cvDestroyWindow( "image1" );
   cvDestroyWindow( "image2" );
   
 }
 
