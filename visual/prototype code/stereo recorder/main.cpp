 
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
   int fps=a.getCamProp(CV_CAP_PROP_FPS);
   cout<<"cam fps "<<fps<<endl;
   //Camera b;
   //b.init(1);
   
   
   CameraManager manager;
   manager.init();
   
   manager.addCam(&a);
  // manager.addCam(&b);
   
   
   
   int picturecount=1;
   
   char buffer [50];
   
   
   //   TimeKeeper::start_time();
   CvSize size=cvSize(a.getCamProp(CV_CAP_PROP_FRAME_WIDTH),a.getCamProp(CV_CAP_PROP_FRAME_HEIGHT));
   
   // CvVideoWriter *writer1=cvCreateVideoWriter("cam1.avi",CV_FOURCC('D','I','V','3'),fps,size);
   // CvVideoWriter *writer2=cvCreateVideoWriter("cam2.avi",CV_FOURCC('D','I','V','3'),fps,size);
   
   try{
	 printf("inited\n");
	 // a.startThread();
	 // b.startThread();
	 
	 manager.startThread();
	 int key=0;
	 cvNamedWindow( "image1", CV_WINDOW_AUTOSIZE );
//	 cvNamedWindow( "image2", CV_WINDOW_AUTOSIZE );
	 while((key&255)!=27){
	   //	   long start=TimeKeeper::GetTime();
	   //printf("looping\n");
	   //   a.updateImage();
	   //   b.updateImage();
	   
	   //   a.colorizeImage();
	   //   b.colorizeImage();
	   manager.updateCameras();
	   manager.colorizeImages();
	   
	   
	   sprintf (buffer, "folder/left%02d.bmp", picturecount++);
	   if(!cvSaveImage(buffer,a.image,0)) printf("Could not save: %s\n",buffer);
	   else printf("picture taken!!!\n");
	   
/*	   
	   sprintf (buffer, "right%02d.bmp", picturecount++);
	   if(!cvSaveImage(buffer,b.image,0)) printf("Could not save: %s\n",buffer);
	   else printf("picture taken!!!\n");
*/	   
	  // cvWriteFrame(writer1,a.image);
	 //  cvWriteFrame(writer2,b.image);
	   
	   cvShowImage("image1",a.image);
//	   cvShowImage("image2",b.image);
	   
	   
	   //	   long dt=TimeKeeper::GetTime()-start;
	   //	   cout<<dt<<endl;
	   key=cvWaitKey(30);
	 }
   }
   catch(...){printf("somthing bad happened\n"); }
 //  cvReleaseVideoWriter(&writer1);
 //  cvReleaseVideoWriter(&writer2);
   cvDestroyWindow( "image1" );
 //  cvDestroyWindow( "image2" );
   
 }
 
 
