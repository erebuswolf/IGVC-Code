/**
*	VisionInterpreter.cpp
*	Author: Jesse Fish
*
*/

#include "VisionInterpreter.h"
#include <iostream> 
#include <fstream>

using namespace std;
using namespace cv;
VisionInterpreter::VisionInterpreter(CameraManager* manager){
  this->manager=manager;
  //TODO: window size value should be moved to a config file
  //hard set value
  windowSize=600;
  processedImage=cvCreateImage( cvSize(windowSize,windowSize), 8, 1 );
  subtractorImage=cvCloneImage(processedImage);
  rotationImage=cvCloneImage(processedImage);
  
  hsv=cvCloneImage(manager->cameras[0]->image);
  dewarped_image=cvCloneImage(hsv);
  hsv_s=cvCreateImage( cvGetSize(hsv), 8, 1 );
  hsv_v=cvCreateImage( cvGetSize(hsv), 8, 1 );
  subtractorLayer=cvCloneImage(hsv_s);

  mapx =cvCreateImage( cvGetSize(dewarped_image), IPL_DEPTH_32F, 1 );
  mapy =cvCreateImage( cvGetSize(dewarped_image), IPL_DEPTH_32F, 1 );
  //This initializes rectification matrices
  //
  
  cvInitUndistortMap(
	manager->cameras[0]->calib.intrinsic,
	manager->cameras[0]->calib.distortion,
	mapx,
	mapy
  );
  
  
}
void VisionInterpreter::init(string config_filepath){
  
  ifstream config_file (config_filepath.c_str());
  
  if (config_file.is_open())
  {
	config_file>>s_c;
	config_file.ignore(300,'\n');
	config_file>>s_k;
	config_file.ignore(300,'\n');
	
	config_file>>v_c;
	config_file.ignore(300,'\n');
	config_file>>v_k;
	config_file.ignore(300,'\n');

	config_file>>lineThreshold;
	config_file.ignore(300,'\n');
	
	config_file>>subScale;
	config_file.ignore(300,'\n');
	
	config_file>>black_space;
	config_file.ignore(300,'\n');
	
	config_file>>debugVision;
	config_file.ignore(300,'\n');
	
  }  else cerr << "Unable to open lidar config file" << endl; 
  
  lineThreshold*=255;
  
  if(debugVision){
	cvNamedWindow("debug add layer", CV_WINDOW_AUTOSIZE );
	cvNamedWindow("debug sub layer", CV_WINDOW_AUTOSIZE );
  }
}

//function to find lines and obsticals in the images from all cameras
void VisionInterpreter::findLinesAndObsticles(double heading){
  heading=heading*180/3.1415;
  
  manager->colorizeImages();
  //grab the first camera
  
  cvRemap(manager->cameras[0]->image, dewarped_image, mapx, mapy );

  cvCvtColor(dewarped_image,hsv,CV_RGB2HSV);
  //conver that image to hsv
  //color filter the hsv
  
  //math for this is (s_c-hsv_s)*s_k - (v_c-hsv_v)*v_k
  cvSplit(hsv,NULL,hsv_s,NULL,NULL);
  cvSplit(hsv,NULL,NULL,hsv_v,NULL);
  
  //threshold appropriately
  
  cvSet(subtractorLayer,cvScalar(s_c));
  cvSub(subtractorLayer,hsv_s,subtractorLayer);
  
  
  cvConvertScale(subtractorLayer,hsv_s,s_k);
  
  
  cvSet(subtractorLayer,cvScalar(v_c));
  
  cvSub(subtractorLayer,hsv_v,subtractorLayer);
  
  cvConvertScale(subtractorLayer,hsv_v,v_k);
  
  cvSub(hsv_s,hsv_v,hsv_s);
  
  
  cvSet(subtractorLayer,cvScalar(255));
  
  cvSub(subtractorLayer,hsv_s,subtractorLayer);
  
  cvConvertScale(subtractorLayer,subtractorLayer,subScale);

  
  cvThreshold(hsv_s,hsv_s,lineThreshold,0,CV_THRESH_TOZERO);
  
  //clear out top portion of image -- determined by black space config variable
  cvSetImageROI(hsv_s, cvRect(0, 0, hsv_s->width, hsv_s->height*black_space));
  cvSetZero(hsv_s);
  cvResetImageROI(hsv_s);
  
  if(debugVision){
	cvShowImage( "debug add layer",hsv_s);
	cvShowImage( "debug sub layer",subtractorLayer);
  }
  
  cvSetZero(processedImage);
  cvSetZero(subtractorImage);
  
  
  cvWarpPerspective(hsv_s,
					processedImage,
					(manager->cameras[0]->calib.H),
					CV_INTER_LINEAR | CV_WARP_INVERSE_MAP | CV_WARP_FILL_OUTLIERS
					);
					
  cvWarpPerspective(subtractorLayer,
					subtractorImage,
					(manager->cameras[0]->calib.H),
					CV_INTER_LINEAR | CV_WARP_INVERSE_MAP | CV_WARP_FILL_OUTLIERS
					);
  
  //rotate by this heading
  Point_<double> center;
  center.x=processedImage->width/2;
  center.y=processedImage->height/2;
  double tranlation_arr[2][3];
  CvMat translation;
  cvInitMatHeader(&translation,2,3,CV_64F,tranlation_arr);
  cvSetZero(&translation);
  cv2DRotationMatrix(center,heading,1.0,&translation);
  cvSetZero(rotationImage);
  cvWarpAffine(processedImage,rotationImage,&translation,CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS,cvScalarAll(0));
  
  IplImage* temp;
  temp=processedImage;
  processedImage=rotationImage;
  rotationImage=temp;
  
  cvSetZero(rotationImage);
  cvWarpAffine(subtractorImage,rotationImage,&translation,CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS,cvScalarAll(0));
  temp=subtractorImage;
  subtractorImage=rotationImage;
  rotationImage=temp;
  
  
  //calibrate this top down view to overlay with our world model image
  //have the wolrd model add this image 
}

VisionInterpreter::~VisionInterpreter(){
  //need to release the matricies from the calibrations vector here
  
}
