/**
*	LidarLocalizer.cpp
*	Author: Jesse Fish
*
*/

#include <iostream>
#include <stdio.h>
#include "LidarLocalizer.h"
using namespace std;

LidarLocalizer::LidarLocalizer(){
  //these values should NOT BE CODED HERE!
  //in meters
  corner_reject_rad=0.2;
  
  scanCount=361;
  scanAngle=0.5;
  
  rvec =cvCreateMat(scanCount,1,CV_64F);
  pingxy =cvCreateMat(scanCount,2,CV_64F);
  
  lidarScan =cvCreateMat(scanCount,1,CV_64F);
  lidarxy=cvCreateMat(scanCount,2,CV_64F);
  
}


void LidarLocalizer::loadMap(char*){
  
}
void LidarLocalizer::loadPresetMap(){
  double damap[11][4] = {{0.0000,0.0000,7.0150,0.0000},\
  {7.0150,0.0000,7.0150,2.1350},\
  {7.0150,2.1350,2.1350,2.1350},\
  {2.1350,2.1350,2.1350,12.8100},\
  {2.1350,12.8100,40.0000,12.8100},\
  {40.0000,14.6400,2.1350,14.6400},\
  {2.1350,14.6400,2.1350,30.8050},\
  {2.1350,30.8050,3.6600,30.8050},\
  {3.6600,30.8050,3.6600,32.9400},\
  {3.6600,32.9400,0.0000,32.9400},\
  {0.0000,32.9400,0.0000,0.0000}};
  
  num_points=11;
  
  map = cvCreateMat(num_points, 4, CV_64F);
  
  line_associations=new int [num_points];
  
  for(int i=0;i<num_points;i++){
	for(int j=0;j<4;j++){
	  cvmSet(map,i,j,damap[i][j]);
	}	
  }
  
  //normals of the map
  normals=cvCreateMat(2,num_points,CV_64F);
}

//no idea why i wrote this, not sure if it is useful at all
//as far as i can tell the algorithm doesn't need it, so im not cleaning this method up.

void LidarLocalizer::findMapNormals(){
  
  double Darr[][2]={{0,-1},{1,0}};
  //begin taking the normals of the map lines
  //90 degree rotation matrix
  CvMat D=cvMat(2,2,CV_64F,Darr);
  
  //pull the starting points out of the map
  CvMat startxy;
  cvInitMatHeader(&startxy,num_points,2,CV_64F);
  cvGetCols(&map,&startxy,0,2);
  //pull the ending points out of the map
  CvMat endxy;
  cvInitMatHeader(&endxy,num_points,2,CV_64F);
  cvGetCols(&map,&endxy,2,4);
  //normalized line vectors of map
  
  double normvects_arr [num_points][2];
  CvMat vects=cvMat(num_points,2,CV_64F,normvects_arr);
  cvSub(&endxy,&startxy,&vects);
  
  //normalize vectors
  CvMat tempPoint;
  cvInitMatHeader(&tempPoint,1,2,CV_64F);
  double mag=0;
  for(int i=0;i<num_points;i++){
	cvGetRows(&vects,&tempPoint,i,i+1);
	mag=cvNorm(&tempPoint);
	cvmSet(&tempPoint,0,0,cvmGet(&tempPoint,0,0)/mag);
	cvmSet(&tempPoint,0,1,cvmGet(&tempPoint,0,1)/mag);
  }
  //transpose of the vectors
  double temp_transpvects [2][num_points];
  CvMat tvects=cvMat(2,num_points,CV_64F, temp_transpvects);
  
  cvTranspose( &vects,&tvects);
  cvMatMul(&D,&tvects,normals);
  //finished taking the normals of the map lines
}


void LidarLocalizer::estimatePosition(PositionState* position, CvMat* lidarScan){
  
  for(int i=0;i<lidarScan.
  
  
  
  display(position);
  
  
  
}

void LidarLocalizer::associatePoints(CvMat* lidarScan){
  
}

void LidarLocalizer::display(PositionState* position,CvMat* lidarScan){
  cvNamedWindow( "keycatcher", CV_WINDOW_AUTOSIZE );
  IplImage* img=cvCreateImage(cvSize(500,500),IPL_DEPTH_32F,3);
  
  
  int headingLen=20;
  CvPoint origin=cvPoint(250+position->crioPosition.x*10,400+position->crioPosition.y*-10);
  cvCircle(img,origin,4,cvScalar(0,255,0),-1,8);
  cvLine(img, origin ,cvPoint(cos(-(position->crioRotRad)-3.1415/2)*headingLen+origin.x,sin(-(position->crioRotRad)-3.1415/2)*headingLen+origin.y),cvScalar(0,0,255),2);
  
  
  for(int i=0;i<num_points;i++){
	cvLine(img,cvPoint(cvmGet(map,i,0)*10+250,cvmGet(map,i,1)*-10+400),cvPoint(cvmGet(map,i,2)*10+250,cvmGet(map,i,3)*-10+400),cvScalar(255,0,0),2);
  }
  
  
  
  CvPoint curve1[scanCount];
  for(int i=0;i<scanCount;i++){
	curve1[i].x=int(cvmGet(pingxy,i,0)*10)+250;
	curve1[i].y=int(cvmGet(pingxy,i,1)*-10)+400;
  }
  
  CvPoint* curveArr[1]={curve1};
  int      nCurvePts[1]={scanCount};
  int      nCurves=1;
  
  cvPolyLine(img,curveArr,nCurvePts,nCurves,0,cvScalar(0,0,255));
  
  
  
  cvShowImage("keycatcher", img); 
  
  cvWaitKey(0);
  cvReleaseImage(&img);
  cvDestroyWindow( "keycatcher" );
  
}
/*
void LidarLocalizer::virtualScan(double crioX, double crioY, double crioRotRad,CvMat* rvec,CvMat* pingxy){
 
 //zero values
 cvSet(rvec,cvScalar(80));
 cvSet(pingxy,cvScalar(0));
 
 // Compute the virtual scan, and FUCK IT
 double ping_ar[2][1];
 double temp1_ar[2][1];
 double temp2_ar[2][1];
 
 CvMat ping=cvMat(2,1,CV_64F,ping_ar);
 CvMat temp1=cvMat(2,1,CV_64F,temp1_ar);
 CvMat temp2=cvMat(2,1,CV_64F,temp2_ar);
 //  scanCount
 for (int iray = 0; iray < scanCount; iray++) {
 //this is where some angle magic happens, might be a source of errors
 double ray_ang = crioRotRad + ((scanAngle*iray))*3.14159/180.0;
 int lclosest = 0;
 //cout<<cvmGet(rvec,iray,0)<<endl;
 
 for (int i = 0; i <num_points; i++) {
 double xs = cvmGet(map, i, 0);
 double ys = cvmGet(map, i, 1);
 double xf = cvmGet(map, i, 2);
 double yf = cvmGet(map, i, 3);
 
 double ps_arr [][1] = {{xs},{ys}};
 CvMat ps=cvMat(2,1,CV_64F,ps_arr);
 
 double pf_arr [][1] = {{xf},{yf}};
 CvMat pf=cvMat(2,1,CV_64F,pf_arr);
 double intersect_arr [2][1];
 
 CvMat intersectMat =cvMat(2, 1, CV_64F,intersect_arr);
 find_intersect(xs, ys, xf, yf, crioX, crioY, ray_ang, &intersectMat);
 double s = cvmGet(&intersectMat,0, 0);
 double r = cvmGet(&intersectMat,1, 0);
 if(s>=0&&s<=01&&r>0 &&r<cvmGet(rvec,iray,0)){
 cvmSet(rvec,iray,0,r);
 cvSub(&pf,&ps,&temp1);
 cvScale(&temp1,&temp2,s);
 cvAdd(&temp2,&ps,&ping);
 //ping=ps+(pf-ps)*s; matrix operation we want to perform
 lclosest=i;
 }
 }
 
 if(lclosest>0){
 cvmSet(pingxy,iray,0,cvmGet(&ping,0,0));
 cvmSet(pingxy,iray,1,cvmGet(&ping,1,0));
 cout<<iray<<" "<<ray_ang<<endl<<"x: "<<cvmGet(&ping,0,0)<<" y:"<<cvmGet(&ping,1,0)<<" r:"<<cvmGet(rvec,iray,0)<<endl;
 }
 }
 
 }*/


/*
//dest_val must be a 2 by 1 double matrix like this cvCreateMat(2, 1, CV_64F);
void LidarLocalizer::find_intersect(double xs, double ys, double xf, double yf, double xc, double yc, double theta,CvMat* dest_val) {
 double A[2][2] = {{xf-xs, -cos(theta)}, {yf-ys, -sin(theta)}};
 double b[2][1] = {{xc-xs}, {yc-ys}};
 //cout<<A[0][0]<<" "<<A[0][1]<<" "<<A[1][0]<<" "<<A[1][1]<<endl;
 CvMat cvA = cvMat(2, 2, CV_64F, A);
 double temp [2][2];
 CvMat cvAinv = cvMat(2, 2, CV_64F,temp);
 CvMat cvb = cvMat(2, 1, CV_64F, b);
 cvInvert(&cvA, &cvAinv,CV_SVD);
 cvMatMul(&cvAinv, &cvb, dest_val);
 }*/

LidarLocalizer::~LidarLocalizer(){
  
  delete [] line_associations;
  
  cvReleaseMat(&map);
  cvReleaseMat(&rvec);
  cvReleaseMat(&pingxy);
  cvReleaseMat(&lidarScan);
  cvReleaseMat(&lidarxy);
  
}

