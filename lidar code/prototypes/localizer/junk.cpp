 
 
 
 /**
 
 *** THIS WAS A PROTOTYPE CLASS, NOW USE THE MAIN.CPP AND LIDARLOCALIZER CLASS
 */
 
 
 
#include "cv.h"
#include "highgui.h"
#include <iostream>
#include <stdio.h>

using namespace std;
using namespace cv;

void find_intersect(double xs, double ys, double xf, double yf, double xc, double yc, double theta,CvMat* dest_val);

int main(){
	
	
	// TODO: Read in the map
	double damap[11][4] = {{0.0000,0.0000,7.0150,0.0000},\
			{7.0150,0.0000,7.0150,2.1350},\
			{7.0150,2.1350,2.1350,2.1350},\
			{2.1350,2.1350,2.1350,12.8100},\
			{2.1350,12.8100,40.0000,12.8100},\
			{40.0000,14.6400,2.1350,14.6400},\
			{2.1350,14.6400,2.1350,30.8050},\
			{2.1350,30.8050,3.6600,30.8050},\
			{3.6600,3.8050,3.6600,32.9400},\
			{3.6600,32.9400,0.0000,32.9400},\
			{0.0000,32.9400,0.0000,0.0000}};
	
	double crioRotRad = 0;
	double crioX = 1.0;
	double crioY = 5.0;
	
	CvMat map = cvMat(11, 4, CV_64F,damap);
	
	double Darr[][2]={{0,-1},{1,0}};
	  
   	int num_points=11;
   



/*
//completely unnecissary step
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
	CvMat* vects=cvCreateMat(num_points,2,CV_64F);
	cvSub(&endxy,&startxy,vects);
   
	//normalize vectors
	CvMat tempPoint;
	cvInitMatHeader(&tempPoint,1,2,CV_64F);
	double mag=0;
	for(int i=0;i<num_points;i++){
		cvGetRows(vects,&tempPoint,i,i+1);
		mag=cvNorm(&tempPoint);
		cvmSet(&tempPoint,0,0,cvmGet(&tempPoint,0,0)/mag);
		cvmSet(&tempPoint,0,1,cvmGet(&tempPoint,0,1)/mag);
	}
	//normals of the map
	CvMat* normals=cvCreateMat(2,num_points,CV_64F);
   	//transpose of the vectors
	CvMat* tvects=cvCreateMat(2,num_points,CV_64F);
	cvTranspose( vects,tvects);
	cvMatMul(&D,tvects,normals);
	//finished taking the normals of the map lines
	*/
	

	//begin virtual scan
	
	CvMat* rvec =cvCreateMat(181,1,CV_64F);
	CvMat* pingxy =cvCreateMat(181,2,CV_64F);
	
	cvSet(rvec,cvScalar(80));
	// Compute the virtual scan, and FUCK IT
	
	CvMat* ping=cvCreateMat(2,1,CV_64F);
	CvMat* temp1=cvCreateMat(2,1,CV_64F);
	CvMat* temp2=cvCreateMat(2,1,CV_64F);
	cout<<"looping\n";
	for (int iray = 0; iray < 181; iray++) {
		double ray_ang = crioRotRad + ((iray-1)-90)*3.14159/180.0;
		int lclosest = 0;
		
		
		for (int i = 0; i <num_points; i++) {
			double xs = cvmGet(&map, i, 0);
			double ys = cvmGet(&map, i, 1);
			double xf = cvmGet(&map, i, 2);
			double yf = cvmGet(&map, i, 3);
			
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
				cvSub(&pf,&ps,temp1);
				cvScale(temp1,temp2,s);
				cvAdd(temp2,&ps,ping);
				//ping=ps+(pf-ps)*s; matrix operation we want to perform
				lclosest=i;
			}
		}
		if(lclosest>0){
			cvmSet(pingxy,iray,0,cvmGet(ping,0,0));
			cvmSet(pingxy,iray,1,cvmGet(ping,1,0));
			
			cout<<cvmGet(ping,0,0)<<" "<<cvmGet(ping,1,0)<<" "<<cvmGet(rvec,iray,0)<<endl;
		}
	}
	
	//visualize
	cout<<"vis...\n";
	
	cvNamedWindow( "keycatcher", CV_WINDOW_AUTOSIZE );
	
	IplImage* img=cvCreateImage(cvSize(500,500),IPL_DEPTH_32F,3);
	
	
	CvPoint curve1[181];
	for(int i=0;i<181;i++){
	  curve1[i].x=int(cvmGet(pingxy,i,0)*10)+250;
	  curve1[i].y=int(cvmGet(pingxy,i,1)*-10)+400;
	}
	
	CvPoint* curveArr[1]={curve1};
	int      nCurvePts[1]={181};
	int      nCurves=1;
	
	cout<<"boop...\n";
	cvPolyLine(img,curveArr,nCurvePts,nCurves,0,cvScalar(255,255,255));
	
	cout<<"boop...\n";
	
	
	cout<<"trying...\n";
	cvShowImage("keycatcher", img); 
	cvWaitKey(0);
	
}
//dest_val must be a 2 by 1 double matrix like this cvCreateMat(2, 1, CV_64F);
void find_intersect(double xs, double ys, double xf, double yf, double xc, double yc, double theta, CvMat* dest_val) {
  	double A[2][2] = {{xf-xs, -cos(theta)}, {yf-ys, -sin(theta)}};
	double b[2][1] = {{xc-xs}, {yc-ys}};
	CvMat cvA = cvMat(2, 2, CV_64F, A);
	double temp [2][2];
	CvMat cvAinv = cvMat(2, 2, CV_64F,temp);
	CvMat cvb = cvMat(2, 1, CV_64F, b);
	cvInvert(&cvA, &cvAinv,CV_SVD);
	cvMatMul(&cvAinv, &cvb, dest_val);
}