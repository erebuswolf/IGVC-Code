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
  corner_reject_rad=.5;
  max_dist=1;
  algorithm_iterations=5;
  
  // scanCount=361;
  scanAngle=0.5;
  
  acceptable_line_error=0.1;
  
  cvNamedWindow( "keycatcher", CV_WINDOW_AUTOSIZE );
//  cvNamedWindow( "new_pings", CV_WINDOW_AUTOSIZE );
  
}


void LidarLocalizer::loadMap(char*){
  
}
void LidarLocalizer::loadPresetMap(){
  int scanCount=361;
  
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
  
  num_lines=11;
  
  map = cvCreateMat(num_lines, 4, CV_64F);
  
  for(int i=0;i<num_lines;i++){
	for(int j=0;j<4;j++){
	  cvmSet(map,i,j,damap[i][j]);
	}	
  }
  
  //pull the starting points out of the map
  cvInitMatHeader(&startxy,num_lines,2,CV_64F);
  
  cvGetCols(map,&startxy,0,2);
  
  //pull the ending points out of the map
  cvInitMatHeader(&endxy,num_lines,2,CV_64F);
  
  cvGetCols(map,&endxy,2,4);
  
  line_associations=new int [scanCount];
  line_distances=new int [scanCount];
  ping_count=new int [num_lines];
  
  for(int i=0;i<num_lines;i++){
	ping_count[i]=0;
  }
  
  //normals of the map
  normals=cvCreateMat(2,num_lines,CV_64F);
  
  TW=cvCreateMat(2,num_lines*2,CV_64F);
  TWPerp=cvCreateMat(2,num_lines*2,CV_64F);
  findMapNormals();
}

//no idea why i wrote this, not sure if it is useful at all
//as far as i can tell the algorithm doesn't need it, so im not cleaning this method up.

void LidarLocalizer::findMapNormals(){
  
  double Darr[][2]={{0,-1},{1,0}};
  //begin taking the normals of the map lines
  //90 degree rotation matrix
  CvMat D=cvMat(2,2,CV_64F,Darr);
  
  //normalized line vectors of map
  double normvects_arr [num_lines][2];
  CvMat vects=cvMat(num_lines,2,CV_64F,normvects_arr);
  cvSub(&endxy,&startxy,&vects);
  
  //normalize vectors
  CvMat tempPoint;
  cvInitMatHeader(&tempPoint,1,2,CV_64F);
  double mag=0;
  for(int i=0;i<num_lines;i++){
	cvGetRow(&vects,&tempPoint,i);
	mag=cvNorm(&tempPoint);
	
	//switching sign!! to make normals point out!!
	cvmSet(&tempPoint,0,0,cvmGet(&tempPoint,0,0)/-mag);
	cvmSet(&tempPoint,0,1,cvmGet(&tempPoint,0,1)/-mag);
  }
  //transpose of the vectors
  double temp_transpvects [2][num_lines];
  CvMat tvects=cvMat(2,num_lines,CV_64F, temp_transpvects);
  
  cvTranspose( &vects,&tvects);
  cvMatMul(&D,&tvects,normals);
  //finished taking the normals of the map lines
  
  
  CvMat TWval;
  cvInitMatHeader(&TWval,2,2,CV_64F);
  
  CvMat TPerpval;
  cvInitMatHeader(&TPerpval,2,2,CV_64F);
  
  double norm_point_arr[2][1];
  CvMat norm_point=cvMat(2,1,CV_64F,norm_point_arr);
  
  double Tnorm_point_arr[1][2];
  CvMat Tnorm_point=cvMat(1,2,CV_64F,Tnorm_point_arr);
  
  double Dtrans_arr[2][2];
  CvMat Dtrans=cvMat(2,2,CV_64F,Dtrans_arr);
  cvTranspose(&D,&Dtrans);
  
  double Temp_mult_arr[2][2];
  CvMat Temp_mult;
  cvInitMatHeader(&Temp_mult,2,2,CV_64F,Temp_mult_arr);
  for(int i=0;i<num_lines;i++){
	//calc TW
	cvGetCols(TW,&TWval,i*2,i*2+2);
	cvGetCol(normals,&norm_point,i);
	cvTranspose( &norm_point,&Tnorm_point);
	cvMatMul(&norm_point,&Tnorm_point,&TWval);
	//calc TWPerp
	cvGetCols(TWPerp,&TPerpval,i*2,i*2+2);
	cvMatMul(&Dtrans,&TWval,&Temp_mult);
	cvMatMul(&Temp_mult,&D,&TPerpval);
  }
}


void LidarLocalizer::estimatePosition(PositionState* posState,const CvMat* lidarScan,int scan_count){
  
  posState->estPosition.x=posState->crioPosition.x;
  posState->estPosition.y=posState->crioPosition.y;
  posState->estRotRad=posState->crioRotRad;
  //  cout<<"starting estimation"<<posState->estPosition.x<<" "<<posState->estPosition.y<<" "<<posState->estRotRad<<endl;
  
  CvMat the_points;
  cvInitMatHeader(&the_points,scan_count,2,CV_64F);
  cvGetRows(lidarScan,&the_points,0,scan_count);
  
  double my_points_arr[scan_count][2];
  CvMat my_points=cvMat(scan_count,2,CV_64F,my_points_arr);
  
  //copy lidar scan into my own points
  cvCopy(&the_points,&my_points);
  
  CvMat Ping_point;
  cvInitMatHeader(&Ping_point,1,2,CV_64F);
  
  for(int i=0;i<scan_count;i++){
	cvGetRow(&my_points,&Ping_point,i);	
	//cout<<"ping point "<<cvmGet(&Ping_point,0,0)<<" "<<cvmGet(&Ping_point,0,1)<<endl;
	cvmSet(&Ping_point,0,0,cvmGet(&Ping_point,0,0)+posState->crioPosition.x);
	cvmSet(&Ping_point,0,1,cvmGet(&Ping_point,0,1)+posState->crioPosition.y);
	
	//cout<<"ping point trans "<<cvmGet(&Ping_point,0,0)<<" "<<cvmGet(&Ping_point,0,1)<<endl;
  }
  //cout<<"criopos: "<<posState->crioPosition.x<<" "<< posState->crioPosition.y<<" "<<posState->crioRotRad <<endl;
  
  double my_points_backup_arr[scan_count][2];
  CvMat my_points_backup=cvMat(scan_count,2,CV_64F,my_points_backup_arr);
  
  //copy lidar scan into my own points
  cvCopy(&my_points,&my_points_backup);
  
  
  //cout<<"starting to associate points"<<endl;
  associatePoints(&my_points,scan_count);
 // cout<<"done associating points"<<endl;
  
  
  calcBetterPos(posState,&my_points,scan_count);
  
  display(posState,&my_points_backup,&my_points,scan_count);
}


void LidarLocalizer::calcBetterPos(PositionState* posState, CvMat* my_points,int scan_count){
  
  for(int iters=0;iters<algorithm_iterations;iters++){
	double Darr[][2]={{0,-1},{1,0}};
	//begin taking the normals of the map lines
	//90 degree rotation matrix
	CvMat D=cvMat(2,2,CV_64F,Darr);
	
	double forces_arr[2][scan_count];
	CvMat forces=cvMat(2,scan_count,CV_64F,forces_arr);
	cvSetZero(&forces);
	
	//begin algorithm
	double sumJT_arr[2][2];
	CvMat sumJT=cvMat(2,2,CV_64F,sumJT_arr);
	cvSetZero(&sumJT);
	
	CvMat TempTW;
	cvInitMatHeader(&TempTW,2,2,CV_64F);
	
	for(int i=0;i<num_lines;i++){  
	  cvGetCols(TW,&TempTW,i*2,i*2+2);
	  cvScaleAdd(&TempTW,cvScalar(ping_count[i]),&sumJT,&sumJT);
	}
	
	CvMat F_i;
	cvInitMatHeader(&F_i,2,1,CV_64F);
	
	//the ping points and line starts... are stupid... sorry
	CvMat Ping_point;
	cvInitMatHeader(&Ping_point,1,2,CV_64F);
	
	CvMat line_start;
	cvInitMatHeader(&line_start,1,2,CV_64F);
	
	double Fnet_arr[2][1];
	CvMat Fnet=cvMat(2,1,CV_64F,Fnet_arr);
	cvSetZero(&Fnet);
	
	
	double Temp_sum_arr[1][2];
	CvMat Temp_sum=cvMat(1,2,CV_64F,Temp_sum_arr);
	double Temp_trans_arr[2][1];
	CvMat Temp_trans=cvMat(2,1,CV_64F,Temp_trans_arr);
	
	
	for(int i=0;i<scan_count;i++){
	  cvGetCol(&forces,&F_i,i);
	  
	  if(line_associations[i]>=0){
		//grab the TW for this line this point is closest to
		cvGetCols(TW,&TempTW,line_associations[i]*2 ,line_associations[i]*2+2);
		//grab the startxy point for the line this point is closest to
		cvGetRow(&startxy,&line_start,line_associations[i]);  
		//get the point we are looking at in the scan
		cvGetRow(my_points,&Ping_point,i);
		//sub the start point from the scan point: (ping-startxy)
		cvSub(&Ping_point,&line_start,&Temp_sum);
		//transpose that answer
		cvTranspose(&Temp_sum,&Temp_trans);
		//get force for this point TW*(ping-startxy)
		cvMatMul(&TempTW,&Temp_trans,&F_i);
		//add to sum of forces Fnet+=f_i
		cvAdd(&F_i,&Fnet,&Fnet);
	//	cout<<"internal force "<<i<<" "<<cvmGet(&F_i,0,0)<<" "<<cvmGet(&F_i,1,0)<<endl;
	  }
	}
	
	//mu value
	double mu_arr[2][1];
	CvMat mu=cvMat(2,1,CV_64F,mu_arr);
	
	double sumJTINV_arr[2][2];
	CvMat sumJTINV=cvMat(2,2,CV_64F,sumJTINV_arr);
	cvInvert(&sumJT,&sumJTINV,CV_SVD);
	
	cvMatMul(&sumJTINV,&Fnet,&mu);
	
	//cout<<"fnet "<<cvmGet(&Fnet,0,0)<<" "<<cvmGet(&Fnet,1,0)<<endl;
	//cout<<cvmGet(&sumJT,0,0)<<" "<<cvmGet(&sumJT,0,1)<<" "<<cvmGet(&sumJT,1,0)<<" "<<cvmGet(&sumJT,1,1)<<endl;
	
	
	//cout<<"mu "<<cvmGet(&mu,0,0)<<" "<<cvmGet(&mu,1,0)<<endl;
	
	posState->estPosition.x = posState->estPosition.x -cvmGet(&mu,0,0);
	posState->estPosition.y = posState->estPosition.y - cvmGet(&mu,1,0);
	//cout<<"estimated position state "<<posState->estPosition.x<<" "<< posState->estPosition.y<<endl;
	
	
	double Tmu_arr[1][2];
	CvMat Tmu=cvMat(1,2,CV_64F,Tmu_arr);
	
	cvTranspose(&mu,&Tmu);
	
	for(int i=0;i<scan_count;i++){
	  cvGetRow(my_points,&Ping_point,i);
	  cvScaleAdd(&Tmu,cvScalar(-1),&Ping_point,&Ping_point);
	}
	
	//******************begin finding center of rotation and torq*********
	
	//	cout<<"finding rotation "<<endl;
	
	
	double c_arr[2][1];
	CvMat c=cvMat(2,1,CV_64F,c_arr);
	cvSetZero(&c);
	
	double Temp_mult_arr[2][2];
	CvMat Temp_mult=cvMat(2,2,CV_64F,Temp_mult_arr);
	
	double TD_arr[2][2];
	CvMat TD=cvMat(2,2,CV_64F,TD_arr);
	
	double point_trans_arr[2][1];
	CvMat point_trans=cvMat(2,1,CV_64F,point_trans_arr);
	
	
	for(int i=0;i<scan_count;i++){
	  
	  //c+=D*sumJTINV*TW[associatedline]*D' *point[i]
	  cvGetRow(my_points,&Ping_point,i);
	  
	  if(line_associations[i]>=0){
		cvMatMul(&D,&sumJTINV,&Temp_mult);
		cvGetCols(TW,&TempTW,line_associations[i]*2 ,line_associations[i]*2+2);
		cvMatMul(&Temp_mult,&TempTW,&Temp_mult);
		cvTranspose(&D,&TD);
		
		cvMatMul(&Temp_mult,&TD,&Temp_mult);
		//transpose point to be multiplied
		cvTranspose(&Ping_point,&point_trans  );
		
		cvMatMul(&Temp_mult,&point_trans,&point_trans);
		
		cvAdd(&point_trans,&c,&c);  
	  }
	}
	
	double trq_net=0;
	double Kt=0;
	
	double trq_arr[1][1];
	CvMat trq=cvMat(1,1,CV_64F,trq_arr);
	
	double d_r_arr[2][1];
	CvMat d_r=cvMat(2,1,CV_64F,d_r_arr);
	
	double d_rT_arr[1][2];
	CvMat d_rT=cvMat(1,2,CV_64F,d_rT_arr);
	
	double r_arr[2][1];
	CvMat r=cvMat(2,1,CV_64F,r_arr);
	
	double rT_arr[1][2];
	CvMat rT=cvMat(1,2,CV_64F,rT_arr);
	
	double rT_twperp_arr[1][2];
	CvMat rT_twperp=cvMat(1,2,CV_64F,rT_twperp_arr);
	
	double Kt_i_arr[1][1];
	CvMat Kt_i=cvMat(1,1,CV_64F,Kt_i_arr);
	
	CvMat TempTWPerp;
	cvInitMatHeader(&TempTWPerp,2,2,CV_64F);
	
	for(int i=0;i<scan_count;i++){
	  if(line_associations[i]>=0){
		cvGetRow(my_points,&Ping_point,i);
		cvTranspose(&Ping_point,&point_trans);
		cvGetCol(&forces,&F_i,i);
		
		//r=Ping_point-c;	  
		cvSub(&point_trans,&c,&r);
		
		//trq=(D*r)'*F_i
		cvMatMul(&D,&r,&d_r);
		cvTranspose(&d_r,&d_rT);
		cvMatMul(&d_rT,&F_i,&trq);
		
		//Kt+=r'*TWPerp_i*r
		cvTranspose(&r,&rT);
		
		cvGetCols(TWPerp,&TempTWPerp,line_associations[i]*2 ,line_associations[i]*2+2);
		
		cvMatMul(&rT,&TempTWPerp,&rT_twperp);
		
		cvMatMul(&rT_twperp,&r,&Kt_i);
		
		Kt+=cvmGet(&Kt_i,0,0);
		
		//trq_net+=trq
		trq_net+=cvmGet(&trq,0,0);
	  }
	}
	
	//sign has been flipped from example to save sign flipping later
	double dpsi_est= -trq_net/Kt;
	//rotate_points dpsi_est, c
	
	//cout<<cvmGet(&c,0,0)<<" "<<cvmGet(&c,1,0)<<" "<<dpsi_est*180/3.14<<endl;
	
	//rotMat=
	// cos(dpsi_est),-sin(dpsi_est)
	// sin(dpsi_est),cos(dpsi_est)
	double rotMat_arr[2][2]={{cos(dpsi_est),-sin(dpsi_est)},{sin(dpsi_est),cos(dpsi_est)}};
	CvMat rotMat=cvMat(2,2,CV_64F,rotMat_arr);
	
	double p_min_c_arr[2][1];
	CvMat p_min_c=cvMat(2,1,CV_64F,p_min_c_arr);
	
	double offset_point_arr[2][1];
	CvMat offset_point=cvMat(2,1,CV_64F,offset_point_arr);
	
	//cout<<"assigning values\n";
	
	//offset all points in the scan by a rotation
	for(int i=0;i<scan_count;i++){
	  //point=rotMat*(point-c)+c
	  cvGetRow(my_points,&Ping_point,i);
	  cvTranspose(&Ping_point,&point_trans);
	  cvSub(&point_trans,&c,&p_min_c);
	  cvMatMul(&rotMat,&p_min_c,&offset_point);
	  cvAdd(&offset_point,&c,&point_trans);
	  cvTranspose(&point_trans,&Ping_point);
	}
	//posState->estPosition=rotMat*(estPosition-c)+c
	//posState->estRotRad+=dpsi_est
	double pos_est_arr[2][1];
	CvMat pos_est=cvMat(2,1,CV_64F,pos_est_arr);
	cvmSet(&pos_est,0,0,posState->estPosition.x);
	cvmSet(&pos_est,1,0,posState->estPosition.y);
	
	cvSub(&pos_est,&c,&pos_est);
	cvMatMul(&rotMat,&pos_est,&pos_est);
	cvAdd(&pos_est,&c,&pos_est);
	posState->estPosition.x=cvmGet(&pos_est,0,0);
	posState->estPosition.y=cvmGet(&pos_est,1,0);
	
	posState->estRotRad+=dpsi_est;
	
	//cout<<"Coord: "<<posState->estPosition.x<<" "<< posState->estPosition.y<<" "<<posState->estRotRad <<endl;
	
  }
}

void LidarLocalizer::associatePoints(const CvMat* lidarScan,int scan_count){
  
  CvMat scan_point;
  cvInitMatHeader(&scan_point,1,2,CV_64F);
  
  CvMat pointRef2;
  cvInitMatHeader(&pointRef2,1,2,CV_64F);
  
  CvMat pointRef3;
  cvInitMatHeader(&pointRef3,1,2,CV_64F);
  
  double vector1_arr[1][2];
  CvMat vector1=cvMat(1,2,CV_64F,vector1_arr);
  
  double vector2_arr[1][2];
  CvMat vector2=cvMat(1,2,CV_64F,vector2_arr);
  
  for(int i=0;i<num_lines;i++){
	ping_count[i]=0;
  }
  
  
  for(int i=0;i<scan_count;i++){
	//extract lidar point
	cvGetRow(lidarScan,&scan_point,i);
	//find distance to all corners
	int too_close=0;
	for(int j=0;j<num_lines;j++){
	  //extract map point
	  cvGetRow(&startxy,&pointRef2,j);  
	  cvSub(&scan_point,&pointRef2,&vector1);
	  
	  double dist=cvNorm(&vector1);
	  if(dist<=corner_reject_rad){
		too_close=1;
		//	cout<<"too close to corner\n";
		break;
	  }
	}
	
	int closest_index=-1;
	double closest_distance=INFINITY;
	if(too_close==0){
	  //if we are not too close to any corners bind to a line
	  for(int j=0;j<num_lines;j++){
		//extract map point
		double dist_to_j =DistanceFromLine(cvmGet(&scan_point,0,0),cvmGet(&scan_point,0,1),cvmGet(&startxy,j,0),cvmGet(&startxy,j,1),cvmGet(&endxy,j,0),cvmGet(&endxy,j,1));
		if(dist_to_j<closest_distance&& max_dist > dist_to_j){
		  closest_distance=dist_to_j;
		  closest_index=j;
		}
	  }
	}
	line_associations[i]=closest_index;
	line_distances[i]=closest_distance;
	if(closest_index>=0){
	  ping_count[closest_index]++;
	}
	//	cout<<closest_distance<<" dist\n";
  }
  
}

//Find the distance from a point to a line
double LidarLocalizer::DistanceFromLine(double cx, double cy, double ax, double ay, double bx, double by){
  double r_numerator = (cx-ax)*(bx-ax) + (cy-ay)*(by-ay);
  double r_denomenator = (bx-ax)*(bx-ax) + (by-ay)*(by-ay);
  double r = r_numerator / r_denomenator;
  
  double px = ax + r*(bx-ax);
  double py = ay + r*(by-ay);
  
  double s =  ((ay-cy)*(bx-ax)-(ax-cx)*(by-ay) ) / r_denomenator;
  double distanceLine = fabs(s)*sqrt(r_denomenator);
  
  if ( (r >= 0) && (r <= 1) ){
	return distanceLine;
  }
  else{
	return INFINITY; 
  }
  
}

void LidarLocalizer::display(PositionState* position,const CvMat* lidarScan,const CvMat* my_points,int scan_count){
  IplImage* img=cvCreateImage(cvSize(500,500),IPL_DEPTH_8U,3);
 // IplImage* img2=cvCreateImage(cvSize(500,500),IPL_DEPTH_8U,3);
  
  CvScalar LineColors [11];
  LineColors[0]=cvScalar(255,255,51);
  LineColors[1]=cvScalar(153,255,51);
  LineColors[2]=cvScalar(255,153,51);
  LineColors[3]=cvScalar(255,51,255);
  LineColors[4]=cvScalar(184,184,0);
  LineColors[5]=cvScalar(184,0,184);
  LineColors[6]=cvScalar(0,0,254);
  LineColors[7]=cvScalar(255,51,51);
  LineColors[8]=cvScalar(51,255,153);
  LineColors[9]=cvScalar(255,51,153);
  LineColors[10]=cvScalar(255,51,51);
  
  
  int yoff=400;
  int xoff=250;
  double meter_2_pixel=10;
  int headingLen=20;
  
  
  for(int i=0;i<num_lines;i++){
	cvLine(img,cvPoint(cvmGet(map,i,0)*meter_2_pixel+xoff,cvmGet(map,i,1)*-meter_2_pixel+yoff),cvPoint(cvmGet(map,i,2)*meter_2_pixel+xoff,cvmGet(map,i,3)*-meter_2_pixel+yoff),LineColors[i],2);
	
  }
  /*
  for(int i=0;i<num_lines;i++){
	cvLine(img2,cvPoint(cvmGet(map,i,0)*meter_2_pixel+xoff,cvmGet(map,i,1)*-meter_2_pixel+yoff),cvPoint(cvmGet(map,i,2)*meter_2_pixel+xoff,cvmGet(map,i,3)*-meter_2_pixel+yoff),LineColors[i],2);
	
  }*/
  
  CvPoint origin=cvPoint(250+position->crioPosition.x*10,400+position->crioPosition.y*-10);
  cvCircle(img,origin,4,cvScalar(0,255,0),-1,8);
  cvLine(img, origin ,cvPoint(cos(-(position->crioRotRad)-3.1415/2)*headingLen+origin.x,sin(-(position->crioRotRad)-3.1415/2)*headingLen+origin.y),cvScalar(0,0,255),2);
  
  /*
  CvPoint neworigin=cvPoint(250+position->estPosition.x*10,400+position->estPosition.y*-10);
  cvCircle(img2,neworigin,4,cvScalar(0,255,0),-1,8);
  cvLine(img2, neworigin ,cvPoint(cos(-(position->estRotRad)-3.1415/2)*headingLen+neworigin.x,sin(-(position->estRotRad)-3.1415/2)*headingLen+neworigin.y),cvScalar(0,0,255),2);
  */
  
  CvPoint curve1;
  for(int i=0;i<scan_count;i++){
	//draw original scan
	curve1.x=int(cvmGet(lidarScan,i,0)*meter_2_pixel)+xoff;
	curve1.y=int(cvmGet(lidarScan,i,1)*-meter_2_pixel)+yoff;
	int lookup=line_associations[i];
	CvScalar color;
	if(lookup>=0){
	  color=LineColors[lookup];
	}
	else{
	  color=cvScalar(255,255,255);
	}
	cvCircle(img,curve1,4,color,-1);
	/*
	//draw new scan
	curve1.x=int(cvmGet(my_points,i,0)*meter_2_pixel)+xoff;
	curve1.y=int(cvmGet(my_points,i,1)*-meter_2_pixel)+yoff;
	lookup=line_associations[i];
	
	cvCircle(img2,curve1,4,color,-1);*/
  }
  
  /*  CvPoint* curveArr[1]={curve1};
  int      nCurvePts[1]={scanCount};
  int      nCurves=1;
  */
  //cvPolyLine(img,curveArr,nCurvePts,nCurves,0,cvScalar(0,0,255));
  
  cvShowImage("keycatcher", img); 
//  cvShowImage("new_pings", img2); 
  
  cvReleaseImage(&img);
 // cvReleaseImage(&img2);
  
}

LidarLocalizer::~LidarLocalizer(){
  
  delete [] line_associations;
  cvReleaseMat(&map);
  cvReleaseMat(&TW);
  cvReleaseMat(&TWPerp);
  cvDestroyWindow( "keycatcher" );
 // cvDestroyWindow( "new_pings" );
}

