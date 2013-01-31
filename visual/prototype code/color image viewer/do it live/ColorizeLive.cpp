#include <stdio.h>
#include <time.h>
#include "cv.h"
#include "highgui.h"

const int GBRG = 0;
const int GRBG = 1;
const int BGGR = 2;
const int RGGB = 3;

static unsigned char * convR(NULL);
static unsigned char * convG(NULL);
static unsigned char * convB(NULL);

void init(){
	convR = new unsigned char[256];
	convG = new unsigned char[256];
	convB = new unsigned char[256];
	for(unsigned int i = 0;i < 256;++i)
	{
		convR[i] = (unsigned char)floor(((double)i)*0.299);
		convG[i] = (unsigned char)floor(((double)i)*0.587);
		convB[i] = (unsigned char)floor(((double)i)*0.114);
	}
}



//0.299*R + 0.587*G + 0.114*B
void BayerToGray(IplImage* source,IplImage *destination, int grid_type)
{
	if(convR==NULL){
		init();
	}
	unsigned char *src=(unsigned char*)source->imageData;
	unsigned char *dest=(unsigned char*)destination->imageData;
	
	int sx=source->width;
	int sy=source->height;
	int channels=source->nChannels;
	
 	register int i,j,bi,bj,gi1,gj1,gi2,gj2,jp,jn,ip,in;
	
  //configure grid alignment things
  if(grid_type==0){
    bi=0;
    bj=1;
    gi1=0;
    gj1=0;
    gi2=1;
    gj2=1;
  }
	else if(grid_type==1){
	  bi=1;
    bj=0;
    gi1=1;
    gj1=1;
    gi2=0;
    gj2=0;
  }
	else if(grid_type==2){
    bi=0;
    bj=0;
    gi1=0;
    gj1=1;
    gi2=1;
    gj2=0;
  }
	else if(grid_type==3){
	  bi=1;
    bj=1;
    gi1=1;
    gj1=0;
    gi2=0;
    gj2=1;
  }
    
    
  
  for (i=0;i<sy;i++)
	{
    for (j=0;j<sx;j++)
		{
			if(j>0)
          jp=j-1;
      else
          jp=j+1;
      
      if(j<sx-1)
          jn=j+1;
      else
          jn=j-1;
      
      if(i>0)
          ip=i-1;
      else
          ip=i+1;
      
      if(i<sy-1)
          in=i+1;
      else
          in=i-1;
      
			if(i%2==bi &&j%2==bj){
				//we are on a blue square
				((uchar*)(dest + destination->widthStep*(i)))[j]=convB[((uchar*)(src + source->widthStep*(i)))[j*channels]];
				//green
				((uchar*)(dest + destination->widthStep*(i)))[j]+= convG[\
				(((uchar*)(src + source->widthStep*(in)))[(j)*channels]+ \
				((uchar*)(src + source->widthStep*(ip)))[(j)*channels]+ \
				((uchar*)(src + source->widthStep*(i)))[(jn)*channels]+ \
				((uchar*)(src + source->widthStep*(i)))[(jp)*channels])/4];
				//red
				((uchar*)(dest + destination->widthStep*(i)))[j]+= convR[\
				(((uchar*)(src + source->widthStep*(in)))[(jn)*channels]+ \
				((uchar*)(src + source->widthStep*(in)))[(jp)*channels]+ \
				((uchar*)(src + source->widthStep*(ip)))[(jp)*channels]+ \
				((uchar*)(src + source->widthStep*(ip)))[(jn)*channels])/4];
				
			}
			else if(i%2==gi1 && j%2==gj1 || i%2==gi2 && j%2==gj2){
				//we are on a green square
				//green
				((uchar*)(dest + destination->widthStep*(i)))[j]=convG[((uchar*)(src + source->widthStep*(i)))[j*channels]];
				
				if(j%2==gj2){
      	//vertically blue aligned
      		//red
      		((uchar*)(dest + destination->widthStep*(i)))[j]+=convR[\
      		(((uchar*)(src + source->widthStep*(i)))[jn*channels]+\
      		((uchar*)(src + source->widthStep*(i)))[jp*channels])/2];
      		
      		//blue
      		((uchar*)(dest + destination->widthStep*(i)))[j]+=convB[\
      		(((uchar*)(src + source->widthStep*(in)))[j*channels]+\
      		((uchar*)(src + source->widthStep*(ip)))[j*channels])/2];
				}
				else{
					//red
      		((uchar*)(dest + destination->widthStep*(i)))[j]+=convR[\
      		(((uchar*)(src + source->widthStep*(in)))[j*channels]+\
      		((uchar*)(src + source->widthStep*(ip)))[j*channels])/2];
      		
      		//blue
      		((uchar*)(dest + destination->widthStep*(i)))[j]+=convB[\
      		(((uchar*)(src + source->widthStep*(i)))[jn*channels]+\
      		((uchar*)(src + source->widthStep*(i)))[jp*channels])/2];
				}
			}
			else{
				//we are on a red square
				
				//change red
				((uchar*)(dest + destination->widthStep*(i)))[j]=convR[((uchar*)(src + source->widthStep*(i)))[j*channels]];
				
				//change green
				((uchar*)(dest + destination->widthStep*(i)))[j]+=convG[ \
				(((uchar*)(src + source->widthStep*(in)))[(j)*channels]+ \
				((uchar*)(src + source->widthStep*(ip)))[(j)*channels]+ \
				((uchar*)(src + source->widthStep*(i)))[(jn)*channels]+ \
				((uchar*)(src + source->widthStep*(i)))[(jp)*channels])/4];
				
				//change blue
				((uchar*)(dest + destination->widthStep*(i)))[j]+=convB[ \
				(((uchar*)(src + source->widthStep*(in)))[(jn)*channels]+ \
				((uchar*)(src + source->widthStep*(in)))[(jp)*channels]+ \
				((uchar*)(src + source->widthStep*(ip)))[(jp)*channels]+ \
				((uchar*)(src + source->widthStep*(ip)))[(jn)*channels])/4];
			}
  	}
  }	
}

void BayerToColor(IplImage* source,IplImage *destination, int grid_type )
{
	unsigned char *src=(unsigned char*)source->imageData;
	unsigned char *dest=(unsigned char*)destination->imageData;
	int sx=source->width;
	int sy=source->height;
	
	int channels=source->nChannels;
	
  register int i,j,bi,bj,gi1,gj1,gi2,gj2,jp,jn,ip,in;
  
  //configure grid alignment things
  if(grid_type==0){
    bi=0;
    bj=1;
    gi1=0;
    gj1=0;
    gi2=1;
    gj2=1;
  }
	else if(grid_type==1){
	  bi=1;
    bj=0;
    gi1=1;
    gj1=1;
    gi2=0;
    gj2=0;
  }
	else if(grid_type==2){
    bi=0;
    bj=0;
    gi1=0;
    gj1=1;
    gi2=1;
    gj2=0;
  }
	else if(grid_type==3){
	  bi=1;
    bj=1;
    gi1=1;
    gj1=0;
    gi2=0;
    gj2=1;
  }
    
    
  
  for (i=0;i<sy;i++)
	{
    for (j=0;j<sx;j++)
		{
			if(j>0)
          jp=j-1;
      else
          jp=j+1;
      
      if(j<sx-1)
          jn=j+1;
      else
          jn=j-1;
      
      if(i>0)
          ip=i-1;
      else
          ip=i+1;
      
      if(i<sy-1)
          in=i+1;
      else
          in=i-1;
      
			if(i%2==bi &&j%2==bj){
				//we are on a blue square
				((uchar*)(dest + destination->widthStep*(i)))[j*3]=((uchar*)(src + source->widthStep*(i)))[j*channels];
				//green
				((uchar*)(dest + destination->widthStep*(i)))[j*3+1]= \
				(((uchar*)(src + source->widthStep*(in)))[(j)*channels]+ \
				((uchar*)(src + source->widthStep*(ip)))[(j)*channels]+ \
				((uchar*)(src + source->widthStep*(i)))[(jn)*channels]+ \
				((uchar*)(src + source->widthStep*(i)))[(jp)*channels])/4;
				//red
				((uchar*)(dest + destination->widthStep*(i)))[j*3+2]= \
				(((uchar*)(src + source->widthStep*(in)))[(jn)*channels]+ \
				((uchar*)(src + source->widthStep*(in)))[(jp)*channels]+ \
				((uchar*)(src + source->widthStep*(ip)))[(jp)*channels]+ \
				((uchar*)(src + source->widthStep*(ip)))[(jn)*channels])/4;
				
			}
			else if(i%2==gi1 && j%2==gj1 || i%2==gi2 && j%2==gj2){
				//we are on a green square
				//green
				((uchar*)(dest + destination->widthStep*(i)))[j*3+1]=((uchar*)(src + source->widthStep*(i)))[j*channels];
				
				if(j%2==gj2){
      	//vertically blue aligned
      		//red
      		((uchar*)(dest + destination->widthStep*(i)))[j*3+2]=\
      		(((uchar*)(src + source->widthStep*(i)))[jn*channels]+\
      		((uchar*)(src + source->widthStep*(i)))[jp*channels])/2;
      		
      		//blue
      		((uchar*)(dest + destination->widthStep*(i)))[j*3]=\
      		(((uchar*)(src + source->widthStep*(in)))[j*channels]+\
      		((uchar*)(src + source->widthStep*(ip)))[j*channels])/2;
				}
				else{
					//red
      		((uchar*)(dest + destination->widthStep*(i)))[j*3+2]=\
      		(((uchar*)(src + source->widthStep*(in)))[j*channels]+\
      		((uchar*)(src + source->widthStep*(ip)))[j*channels])/2;
      		
      		//blue
      		((uchar*)(dest + destination->widthStep*(i)))[j*3]=\
      		(((uchar*)(src + source->widthStep*(i)))[jn*channels]+\
      		((uchar*)(src + source->widthStep*(i)))[jp*channels])/2;
				}
			}
			else{
				//we are on a red square
				
				//change red
				((uchar*)(dest + destination->widthStep*(i)))[j*3+2]=((uchar*)(src + source->widthStep*(i)))[j*channels];
				
				//change green
				((uchar*)(dest + destination->widthStep*(i)))[j*3+1]= \
				(((uchar*)(src + source->widthStep*(in)))[(j)*channels]+ \
				((uchar*)(src + source->widthStep*(ip)))[(j)*channels]+ \
				((uchar*)(src + source->widthStep*(i)))[(jn)*channels]+ \
				((uchar*)(src + source->widthStep*(i)))[(jp)*channels])/4;
				
				//change blue
				((uchar*)(dest + destination->widthStep*(i)))[j*3]= \
				(((uchar*)(src + source->widthStep*(in)))[(jn)*channels]+ \
				((uchar*)(src + source->widthStep*(in)))[(jp)*channels]+ \
				((uchar*)(src + source->widthStep*(ip)))[(jp)*channels]+ \
				((uchar*)(src + source->widthStep*(ip)))[(jn)*channels])/4;
			}
  	}
  }	
}


int main( int argc, char** argv )
{
	init();
	//clock_t start=clock();
	//clock_t end=clock();
	
	CvCapture* capture = cvCaptureFromCAM( CV_CAP_ANY );
  if( !capture ) {
    fprintf( stderr, "ERROR: capture is NULL \n" );
    getchar();
    return -1;
  }
  
  
	IplImage* frame = cvQueryFrame( capture );
	
	/* create a window */ 
	cvNamedWindow( "Colored image", CV_WINDOW_AUTOSIZE );   
	IplImage *colored_image=cvCreateImage(cvSize(frame->width,frame->height),8,3);
	
	
	/* create a window */ 
//	cvNamedWindow( "BW image", CV_WINDOW_AUTOSIZE );
//  IplImage *bw_image=cvCreateImage(cvSize(frame->width,frame->height),8,1);
	
	
	// Show the image captured from the camera in the window and repeat
  while( 1 ) {
 // start=clock();
  
    // Get one frame
    IplImage* frame = cvQueryFrame( capture );
    if( !frame ) {
      fprintf( stderr, "ERROR: frame is null...\n" );
      getchar();
      break;
    }
    
		BayerToColor(frame,colored_image,GBRG);
	//	BayerToGray(frame,bw_image,GBRG);


		/* display the image */  
		//cvShowImage( "Colored image", frame );
		cvShowImage( "Colored image", colored_image );
//    cvShowImage( "BW image", bw_image );


   
    // Do not release the frame!
		
    //If ESC key pressed, Key=0x10001B under OpenCV 0.9.7(linux version),
    //remove higher bits using AND operator
    int key=cvWaitKey(2);   
    if( (key & 255) == 27 ) break;
    
    
    //end=clock();
    //printf("%f\n",(double)(end-start));
  }


	/* free memory */
	cvDestroyWindow( "Colored image" );
//	cvDestroyWindow( "BW image");
	cvReleaseImage( &colored_image );
//	cvReleaseImage( &bw_image );
  cvReleaseCapture( &capture );
	return 0;
}

