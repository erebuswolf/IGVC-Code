#include <stdio.h>
#include "cv.h"
#include "highgui.h"

int main( int argc, char** argv )
{
	/* data structure for the image */
	IplImage *img = 0;

	/* check for supplied argument */
	if( argc < 2 ) {
		fprintf( stderr, "Usage: loadimg <filename>\n" );
		return 1;
	}
	/* load the image,
	use CV_LOAD_IMAGE_GRAYSCALE to load the image in grayscale */
	img = cvLoadImage( argv[1], CV_LOAD_IMAGE_COLOR );

	/* always check */
	if( img == 0 ) {
		fprintf( stderr, "Cannot load file %s!\n", argv[1] );
		return 1;
	}

	/* create a window */ 
	cvNamedWindow( "image", CV_WINDOW_AUTOSIZE );


	CvPoint  curve1[]={10,10,  10,100,  100,100,  100,10};
	CvPoint  curve2[]={30,30,  30,130,  130,130,  130,30,  150,10};
	CvPoint* curveArr[2]={curve1, curve2};
	int      nCurvePts[2]={4,5};
	int      nCurves=2;
	int      isCurveClosed=1;
	int      lineWidth=15;

	cvPolyLine(img,curveArr,nCurvePts,nCurves,isCurveClosed,cvScalar(0,255,255),lineWidth);

	/* display the image */  
	cvShowImage( "image", img );

	/* wait until user press a key */
	cvWaitKey(0);

	/* free memory */
	cvDestroyWindow( "image" );

	cvReleaseImage( &img );
	return 0;
}

