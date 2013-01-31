#! /usr/bin/env python
from opencv import cv, highgui
from opencv.cv import *
from opencv.highgui import *

im = cvLoadImage("molecule.jpg", 1)
dst = cvCreateImage(cvGetSize(im), IPL_DEPTH_16S, 3);
laplace = cvLaplace(im, dst)

cvConvertScaleAbs( dst, im, 1, 0 );

highgui.cvNamedWindow( "image", highgui.CV_WINDOW_AUTOSIZE)
highgui.cvShowImage( "image", dst )

highgui.cvWaitKey(0)

highgui.cvDestroyWindow( "image" )


