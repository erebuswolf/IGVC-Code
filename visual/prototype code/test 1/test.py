#! /usr/bin/env python

import sys

# import the necessary things for OpenCV
from opencv import cv
from opencv import highgui

im = highgui.cvLoadImage("molecule.jpg", 1)

highgui.cvNamedWindow( "image", highgui.CV_WINDOW_AUTOSIZE)
highgui.cvShowImage( "image", im )

highgui.cvWaitKey(0)

highgui.cvDestroyWindow( "image" )


