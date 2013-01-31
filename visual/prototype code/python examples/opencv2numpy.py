#! /usr/bin/env python

import sys

from opencvnumpyconversion import *
from opencv import cv, highgui
from pylab import imread, imshow, gray, mean
import pylab

# import the necessary things for OpenCV
from opencv.cv import *
from opencv.highgui import *

im = cvLoadImage("molecule.jpg", 1)


a=cv2array(im)

#generates a RGB image, so do
aa=mean(a,2) # to get a 2-D array
pylab.figure(1)
imshow(aa)
gray()
pylab.show()




