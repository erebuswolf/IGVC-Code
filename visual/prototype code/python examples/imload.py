#!/usr/local/bin/python

from pylab import imread, imshow, gray, mean
import pylab
a = imread('molecule.jpg')
#generates a RGB image, so do
aa=mean(a,2) # to get a 2-D array
pylab.figure(1)
imshow(aa)
gray()
pylab.show()
