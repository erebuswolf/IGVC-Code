
#include "convertgps.h"
CvPoint2D32f convertgps(float latitude, float longitude){
  CvPoint2D32f point;
  point.y=(latitude-42.6778389)*111063.5799731039; 
  point.x=(longitude -(-83.1952306))*81968 ;
  return point;
}