#include "LidarLocalizer.h"

int main(){
  PositionState pos;
  //when the rotation is zero there is a weird cosin sin issue that makes the computer
  //think the line should be going up instead of to the right.
  pos.crioRotRad=9;
  pos.crioPosition.x=1;
  pos.crioPosition.y=16;
  
  LidarLocalizer localizer;
  localizer.loadPresetMap();
  
  localizer.estimatePosition(&pos);

  return 0;
}