
#ifndef _birds_eye_calibrator_h
#define _birds_eye_calibrator_h
int birdseyecalibrator(int board_w, int board_h, const char * intrinsic_path, const char * distortion_path, float cm_grid_width, float cm_per_pixel, float cm_in_front_of_robot, int image_width, int image_height);

#endif

