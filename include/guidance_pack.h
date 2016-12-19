/*  
This file is part of Marvel project designed and developed by Ali Jameei 
and Mohammad hossein kazemi in AUT-MAV team from Amirkabir University of 
Technology. 
    Author : Ali Jameei
    E-Mail : celarco.group@gmail.com 
*/ 
    
#ifndef GUIDANCE_PACK_HEADER
#define GUIDANCE_PACK_HEADER

#include <string>
#include <sstream>
#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

void Read_Flight_Plan();
string fp_dir;

enum vertical_mode {
    VERTICAL_POS,
    VERTICAL_VELOCITY
};

enum horizontal_mode {
    HORIZONTAL_POS,
    HORIZONTAL_VELOCITY,
    HORIZONTAL_ATTITUDE,
    HORIZONTAL_PRECISION
};

enum heading_mode {
    HEADING_RATE,
    HEADING_ANGLE,
    HEADING_FREE
};

enum block_type { 
    BLOCK_TAKE_OFF = 1,
    BLOCK_LAND,
    BLOCK_POSITION_HOLD
};
//
//enum take_off_horizontal_mode {
//	HOLD_POS,
//	HOLD_ATTITUDE
//}
//enum take_off_heading_mode {
//	ZERO_RATE,
//	HOLD_ANGLE,
//	FREE
//}
//struct take_off_block{
//	float height = 0.0;
//	float climb_rate = 0.0;
//	take_off_horizontal_mode take_off_h_mode;
//	take_off_heading_mode take_off_hdg_mode;
//	
//}
struct block_struct {
    block_type type;
    
    vertical_mode v_mode;
    horizontal_mode h_mode;
    heading_mode hdg_mode;

	float v_x_setpoint = 0.0, v_y_setpoint = 0.0, v_z_setpoint = 0.0;
	float x_setpoint = 0.0, y_setpoint = 0.0, z_setpoint = 0.0; 

	float heading_setpoint = 0.0;
	float roll_setpoint = 0.0;
	float pitch_setpoint = 0.0;
		
	float yaw_rate_setpoint = 0.0;
};


#endif
