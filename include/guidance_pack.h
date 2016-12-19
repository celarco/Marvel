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

struct h_detection {
	string obj_dir;
	string method;
}

struct color_tracking{
	int h_min;
	int h_max;
	int s_min;
	int s_max;
	int v_min;
	int v_max;
}

struct object_tracking {
	string obj_dir;
	string method;
	int angle;
}

struct block_struct {
    block_type type;
    
    vertical_mode v_mode;
    horizontal_mode h_mode;
    heading_mode hdg_mode;
	
	h_detection h_detect;
	color_tracking color_track;
	object_tracking obj_track;

	float v_x_setpoint = 0.0, v_y_setpoint = 0.0, v_z_setpoint = 0.0;
	float x_setpoint = 0.0, y_setpoint = 0.0, z_setpoint = 0.0; 
	float delta_x_setpoint = 0.0, delta_y_setpoint = 0.0, delta_z_setpoint = 0.0; 

	float heading_setpoint = 0.0;
	float delta_heading_setpoint = 0.0;
	
	float roll_setpoint = 0.0;
	float pitch_setpoint = 0.0;
	float delta_roll_setpoint = 0.0;
	float delta_pitch_setpoint = 0.0;
	
	float psi_setpoint = 0.0;
	float delta_psi_setpoint = 0.0;
		
	float yaw_rate_setpoint = 0.0;
	float delta_yaw_rate_setpoint = 0.0;
};


#endif
