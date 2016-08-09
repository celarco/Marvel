/*  
This file is part of Marvel project designed and developed by Ali Jameei 
and Mohammad hossein kazemi in AUT-MAV team from Amirkabir University of 
Technology. 
    Author : Ali Jameei
    E-Mail : celarco.group@gmail.com 
*/

#include <radio.h> 
int radio::calc_roll(int roll_cmd) {
	int range = roll.max - roll.min;
	float coeff = range / 200;
	int result = roll.trim + coeff * roll_cmd;
	return result;
}

int radio::calc_pitch(int pitch_cmd) {
	int range = pitch.max - pitch.min;
	float coeff = range / 200;
	int result = pitch.trim + coeff * pitch_cmd;
	return result;
}

int radio::calc_yaw(int yaw_cmd) {
	int range = yaw.max - yaw.min;
	float coeff = range / 200;
	int result = yaw.trim + coeff * yaw_cmd;
	return result;
}

int radio::calc_throttle(int throttle_cmd) {
	int range = throttle.max - throttle.min;
	float coeff = range / 100;
	int result = throttle.trim + coeff * throttle_cmd;
	return result;
}

int radio::calc_mode(int mode_cmd) {
	int range = mode.max - mode.min;
	float coeff = range / 100;
	int result = mode.trim + coeff * mode_cmd;
	return result;
}