/*  
This file is part of Marvel project designed and developed by Ali Jameei 
and Mohammad hossein kazemi in AUT-MAV team from Amirkabir University of 
Technology. 
    Author : Ali Jameei
    E-Mail : celarco.group@gmail.com 
*/ 
    
struct rc_channel {
	int min, trim, max, rev, dz;
};
class radio {
public:
	rc_channel roll, pitch, yaw, throttle, mode;	
	int calc_roll(int roll_cmd);
	int calc_pitch(int pitch_cmd);
	int calc_yaw(int yaw_cmd);
	int calc_throttle(int throttle_cmd);
	int calc_mode(int mode_cmd);
	
};
