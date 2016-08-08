/*  
This file is part of Marvel project designed and developed by Ali Jameei 
and Mohammad hossein kazemi in AUT-MAV team from Amirkabir University of 
Technology. 
    Author : Ali Jameei
    E-Mail : celarco.group@gmail.com 
*/ 
    
#ifndef PID_HEADER
#define PID_HEADER

#include <time.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

class pid {
private:
    float gain_kp, gain_kd, gain_ki;
	double err_sum, max_err_sum, dt;
	struct timespec now;
	struct timespec end;
	void calc_dt();
public:
    pid();
	void set_coeff(float kp, float ki, float kd);
    float loop_once(float err, float err_dot);
	void reset_integrator();
	void set_max_sum(double max);
};

#endif
