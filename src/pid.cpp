/*  
This file is part of Marvel project designed and developed by Ali Jameei 
and Mohammad hossein kazemi in AUT-MAV team from Amirkabir University of 
Technology. 
    Author : Ali Jameei
    E-Mail : celarco.group@gmail.com 
*/

#include <../include/pid.h>

void pid::set_coeff(float kp, float ki, float kd) {
    gain_kp = kp;
	gain_ki = ki;
    gain_kd = kd;
}

float pid::loop_once(float err, float err_dot) {
    calc_dt();
	err_sum += err * dt;
	if((err_sum > max_err_sum) || (err_sum < (-1 * max_err_sum))) {
		err_sum = max_err_sum;
	}
	float command = gain_kp * err + gain_kd * err_dot + gain_ki * err_sum;
    return command;
}

void pid::reset_integrator() {
	err_sum = 0;
}

void pid::set_max_sum(double max) {
	max_err_sum = max;
}

void pid::calc_dt () {
	clock_gettime(CLOCK_REALTIME, &now);
	dt = (double)((now.tv_sec + now.tv_nsec *1e-9) - (double)(end.tv_sec + end.tv_nsec *1e-9));  
	clock_gettime(CLOCK_REALTIME, &end);
}

pid::pid() {
	dt = 0;
	gain_kp = 0;
	gain_ki = 0;
	gain_kd = 0;
	err_sum = 0;
}