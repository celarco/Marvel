/*  
This file is part of Marvel project designed and developed by Ali Jameei 
and Mohammad hossein kazemi in AUT-MAV team from Amirkabir University of 
Technology. 
    Author : Ali Jameei
    E-Mail : celarco.group@gmail.com 
*/

#include <stdlib.h>
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <../include/guidance_pack.h>
#include <../include/pid.h>
#include <ros/ros.h>
#include <Marvel/Server.h>
#include <Marvel/Guidance_Command.h>


// Guidance variables

vertical_mode g_vertical_mode;
horizontal_mode g_horizontal_mode;
heading_mode g_heading_mode;

float v_x_setpoint = 0.0, v_y_setpoint = 0.0, v_z_setpoint = 0.0;
float v_x = 0.0, v_y = 0.0, v_z = 0.0;

float heading_setpoint = 0.0;
float heading = 0.0;

float roll_setpoint = 0.0;
float roll = 0.0;

float pitch_setpoint = 0.0;
float pitch = 0.0;

float x_setpoint = 0.0, y_setpoint = 0.0, z_setpoint = 0.0;
float x = 0.0, y = 0.0, z = 0.0;

float yaw_rate_setpoint = 0.0;
float yaw_rate = 0.0;

bool armed = false;
bool server_ready = false;

pid pid_x, pid_y, pid_z, pid_v_x, pid_v_y, pid_v_z;
pid pid_heading, pid_yaw_rate;

float nominal_hover_throttle = 50.0;

// Flight plan variables

//vector <block_struct> blocks;

unsigned short int current_block_no = 0;

// Server receive callback function

void server_receive_Callback(const Marvel::Server::ConstPtr& msg) {
	armed = msg->armed;
	server_ready = msg->ready;
}

//Reading all parameters from flight plan

void Read_Flight_Plan(){

	//while(fp_dir == ""){}

	FileStorage fs("/home/hojat/WS/sandbox/Marvel/fp.yaml", FileStorage::READ);
	
	int c = 1;
	bool blocks_finished = true;
	stringstream convert;

	//Extracting take off blocks
	while(1){
		blocks_finished = true;

		convert.str("");
		convert << c;
		string block_name_to = "take_off_" + convert.str();
		string block_name_l = "landing_" + convert.str();
		string block_name_ph = "position_hold_" + convert.str(); 

		FileNode take_off = fs[block_name_to];
		FileNode landing = fs[block_name_l];
		FileNode position_hold = fs[block_name_ph];

		if(!take_off.empty()){
			
			cout << block_name_to << "\t";

			FileNodeIterator it = take_off.begin() , it_end = take_off.end();

			for( ; it != it_end; ++it){
				cout << (float)(*it)["height"] << "\t";
				cout << (float)(*it)["speed"] << "\t";
				cout << (string)(*it)["horizontal_mode"] << "\t";
				cout << (float)(*it)["roll"] << "\t";
				cout << (float)(*it)["pitch"] << "\n";
			}	

			blocks_finished = false;
		}

		if(!landing.empty()){
			
			cout << block_name_l << "\t";

			FileNodeIterator it = landing.begin() , it_end = landing.end();

			for( ; it != it_end; ++it){
				cout << (float)(*it)["speed"] << "\t";
				cout << (string)(*it)["horizontal_mode"] << "\t";
				cout << (string)(*it)["heading_mode"] << "\t";
				cout << (string)(*it)["object_dir"] << "\t";
				cout << (string)(*it)["method"] << "\t";
				cout << (int)(*it)["h_min"] << "\t";
				cout << (int)(*it)["h_max"] << "\t";
				cout << (int)(*it)["s_min"] << "\t";
				cout << (int)(*it)["s_max"] << "\t";
				cout << (int)(*it)["v_min"] << "\t";
				cout << (int)(*it)["v_max"] << "\n";
			}	

			blocks_finished = false;
		}

		if(!position_hold.empty()){
			
			cout << block_name_ph << "\t";

			FileNodeIterator it = position_hold.begin() , it_end = position_hold.end();

			for( ; it != it_end; ++it){
				cout << (string)(*it)["heading"] << "\t";
				cout << (string)(*it)["stop_condition"] << "\t";
				cout << (float)(*it)["global_angle"] << "\t";
				cout << (float)(*it)["rate"] << "\t";
				cout << (float)(*it)["turns"] << "\t";
				cout << (float)(*it)["time"] << "\t";
				cout << (float)(*it)["angle"] << "\n";
			}	

			blocks_finished = false;
		}

		if(blocks_finished)
			break;

		c++;

	}

}

// Main program start

int main(int argc, char **argv) {

    Read_Flight_Plan();
    // Ros initialization
   
    ros::init(argc, argv, "guidance_pack");
    ros::NodeHandle n;
    ros::Subscriber sub2 = n.subscribe("server", 1000, server_receive_Callback);
	ros::Publisher pub = n.advertise<Marvel::Guidance_Command>("guidance_pack", 1000);
    Marvel::Guidance_Command guidance_command;
    
	// Wait for the server to be ready
	
	std::cout<<"Waiting for server to be ready...!"<<std::endl;
    
	system("rosrun Marvel server &> /dev/null &");
	while(!server_ready) {
		ros::spinOnce();
	}
	
	std::cout<<"Server ready...!"<<std::endl;

    // Autopilot initialization
	
	guidance_command.arm = 1;
	guidance_command.mode = 0;
	heading_setpoint = heading;
	guidance_command.roll = 0;
    guidance_command.pitch = 0;
	guidance_command.yaw = 0;
	guidance_command.throttle = 50;
    // Guidance loop
    std::cout<<"running...!"<<std::endl;
    while(true) {
        /*
        // Handle function characteristics
        
        if(f[current_function_no].done == true){
			current_function_no ++;
		} 

        switch(f[current_function_no].type) {

        case take_off:
            g_vertical_mode = VERTICAL_CLIMB;
            g_horizontal_mode = HORIZONTAL_HOLD;
            g_heading_mode = HEADING_HOLD;
            v_z_setpoint = f[current_function_no].arg[0];
			height_setpoint = f[current_function_no].arg[1];
			if(height >= height_setpoint) {
				f[current_function_no].done = true;
				g_vertical_mode = VERTICAL_HOLD;
			}
		break;

        case hold_position:
            g_vertical_mode = VERTICAL_HOLD;
            g_horizontal_mode = HORIZONTAL_HOLD;
            g_heading_mode = HEADING_HOLD;
		break;

        case set_heading:
            g_heading_mode = HEADING_RATE;
            heading_setpoint = f[current_function_no].arg[0];
        break;

        case heading_lock:
            g_heading_mode = HEADING_LOCK;
            
        break;

        case heading_unlock:
            g_heading_mode = HEADING_HOLD;
        break;

        case rotate:
            g_heading_mode = HEADING_RATE;
            rate_setpoint = f[current_function_no].arg[0];
            rotate_break_cond = int(f[current_function_no].arg[1]);
        break;

        case move_oa:
            g_vertical_mode = VERTICAL_HOLD;
            g_horizontal_mode = HORIZONTAL_VELOCITY;
            v_x_setpoint = f[current_function_no].arg[0];
            v_y_setpoint = f[current_function_no].arg[1];
            move_oa_break_cond = int(f[current_function_no].arg[2]);
            height_setpoint = f[current_function_no].arg[3];
        break;

        case move:
            g_vertical_mode = VERTICAL_HOLD;
            g_horizontal_mode = HORIZONTAL_VELOCITY;
            v_x_setpoint = f[current_function_no].arg[0];
            v_y_setpoint = f[current_function_no].arg[1];
            move_oa_break_cond = int(f[current_function_no].arg[2]);
            height_setpoint = f[current_function_no].arg[3];
        break;

        case go:
            g_vertical_mode = VERTICAL_LOCK;
            g_horizontal_mode = HORIZONTAL_LOCK;
          
        break;

        case go_oa:
		
            g_vertical_mode = VERTICAL_LOCK;
            g_horizontal_mode = HORIZONTAL_LOCK;
            g_lock_param = lock_param(f[current_function_no].arg[0]);
			
			switch (g_lock_param){
			 
			case WINDOW_DETECTOR:
				//system("roslaunch openni2_launch openni2.launch &> /dev/null &");
				//system("rosrun Marvel window_detector &> /dev/null &");
				//system("rosrun Marvel obstacle_avoidance &> /dev/null &");
				guidance_command.vx_uni = 1.4;
				guidance_command.vy_uni = window_deltax * 0.0004 ;
				guidance_command.vz_uni = window_deltay * 0.0005;
				v_x_setpoint = obstacle_vx_setpoint;
				if (v_x_setpoint > 0.15) v_x_setpoint = 0.15;
				if (v_x_setpoint < -0.15) v_x_setpoint = -0.15;
				v_y_setpoint = obstacle_vy_setpoint;
				if (v_y_setpoint > 0.15) v_y_setpoint = 0.15;
				if (v_y_setpoint < -0.15) v_y_setpoint = -0.15;
				v_z_setpoint = obstacle_vz_setpoint;
				if (v_z_setpoint > 0.15) v_z_setpoint = 0.15;
				if (v_z_setpoint < -0.15) v_z_setpoint = -0.15;
				
			break;
				 
			case FLOWER:
			break;
				 
			case MARKER:
			break;
				 
			case ROPE:
			break; 
				 
			}
		break;
		
        }
		
		// Vertical mode pd
		
        switch(g_vertical_mode) {
        case VERTICAL_HOLD:
			pid_z.set_coeff(0.4,0.02,0);
			guidance_command.throttle = 100 * pid_z.loop_once((height_setpoint - height),0) + nominal_hover_throttle;
        break;

        case VERTICAL_LOCK:
			pid_z.set_coeff(0,0,0.02);
			guidance_command.throttle = 100 * pid_z.loop_once(0,(v_z_setpoint - v_z)) + nominal_hover_throttle;

        break;
        case VERTICAL_CLIMB:
			pid_z.set_coeff(0,0.02,0.25);
			guidance_command.throttle = 100 * pid_z.loop_once(0,(v_z_setpoint - v_z)) + nominal_hover_throttle ;   
		break;
        }
		
		// Horizontal pd
		
		switch(g_horizontal_mode) {
		case HORIZONTAL_HOLD:
			pid_x.set_coeff(0,0,1);
			if(((0 - v_x) >= 0.01) || ((0 - v_x) <= -0.01)) {
				guidance_command.roll = 100 * pid_x.loop_once(0,(0 - v_x));
			}
			else if (((0 - v_x) <= 0.01) || ((0 - v_x) >= -0.01)) {
				guidance_command.roll = 0;
			}
			
			pid_y.set_coeff(0,0,1);
			if(((0 - v_y) >= 0.01) || ((0 - v_y) <= -0.01)) {
				guidance_command.pitch = 100 * pid_y.loop_once(0,(0 - v_y));
			}
			else if (((0 - v_y) <= 0.01) || ((0 - v_y) >= -0.01)) {
				guidance_command.pitch = 0;
			}
		break;
		case HORIZONTAL_LOCK:
			pid_x.set_coeff(0,0,1);
			if(((v_x_setpoint - v_x) >= 0.01) || ((v_x_setpoint - v_x) <= -0.01)) {
				guidance_command.roll = 100 * pid_x.loop_once(0,(v_x_setpoint - v_x));
			}
			else if (((v_x_setpoint - v_x) <= 0.01) || ((v_x_setpoint - v_x) >= -0.01)) {
				guidance_command.roll = 0;
			}
			
			pid_y.set_coeff(0,0,1);
			if(((v_y_setpoint - v_y) >= 0.01) || ((v_y_setpoint - v_y) <= -0.01)) {
				guidance_command.pitch = 100 * pid_y.loop_once(0,(v_y_setpoint - v_y));
			}
			else if (((v_y_setpoint - v_y) <= 0.01) || ((v_y_setpoint - v_y) >= -0.01)) {
				guidance_command.pitch = 0;
			}
		break;
		case HORIZONTAL_VELOCITY:
			
		break;
		}
		
		// Heading pd
		
		switch(g_heading_mode) {
		case HEADING_HOLD:
			pid_h.set_coeff(0.007,0,0);
			guidance_command.yaw = 100 * pid_h.loop_once((heading_setpoint-heading),0);
		break;
		case HEADING_LOCK:
			
		break;
		case HEADING_RATE:
		
		break;
		}
		*/
        //std::cout << current_function_no << std::endl;
        //std::cout << guidance_command.roll << " " << guidance_command.pitch << " " << guidance_command.throttle << " " << guidance_command.yaw << " " << std::endl;
        ros::spinOnce();
        pub.publish(guidance_command); 
    }

    return 0;
}
