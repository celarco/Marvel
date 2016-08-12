/*  
This file is part of Marvel project designed and developed by Ali Jameei 
and Mohammad hossein kazemi in AUT-MAV team from Amirkabir University of 
Technology. 
    Author : Ali Jameei
    E-Mail : celarco.group@gmail.com 
*/

#include <boost/asio.hpp>
#include <iostream>
#include <pixhawk/mavlink.h>
#include "ros/ros.h"
#include <Marvel/Autopilot.h>
#include <Marvel/Guidance_Command.h>
#include <radio.h>

// Initializing boost

using namespace boost;
asio::io_service ios;
asio::serial_port port(ios);

// Message initialization

Marvel::Autopilot autopilot_msg;
Marvel::Guidance_Command guidance_msg;

// Global variables

bool last_arm_status = 0;
bool current_arm_status = 0;
int roll = 0, pitch = 0, yaw = 0, throttle = 0;
int mode = 0;
radio rc;

// Guidance commands bound

float bound(float command) {
	float c;
	c = command;
	if(command > 90) c = 90.0;
	if(command < -90) c = -90.0;
	return c;
}

// Mavlink message receive and handle function

void msg_receive(uint8_t c) {
    mavlink_message_t msg;
    mavlink_status_t status;
    
    // Try to get a new message
    
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
    {   
        // Handle message
        
        // std::cout << int(msg.msgid) <<std::endl;
		switch(msg.msgid)
        {   case MAVLINK_MSG_ID_HEARTBEAT:
				mavlink_heartbeat_t heartbeat;
				mavlink_msg_heartbeat_decode(&msg, &heartbeat);
				if(int(heartbeat.base_mode) == 81) autopilot_msg.armed = false;
				if(int(heartbeat.base_mode) == 209) autopilot_msg.armed = true;
			break;
            case MAVLINK_MSG_ID_ATTITUDE:
				mavlink_attitude_t attitude_msg;
				mavlink_msg_attitude_decode(&msg, &attitude_msg);
				autopilot_msg.rate = attitude_msg.yawspeed * 180 / 3.1415;
				autopilot_msg.heading = attitude_msg.yaw * 180 / 3.1415;
			break;
			case MAVLINK_MSG_ID_VFR_HUD:
				mavlink_vfr_hud_t vfr_msg;
				mavlink_msg_vfr_hud_decode(&msg, &vfr_msg);
				autopilot_msg.climb = vfr_msg.climb;
			break;
			case MAVLINK_MSG_ID_PARAM_VALUE:
				mavlink_param_value_t param_value_msg;
				mavlink_msg_param_value_decode(&msg, &param_value_msg);
				std::cout << int(param_value_msg.param_index) << std::endl;
				std::cout << mavlink_msg_param_value_get_param_value(&msg) << std::endl;				
				switch(int(param_value_msg.param_index)) {
					
					// Extracting radio roll parameters
					
					case 70:
					rc.roll.min = int(mavlink_msg_param_value_get_param_value(&msg));
					break;
					
					case 71:
					rc.roll.trim = int(mavlink_msg_param_value_get_param_value(&msg));
					break;
					
					case 72:
					rc.roll.max = int(mavlink_msg_param_value_get_param_value(&msg));
					break;
					
					case 73:
					rc.roll.rev = int(mavlink_msg_param_value_get_param_value(&msg));
					break;
					
					case 74:
					rc.roll.dz = int(mavlink_msg_param_value_get_param_value(&msg));
					break;
					
					// Extracting radio pitch parameters
					
					case 75:
					rc.pitch.min = int(mavlink_msg_param_value_get_param_value(&msg));
					break;
					
					case 76:
					rc.pitch.trim = int(mavlink_msg_param_value_get_param_value(&msg));
					break;
					
					case 77:
					rc.pitch.max = int(mavlink_msg_param_value_get_param_value(&msg));
					break;
					
					case 78:
					rc.pitch.rev = int(mavlink_msg_param_value_get_param_value(&msg));
					break;
					
					case 79:
					rc.pitch.dz = int(mavlink_msg_param_value_get_param_value(&msg));
					break;
					
					// Extracting radio throttle parameters
					
					case 80:
					rc.throttle.min = int(mavlink_msg_param_value_get_param_value(&msg));
					break;
					
					case 81:
					rc.throttle.trim = int(mavlink_msg_param_value_get_param_value(&msg));
					break;
					
					case 82:
					rc.throttle.max = int(mavlink_msg_param_value_get_param_value(&msg));
					break;
					
					case 83:
					rc.throttle.rev = int(mavlink_msg_param_value_get_param_value(&msg));
					break;
					
					case 84:
					rc.throttle.dz = int(mavlink_msg_param_value_get_param_value(&msg));
					break;
					
					// Extracting radio yaw parameters
					
					case 85:
					rc.yaw.min = int(mavlink_msg_param_value_get_param_value(&msg));
					break;
					
					case 86:
					rc.yaw.trim = int(mavlink_msg_param_value_get_param_value(&msg));
					break;
					
					case 87:
					rc.yaw.max = int(mavlink_msg_param_value_get_param_value(&msg));
					break;
					
					case 88:
					rc.yaw.rev = int(mavlink_msg_param_value_get_param_value(&msg));
					break;
					
					case 89:
					rc.yaw.dz = int(mavlink_msg_param_value_get_param_value(&msg));
					break;
					
					// Extracting radio mode parameters
					
					case 90:
					rc.mode.min = int(mavlink_msg_param_value_get_param_value(&msg));
					break;
					
					case 91:
					rc.mode.trim = int(mavlink_msg_param_value_get_param_value(&msg));
					break;
					
					case 92:
					rc.mode.max = int(mavlink_msg_param_value_get_param_value(&msg));
					break;
					
					case 93:
					rc.mode.rev = int(mavlink_msg_param_value_get_param_value(&msg));
					break;
					
					case 94:
					rc.mode.dz = int(mavlink_msg_param_value_get_param_value(&msg));
					break;
					
					case 500:
					std::cout << "Parameters received...!" << std::endl;
					autopilot_msg.ready = true;
					break;
				}
			break;
        }
    }
}

// Mavlink radio message send function

void msg_send_radio(float throttle, float roll, float pitch, float yaw) {
	if(autopilot_msg.armed == true) {
		
		// Command initializtion
		
		mavlink_rc_channels_override_t radio_commands;
		radio_commands.target_system = 1;
		radio_commands.target_component = MAV_COMP_ID_SYSTEM_CONTROL;
		radio_commands.chan1_raw = roll;
		radio_commands.chan2_raw = pitch;
		radio_commands.chan3_raw = throttle;
		radio_commands.chan4_raw = yaw;
		radio_commands.chan5_raw = mode; 
		radio_commands.chan6_raw = 0; 
		radio_commands.chan7_raw = 0;
		radio_commands.chan8_raw = 0; 
		
		// Message pack and send
		
		mavlink_message_t msg;
		uint8_t buf[MAVLINK_MAX_PACKET_LEN];
		mavlink_msg_rc_channels_override_encode(255, 0, &msg, &radio_commands);
		unsigned len = mavlink_msg_to_send_buffer(buf, &msg);
		asio::write(port, asio::buffer(buf,len));	
	}
	if(autopilot_msg.armed == false) {
		
		// Command initializtion
		
		mavlink_rc_channels_override_t radio_commands;
		radio_commands.target_system = 1;
		radio_commands.target_component = MAV_COMP_ID_SYSTEM_CONTROL;
		radio_commands.chan1_raw = 0;
		radio_commands.chan2_raw = 0;
		radio_commands.chan3_raw = 0;
		radio_commands.chan4_raw = 0;
		radio_commands.chan5_raw = 0; 
		radio_commands.chan6_raw = 0; 
		radio_commands.chan7_raw = 0;
		radio_commands.chan8_raw = 0; 
		
		// Message pack and send
		
		mavlink_message_t msg;
		uint8_t buf[MAVLINK_MAX_PACKET_LEN];
		mavlink_msg_rc_channels_override_encode(255, 0, &msg, &radio_commands);
		unsigned len = mavlink_msg_to_send_buffer(buf, &msg);
		asio::write(port, asio::buffer(buf,len));	
	} 
}
 
// Radio send timer callback function

void msg_send_radio_callback(const ros::TimerEvent&)
{
	msg_send_radio(throttle, roll, pitch, yaw);
}

// Mavlink arm message send function

void msg_send_arm() {
	
	// Command initializtion
	
	mavlink_command_long_t arm_command_msg;
	arm_command_msg.command = MAV_CMD_COMPONENT_ARM_DISARM;
    arm_command_msg.target_system = 1;
    arm_command_msg.target_component = MAV_COMP_ID_SYSTEM_CONTROL;
    arm_command_msg.confirmation = 0;
    arm_command_msg.param1 = 1;
    arm_command_msg.param2 = 0;
    arm_command_msg.param3 = 0;
    arm_command_msg.param4 = 0;
    arm_command_msg.param5 = 0;
    arm_command_msg.param6 = 0;
    arm_command_msg.param7 = 0;
	
	// Message pack and send
	 
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	mavlink_msg_command_long_encode(255, 0, &msg, &arm_command_msg);
	unsigned len = mavlink_msg_to_send_buffer(buf, &msg);
	asio::write(port, asio::buffer(buf,len));
}

// Mavlink disarm message send function

void msg_send_disarm() {
	
	// Command initializtion
	
	mavlink_command_long_t arm_command_msg;
	arm_command_msg.command = MAV_CMD_COMPONENT_ARM_DISARM;
    arm_command_msg.target_system = 1;
    arm_command_msg.target_component = MAV_COMP_ID_SYSTEM_CONTROL;
    arm_command_msg.confirmation = 0;
    arm_command_msg.param1 = 0;
    arm_command_msg.param2 = 0;
    arm_command_msg.param3 = 0;
    arm_command_msg.param4 = 0;
    arm_command_msg.param5 = 0;
    arm_command_msg.param6 = 0;
    arm_command_msg.param7 = 0;
	
	// Message pack and send
	
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];	
	mavlink_msg_command_long_encode(255, 0, &msg, &arm_command_msg);
	unsigned len = mavlink_msg_to_send_buffer(buf, &msg);
	asio::write(port, asio::buffer(buf,len));
}

// Mavlink heartbeat message send function

void msg_send_heartbeat() {
	
	// Command initializtion

	mavlink_heartbeat_t heartbeat_msg;
	heartbeat_msg.type = MAV_TYPE_GCS;
	heartbeat_msg.autopilot = MAV_AUTOPILOT_INVALID;
	heartbeat_msg.base_mode = 0;
	heartbeat_msg.custom_mode = 0;
	heartbeat_msg.system_status = 0;
	heartbeat_msg.mavlink_version = 3;
	
	// Message pack and send
	
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	mavlink_msg_heartbeat_encode(255, 0, &msg, &heartbeat_msg);
	unsigned len = mavlink_msg_to_send_buffer(buf, &msg);
	asio::write(port, asio::buffer(buf,len));
}

// Heartbeat send timer callback function

void msg_send_heartbeat_callback(const ros::TimerEvent&)
{
	msg_send_heartbeat();
}

// Mavlink request parameter message send fucntion

void msg_send_request_param() {
	
	// Command initializtion
	
	mavlink_param_request_list_t request_param_list_msg;
	request_param_list_msg.target_system = 1;
	request_param_list_msg.target_component = MAV_COMP_ID_SYSTEM_CONTROL;	 
	
	// Message pack and send
	
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];	
	mavlink_msg_param_request_list_encode(255, 0, &msg, &request_param_list_msg);
	unsigned len = mavlink_msg_to_send_buffer(buf, &msg);
	asio::write(port, asio::buffer(buf,len));	
}

// Mavlink request data stream function

void msg_send_request_stream() {
	
	// Command initializtion
	
	mavlink_request_data_stream_t request_stream_msg;
	request_stream_msg.target_system = 1;
	request_stream_msg.target_component = MAV_COMP_ID_SYSTEM_CONTROL;
	request_stream_msg.req_stream_id = 0;
	request_stream_msg.req_message_rate = 5;
	request_stream_msg.start_stop = 1;
	
	// Message pack and send
	
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];	
	mavlink_msg_request_data_stream_encode(255, 0, &msg, &request_stream_msg);
	unsigned len = mavlink_msg_to_send_buffer(buf, &msg);
	asio::write(port, asio::buffer(buf,len));
}

// Guidance message callback function

void guidance_msg_Callback(const Marvel::Guidance_Command::ConstPtr& msg) {
	current_arm_status = msg->arm;
	throttle = rc.calc_throttle(int(bound(msg->throttle)));
	roll = rc.calc_roll(int(bound(msg->roll)));
	pitch = rc.calc_pitch(int(bound(msg->pitch)));
	yaw = rc.calc_yaw(int(bound(msg->yaw)));
	mode = rc.calc_mode(int(bound(msg->mode)));
}

// Main program

int main(int argc, char **argv) {

    // Port configuration
    
    port.open("/dev/ttySAC0");
    port.set_option(asio::serial_port_base::baud_rate(115200));
   
    // Ros configuration
    
    ros::init(argc, argv, "server");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<Marvel::Autopilot>("server", 1000);
    ros::Subscriber sub = n.subscribe("guidance_pack", 1000, guidance_msg_Callback);
    ros::Timer rc_timer = n.createTimer(ros::Duration(0.05), msg_send_radio_callback);
	ros::Timer heartbeat_timer = n.createTimer(ros::Duration(1.0), msg_send_heartbeat_callback);
	
	// Initial configuration
	
	autopilot_msg.ready = false;
	
	// Read autopilot parameters
	
	msg_send_request_stream();
	msg_send_request_param();
	
    // Main loop
    
	char r_byte;
    while(1) {
        asio::read(port, asio::buffer(&r_byte,1));
        msg_receive(uint8_t(r_byte));
		
	 	if((current_arm_status == 1)&&(last_arm_status == 0)) {
			last_arm_status = 1;
			msg_send_arm();
		}
		if((current_arm_status == 0)&&(last_arm_status == 1)) {
			last_arm_status = 0;
			//msg_send_disarm();
		} 
		ros::spinOnce();
        pub.publish(autopilot_msg);
    }
    return 0;
}
