#include <ros/ros.h>
#include <signal.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>

#include <coax_msgs/CoaxConfigureComm.h>
#include <coax_msgs/CoaxControl.h>
#include <coax_msgs/CoaxReachNavState.h>
#include <coax_msgs/CoaxSetTrimMode.h>

#include <com/sbapi.h>

#define KEYCODE_RGT 0x36 
#define KEYCODE_LFT 0x34
#define KEYCODE_UP 0x38
#define KEYCODE_DWN 0x35
#define KEYCODE_Q 0x71
#define KEYCODE_W 0x77
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64
#define KEYCODE_A 0x61
#define KEYCODE_I 0x69
#define KEYCODE_K 0x6B
#define KEYCODE_H 0x68
#define KEYCODE_F 0x66
#define KEYCODE_V 0x76
#define KEYCODE_C 0x63
#define KEYCODE_B 0x62


class CoaxKeyboardOperation
{
protected:
	ros::ServiceClient reach_nav_state;
	ros::ServiceClient set_trim_mode;
	ros::ServiceClient configure_comm;
	
	ros::Publisher control_pub;	
	
	int kfd;
	struct termios cooked, raw;
	
public:
	CoaxKeyboardOperation(ros::NodeHandle & n)
	{
		kfd = 0;
		reach_nav_state = n.serviceClient<coax_msgs::CoaxReachNavState>("coax_server/reach_nav_state");
		set_trim_mode = n.serviceClient<coax_msgs::CoaxSetTrimMode>("coax_server/set_trim_mode");
		configure_comm = n.serviceClient<coax_msgs::CoaxConfigureComm>("coax_server/configure_comm");
		
		control_pub = n.advertise<coax_msgs::CoaxControl>("coax_server/control",1);
	}
	~CoaxKeyboardOperation(){
	}
	
	bool configureComm(int frequency, int contents)
	{
		coax_msgs::CoaxConfigureComm srv;
		srv.request.frequency = frequency;
		srv.request.contents = contents;
		if(configure_comm.call(srv)){
			ROS_INFO("Communication configured: Freq[%dHz], Cont[%d]", frequency, contents);
		}else{
			ROS_INFO("Failed to call service configure_comm");
		}
		
		return 0;
	}
	
	bool reachNavState(int des_state, float timeout)
	{
		coax_msgs::CoaxReachNavState srv;
		srv.request.desiredState = des_state;
		srv.request.timeout = timeout;
		if(reach_nav_state.call(srv)){
			ROS_INFO("Set nav_state to: %d, Result: %d", des_state, srv.response.result);
		}else{
			ROS_INFO("Failed to call service reach_nav_state");
		}
		
		return 0;
	}
	
	bool setTrimMode(int mode, float rollTrim, float pitchTrim)
	{
		coax_msgs::CoaxSetTrimMode srv;
		srv.request.mode.trimMode = mode;
		srv.request.mode.rollTrim = rollTrim;
		srv.request.mode.pitchTrim = pitchTrim;
		if(set_trim_mode.call(srv)){
			ROS_INFO("Trim Mode: [%d], Roll Trim: [%f], Pitch Trim: [%f] ", mode, rollTrim, pitchTrim);
		}else{
			ROS_INFO("Failed to call service set_trim_mode");
		}
		
		return 0;
	}
	
	void keyLoop(unsigned int rate)
	{
		ros::Rate loop_rate(rate);
		char c;
		int tem;
		float roll, pitch, yaw, altitude;
		float rollTrim, pitchTrim;
		roll = 0.1;
		pitch = 0;
		yaw = 0;
		altitude = 0;
		pitchTrim = -0.35;
		rollTrim = -0.24;
		setTrimMode(1,rollTrim,pitchTrim);
		
		// get the console in raw mode                                                              
		tcgetattr(kfd, &cooked);
		memcpy(&raw, &cooked, sizeof(struct termios));
		raw.c_lflag &=~ (ICANON | ECHO);
		// Setting a new line, then end of file                         
		raw.c_cc[VEOL] = 1;
		raw.c_cc[VEOF] = 2;
		tcsetattr(kfd, TCSANOW, &raw);	
		
		tem = fcntl(0, F_GETFL, 0);
		fcntl(0, F_SETFL, (tem | O_NDELAY));

		while(ros::ok())
		{
			// get the next event from the keyboard  
			if(read(kfd, &c, 1) < 0)
			{
				roll = 0;
				pitch = 0;
				yaw = 0;
			}else{
			
			//ROS_INFO("value: 0x%02X\n", c);
			//ROS_INFO("get to switch");
			
			switch(c)
			{
				case KEYCODE_LFT:
					ROS_DEBUG("LEFT");
					roll = -0.2;
					break;
				case KEYCODE_RGT:
					ROS_DEBUG("RIGHT");
					roll = 0.2;
					break;
				case KEYCODE_UP:
					ROS_DEBUG("UP");
					pitch = -0.2;
					break;
				case KEYCODE_DWN:
					ROS_DEBUG("DOWN");
					pitch = 0.2;
					break;
				case KEYCODE_W:
					ROS_DEBUG("W");
					altitude += 0.05;
					if (altitude > 1)
						altitude = 1;
					break;
				case KEYCODE_S:
					ROS_DEBUG("S");
					altitude -= 0.05;
					if (altitude < 0)
						altitude = 0;
					break;
				case KEYCODE_D:
					ROS_DEBUG("D");
					yaw = 0.3;
					break;
				case KEYCODE_A:
					ROS_DEBUG("A");
					yaw = -0.3;
					break;
				case KEYCODE_I:
					ROS_DEBUG("I");
					reachNavState(1,0.5);
					break;
				case KEYCODE_K:
					ROS_DEBUG("K");
					reachNavState(0,0.5);
					break;
				case KEYCODE_H:
					ROS_DEBUG("H");
					reachNavState(5,0.5);
					break;
				case KEYCODE_Q:
					ROS_DEBUG("Q");
					quit(0);
					break;
				case KEYCODE_F:
					ROS_DEBUG("F");
					pitchTrim -= 0.01;
					setTrimMode(1,rollTrim,pitchTrim);
					break;
				case KEYCODE_V:
					ROS_DEBUG("V");
					pitchTrim += 0.01;
					setTrimMode(1,rollTrim,pitchTrim);
					break;
				case KEYCODE_C:
					ROS_DEBUG("C");
					rollTrim -= 0.01;
					setTrimMode(1,rollTrim,pitchTrim);
					break;
				case KEYCODE_B:
					ROS_DEBUG("B");
					rollTrim += 0.01;
					setTrimMode(1,rollTrim,pitchTrim);
					break;
				default:
					roll = 0;
					pitch = 0;
					yaw = 0;
					break;
			}
			
			}
			
			coax_msgs::CoaxControl control;
			control.roll = roll;
			control.pitch = pitch;
			control.yaw = yaw;
			control.altitude = altitude;
			control_pub.publish(control);
			
			ros::spinOnce();
			loop_rate.sleep();

		}
		fcntl(0, F_SETFL, tem);
		return;
	}
	
	void quit(int sig)
	{
		tcsetattr(kfd, TCSANOW, &cooked);
		reachNavState(0,0.5);
		ros::shutdown();
		exit(0);
	}
	
};





int main(int argc, char** argv)
{
	ros::init(argc, argv, "coax_keyboard_op");
	
	ros::NodeHandle n;
	
	CoaxKeyboardOperation api(n);
	
	api.configureComm(10, SBS_MODES | SBS_BATTERY); // configuration of sending back data
	
	api.keyLoop(10);
	
	return(0);
}


