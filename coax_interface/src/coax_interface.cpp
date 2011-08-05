#include <ros/ros.h>
#include <string>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Quaternion.h>

#include <coax_interface/SetControlMode.h>
#include <coax_msgs/CoaxConfigureComm.h>
#include <coax_msgs/CoaxReachNavState.h>
#include <coax_msgs/CoaxRawControl.h>
#include <coax_msgs/CoaxSetTimeout.h>
#include <coax_msgs/CoaxState.h>

#include <com/sbapi.h>


class CoaxInterface
{
protected:
	
	ros::ServiceClient reach_nav_state;
	ros::ServiceClient configure_comm;
	ros::ServiceClient set_timeout;
	
	ros::Publisher raw_control_pub;
	ros::Publisher coax_info_pub;
	ros::Publisher control_mode_pub;
	
	ros::Subscriber coax_state_sub;
	ros::Subscriber matlab_trim_sub;
	ros::Subscriber matlab_nav_mode_sub;
	ros::Subscriber matlab_raw_control_sub;
	
	std::vector<ros::ServiceServer> set_control_mode;
	
	float roll_trim;
	float pitch_trim;
	float motor1;
	float motor2;
	float servo1;
	float servo2;
	int matlab_rawcontrol_age;
	int coax_state_age;
	int desired_nav_mode;
	
public:
	
	CoaxInterface(ros::NodeHandle & n)
	{
		reach_nav_state = n.serviceClient<coax_msgs::CoaxReachNavState>("reach_nav_state");
		configure_comm = n.serviceClient<coax_msgs::CoaxConfigureComm>("configure_comm");
		set_timeout = n.serviceClient<coax_msgs::CoaxSetTimeout>("set_timeout");
		
		raw_control_pub = n.advertise<coax_msgs::CoaxRawControl>("rawcontrol",1);
		coax_info_pub = n.advertise<geometry_msgs::Quaternion>("info",1);
		control_mode_pub = n.advertise<geometry_msgs::Quaternion>("control_mode",1);
		
		coax_state_sub = n.subscribe("state", 10, &CoaxInterface::coaxStateCallback, this);
		matlab_trim_sub = n.subscribe("trim", 10, &CoaxInterface::matlabTrimCallback, this);
		matlab_nav_mode_sub = n.subscribe("nav_mode", 10, &CoaxInterface::matlabNavModeCallback, this);
		matlab_raw_control_sub = n.subscribe("raw_control", 10, &CoaxInterface::matlabRawControlCallback, this);
		
		set_control_mode.push_back(n.advertiseService("set_control_mode", &CoaxInterface::setControlMode, this));
		
		roll_trim = 0;
		pitch_trim = 0;
		motor1 = 0;
		motor2 = 0;
		servo1 = 0;
		servo2 = 0;
		matlab_rawcontrol_age = 0;
		coax_state_age = 0;
		desired_nav_mode = 0;
	}
	~CoaxInterface(){
	}
	
	//===================
	// Service Clients
	//===================
	
	bool reachNavState(int des_state, float timeout)
	{
		coax_msgs::CoaxReachNavState srv;
		srv.request.desiredState = des_state;
		srv.request.timeout = timeout;
		if (reach_nav_state.call(srv)){
			ROS_INFO("Set nav_state to: %d, Result: %d", des_state, srv.response.result);
		}else{
			ROS_INFO("Failed to call service reach_nav_state");
		}
		if (srv.response.result == 0) {
			return 0; // successful
		} else {
			return 1; // not successful
		}
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
	
	bool setTimeout(unsigned int control_timeout_ms, unsigned int watchdog_timeout_ms)
	{
		coax_msgs::CoaxSetTimeout srv;
		srv.request.control_timeout_ms = control_timeout_ms;
		srv.request.watchdog_timeout_ms = watchdog_timeout_ms;
		if(set_timeout.call(srv)){
			ROS_INFO("Timeouts set: Control[%dms], Watchdog[%dms]", control_timeout_ms, watchdog_timeout_ms);
		}else{
			ROS_INFO("Failed to call service set_timeout");
		}
		return 0;
	}
	
	//===================
	// Callback Functions
	//===================
	
	void coaxStateCallback(const coax_msgs::CoaxState::ConstPtr & message)
	{
		geometry_msgs::Quaternion coax_info;
		coax_info.x = message->mode.navigation;
		coax_info.y = message->battery;
		coax_info.z = 0;
		coax_info.w = 0;
		
		//if ((message->mode.navigation == 0) && (desired_nav_mode == 1)) {
		//	ROS_INFO("Lost Zigbee connection for too long!!!");
		//	desired_nav_mode = 0;
		//}
		
		coax_state_age = 0;
		
		coax_info_pub.publish(coax_info);
	}
	
	void matlabTrimCallback(const geometry_msgs::Quaternion::ConstPtr & message)
	{
		roll_trim = message->x;
		pitch_trim = message->y;
	}
	
	void matlabNavModeCallback(const std_msgs::Bool::ConstPtr & message)
	{
		motor1 = 0;
		motor2 = 0;
		servo1 = 0;
		servo2 = 0;
		bool result;

		if (message->data) {
			result = reachNavState(SB_NAV_RAW, 0.5); // Navigation raw mode
			desired_nav_mode = 1;
			if (result) {
				// reachNavState not successful -> send error state 8 to matlab
				geometry_msgs::Quaternion control_mode;
				control_mode.x = 8;
				control_mode.y = 0;
				control_mode.z = 0;
				control_mode.w = 0;
				control_mode_pub.publish(control_mode);
			}
			
		} else {
			reachNavState(SB_NAV_STOP, 0.5); // Navigation stop mode
			desired_nav_mode = 0;
		}
	}
	
	void matlabRawControlCallback(const geometry_msgs::Quaternion::ConstPtr & message)
	{
		motor1 = message->x;
		motor2 = message->y;
		servo1 = message->z;
		servo2 = message->w;
		
		matlab_rawcontrol_age = 0;
	}
	
	//===================
	// Publisher
	//===================
	
	void rawControlPublisher(unsigned int rate)
	{
		ros::Rate loop_rate(rate);
		
		coax_msgs::CoaxRawControl raw_control;
		
		while(ros::ok())
		{
			if (matlab_rawcontrol_age < 20) {
				raw_control.motor1 = motor1;
				raw_control.motor2 = motor2;
				raw_control.servo1 = servo1 + roll_trim;
				raw_control.servo2 = servo2 + pitch_trim;
			} else { // if matlab does not send any commands for too long send zero inputs
				raw_control.motor1 = 0;
				raw_control.motor2 = 0;
				raw_control.servo1 = 0;
				raw_control.servo2 = 0;
			}
			
			if ((coax_state_age > 0.5*rate) && (coax_state_age <= 0.5*rate + 1)) {
				ROS_INFO("Lost Zigbee connection for too long (>0.5s)!!!");
				geometry_msgs::Quaternion control_mode;
				control_mode.x = 7;
				control_mode.y = 0;
				control_mode.z = 0;
				control_mode.w = 0;
				control_mode_pub.publish(control_mode);
			}
			
			raw_control_pub.publish(raw_control);
			matlab_rawcontrol_age += 1;
			coax_state_age += 1;
			
			ros::spinOnce();
			loop_rate.sleep();
		}
	}
	
	//===================
	// Services
	//===================
	
	bool setControlMode(coax_interface::SetControlMode::Request &req, coax_interface::SetControlMode::Response &out)
	{
		if (req.mode !=8) {
			geometry_msgs::Quaternion control_mode;
			control_mode.x = req.mode;
			control_mode.y = 0;
			control_mode.z = 0;
			control_mode.w = 0;
			control_mode_pub.publish(control_mode);
			
			if (req.mode == 9) {
				reachNavState(SB_NAV_STOP,0.5);
				desired_nav_mode = 0;
			}
		}else {
			ROS_INFO("Cannot manually call control mode 8");
		}

		
		/*
		char * key;
		key = req.mode;
		
		if (!strcmp(key, "s")) {
			control_mode.x = 1;
			control_mode_pub.publish(control_mode);
		} else if (!strcmp(key, "h")) {
			control_mode.x = 2;
			control_mode_pub.publish(control_mode);
		} else if (!strcmp(key, "g")) {
			control_mode.x = 3;
			control_mode_pub.publish(control_mode);
		} else if (!strcmp(key, "t")) {
			control_mode.x = 4;
			control_mode_pub.publish(control_mode);
		} else if (!strcmp(key, "l")) {
			control_mode.x = 5;
			control_mode_pub.publish(control_mode);
		} else if (!strcmp(key, "q")) {
			control_mode.x = 9;
			control_mode_pub.publish(control_mode);
		} else {
			printf("Usage: [s]=start, [h]=hover, [g]=go to position, [t]= trajectory following, [l]=land, [q]=quit \n");
		}
		*/
		
		out.result = 1;
		return true;
	}
	
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "coax_interface");
	
	ros::NodeHandle n("/coax_interface");
	
	CoaxInterface api(n);
	
	int simulation;
	n.param("simulation", simulation, 0);
	if (!simulation) {
		ros::Duration(1.5).sleep(); // make sure coax_server has enough time to boot up
		api.configureComm(10, SBS_MODES | SBS_BATTERY); // configuration of sending back data from CoaX
		api.setTimeout(500, 5000);
	} else {
		ROS_INFO("CoaX Interface in Simulation Mode");
	}

	int frequency;
	n.param("frequency", frequency, 100);

	api.rawControlPublisher(frequency);
	
	return(0);
}
