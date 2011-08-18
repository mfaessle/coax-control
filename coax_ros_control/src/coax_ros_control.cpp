#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <coax_msgs/CoaxState.h>
#include <coax_msgs/CoaxReachNavState.h>
#include <coax_ros_control/SetControlMode.h>
#include <coax_msgs/CoaxConfigureComm.h>
#include <coax_msgs/CoaxSetTimeout.h>
#include <coax_msgs/CoaxRawControl.h>

#include <armadillo>
#include <com/sbapi.h>
#include <coax_ros_control.h>


class CoaxRosControl
{
protected:
	
	ros::ServiceClient reach_nav_state;
	ros::ServiceClient configure_comm;
	ros::ServiceClient set_timeout;
	
	ros::Publisher raw_control_pub;
	
	ros::Subscriber coax_odom_sub;
	ros::Subscriber coax_state_sub;
	
	std::vector<ros::ServiceServer> set_control_mode;
	
	double battery_voltage;
	int coax_state_age;
	int coax_nav_mode;
	int raw_control_age;
	bool LOW_POWER_DETECTED;
	int CONTROL_MODE;
	bool FIRST_START;
	bool FIRST_HOVER;
	bool FIRST_TRAJECTORY;
	bool FIRST_LANDING;
	double roll_trim;
	double pitch_trim;
	double motor_up;
	double motor_lo;
	double servo_roll;
	double servo_pitch;
	int COAX;
	double hover_position[3];
	double hover_orientation;

public:
	
	CoaxRosControl(ros::NodeHandle & n)
	{
		reach_nav_state = n.serviceClient<coax_msgs::CoaxReachNavState>("reach_nav_state");
		configure_comm = n.serviceClient<coax_msgs::CoaxConfigureComm>("configure_comm");
		set_timeout = n.serviceClient<coax_msgs::CoaxSetTimeout>("set_timeout");
		
		raw_control_pub = n.advertise<coax_msgs::CoaxRawControl>("rawcontrol",1);
		
		coax_odom_sub = n.subscribe("odom", 10, &CoaxRosControl::coaxOdomCallback, this);
		coax_state_sub = n.subscribe("state", 10, &CoaxRosControl::coaxStateCallback, this);
		
		set_control_mode.push_back(n.advertiseService("set_control_mode", &CoaxRosControl::setControlMode, this));
		
		COAX = 56;
		
		battery_voltage = 12;
		coax_state_age = 0;
		coax_nav_mode = 0;
		raw_control_age = 0;
		LOW_POWER_DETECTED = false;
		CONTROL_MODE = CONTROL_LANDED;
		FIRST_START = false;
		FIRST_HOVER = false;
		FIRST_TRAJECTORY = false;
		FIRST_LANDING = false;
		
		roll_trim = 0;
		pitch_trim = 0;
		motor_up = 0;
		motor_lo = 0;
		servo_roll = 0;
		servo_pitch = 0;
	}
	~CoaxRosControl(){
	}
	
	//===================
	// Service Clients
	//===================
	
	bool reachNavState(int des_state, float timeout)
	{
		coax_msgs::CoaxReachNavState srv;
		srv.request.desiredState = des_state;
		srv.request.timeout = timeout;
		reach_nav_state.call(srv);
		
		return 0;
	}
	
	bool configureComm(int frequency, int contents)
	{
		coax_msgs::CoaxConfigureComm srv;
		srv.request.frequency = frequency;
		srv.request.contents = contents;
		configure_comm.call(srv);
		
		return 0;
	}
	
	bool setTimeout(unsigned int control_timeout_ms, unsigned int watchdog_timeout_ms)
	{
		coax_msgs::CoaxSetTimeout srv;
		srv.request.control_timeout_ms = control_timeout_ms;
		srv.request.watchdog_timeout_ms = watchdog_timeout_ms;
		set_timeout.call(srv);

		return 0;
	}
	
	//===================
	// Callback Functions
	//===================
	
	void coaxStateCallback(const coax_msgs::CoaxState::ConstPtr & message)
	{
		battery_voltage = 0.8817*message->battery + 1.5299;
		coax_nav_mode = message->mode.navigation;
		
		if ((battery_voltage < 10.80) && !LOW_POWER_DETECTED){
			ROS_INFO("Battery Low!!! (%fV) Landing initialized",battery_voltage);
			LOW_POWER_DETECTED = true;
		}
		
		coax_state_age = 0;
	}
	
	void coaxOdomCallback(const nav_msgs::Odometry::ConstPtr & message)
	{
		// getting odom information
		// calculation of coax_state and Rb2w
		// vicon jump detection ?
		// estimate non observable states		
		
		arma::colvec coax_state = arma::zeros(17);
		arma::mat Rb2w = arma::zeros(3,3);
		arma::colvec trajectory = arma::zeros(11);
		arma::colvec control = arma::zeros(4);
		
		switch (CONTROL_MODE) {
			case CONTROL_START:
				motor_up = 0;
				motor_lo = 0;
				servo_roll = 0;
				servo_pitch = 0;
				break;
			
			case CONTROL_HOVER:
				if (FIRST_HOVER) {
					hover_position[0] = coax_state(0);
					hover_position[1] = coax_state(1);
					hover_position[2] = coax_state(2);
					hover_orientation = atan2(Rb2w(1,0),Rb2w(0,0));
					FIRST_HOVER = 0;
				}
				if (LOW_POWER_DETECTED) {
					CONTROL_MODE = CONTROL_LANDING;
					FIRST_LANDING = 1;
				}
				
				trajectory(0) = hover_position[0];
				trajectory(1) = hover_position[1];
				trajectory(2) = hover_position[2];
				trajectory(9) = hover_orientation;
				
				// compute control commands
				//control_function(&control, coax_state, Rb2w, trajectory, e_i, dt, param, contr_param);
				
				// set control commands
				motor_up = control(0);
				motor_lo = control(1);
				servo_roll = control(2);
				servo_pitch = control(3);
				
				break;
			
			case CONTROL_LANDED:
				motor_up = 0;
				motor_lo = 0;
				servo_roll = 0;
				servo_pitch = 0;
				break;

			default:
				break;
		}
		
		raw_control_age = 0;
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
			if (raw_control_age < 20) {
				raw_control.motor1 = motor_up;
				raw_control.motor2 = motor_lo;
				raw_control.servo1 = servo_roll + roll_trim;
				raw_control.servo2 = servo_pitch + pitch_trim;
			} else { // if we do not get new control_values for too long -> send zero commands
				raw_control.motor1 = 0;
				raw_control.motor2 = 0;
				raw_control.servo1 = 0;
				raw_control.servo2 = 0;
			}
			
			if ((coax_state_age > 0.5*rate) && (coax_state_age <= 0.5*rate + 1)) {
				ROS_INFO("Lost Zigbee connection for too long (>0.5s)!!!");
			}
			
			raw_control_pub.publish(raw_control);
			raw_control_age += 1;
			coax_state_age += 1;
			
			ros::spinOnce();
			loop_rate.sleep();
		}
	}
	
	//===================
	// Services
	//===================
	
	bool setControlMode(coax_ros_control::SetControlMode::Request &req, coax_ros_control::SetControlMode::Response &out)
	{
		out.result = 1;
		
		switch (req.mode)
		{
			case 1:
				if (battery_voltage > 11) {
                    if (coax_nav_mode != SB_NAV_RAW) {
						if (coax_nav_mode != SB_NAV_STOP) {
							reachNavState(SB_NAV_STOP, 0.5);
							ros::Duration(0.5).sleep(); // make sure CoaX is in SB_NAV_STOP mode
						}
                        reachNavState(SB_NAV_RAW, 0.5);
					}
					// set initial trim
					if (COAX == 56) {
						roll_trim = 0.0285;
						pitch_trim = 0.0921;
					} else {
						roll_trim = 0;
						pitch_trim = 0;
					}
					// switch to start procedure
					CONTROL_MODE = CONTROL_START;
					FIRST_START = true;
                } else {
					ROS_INFO("Battery Low!!! (%f V) Start denied",battery_voltage);
					LOW_POWER_DETECTED = true;
					out.result = 0;
				}
				break;
			
			case 3:
				if ((CONTROL_MODE == CONTROL_GOTOPOS) || (CONTROL_MODE == CONTROL_TRAJECTORY)){
					CONTROL_MODE = CONTROL_HOVER;
				FIRST_HOVER = true;
				} else {
					out.result = 0;
				}
				break;
			
			case 5:
				if ((CONTROL_MODE == CONTROL_HOVER) || (CONTROL_MODE == CONTROL_GOTOPOS)){
					CONTROL_MODE = CONTROL_TRAJECTORY;
					FIRST_TRAJECTORY = true;
				} else {
					out.result = 0;
				}
				break;
			
			case 6:
				if (CONTROL_MODE == CONTROL_HOVER){
					CONTROL_MODE = CONTROL_LANDING;
					FIRST_LANDING = true;
				} else {
					out.result = 0;
				}
				break;
			
			case 9:
				motor_up = 0;
				motor_lo = 0;
				servo_roll = 0;
				servo_pitch = 0;
				
				reachNavState(SB_NAV_STOP, 0.5);
				break;
			
			default:
				ROS_INFO("Non existent or non valid state request!");
				out.result = 0;
		}
		
		return true;
	}
	
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "coax_ros_control");
	
	ros::NodeHandle n("/coax_ros_control");
	
	CoaxRosControl api(n);
	
	ros::Duration(1.5).sleep(); // make sure coax_server has enough time to boot up
	api.configureComm(10, SBS_MODES | SBS_BATTERY); // configuration of sending back data from CoaX
	api.setTimeout(500, 5000);
	
	int frequency;
	n.param("frequency", frequency, 100);
	
	api.rawControlPublisher(frequency);

	return(0);
}