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
#include <CoaxRosControl.h>


CoaxRosControl::CoaxRosControl(ros::NodeHandle & n)
{
	reach_nav_state = n.serviceClient<coax_msgs::CoaxReachNavState>("reach_nav_state");
	configure_comm = n.serviceClient<coax_msgs::CoaxConfigureComm>("configure_comm");
	set_timeout = n.serviceClient<coax_msgs::CoaxSetTimeout>("set_timeout");
		
	raw_control_pub = n.advertise<coax_msgs::CoaxRawControl>("rawcontrol",1);
		
	coax_odom_sub = n.subscribe("odom", 10, &CoaxRosControl::coaxOdomCallback, this);
	coax_state_sub = n.subscribe("state", 10, &CoaxRosControl::coaxStateCallback, this);
		
	set_control_mode.push_back(n.advertiseService("set_control_mode", &CoaxRosControl::setControlMode, this));
		
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
	FIRST_RUN = true;
		
	roll_trim = 0;
	pitch_trim = 0;
	motor_up = 0;
	motor_lo = 0;
	servo_roll = 0;
	servo_pitch = 0;
	
	Omega_up = 0;
	Omega_lo = 0;
	prev_Omega_up = 0;
	prev_Omega_lo = 0;
}
CoaxRosControl::~CoaxRosControl()
{

}
	
//===================
// Service Clients
//===================
	
bool CoaxRosControl::reachNavState(int des_state, float timeout)
{
	coax_msgs::CoaxReachNavState srv;
	srv.request.desiredState = des_state;
	srv.request.timeout = timeout;
	reach_nav_state.call(srv);
	
	return 0;
}

bool CoaxRosControl::configureComm(int frequency, int contents)
{
	coax_msgs::CoaxConfigureComm srv;
	srv.request.frequency = frequency;
	srv.request.contents = contents;
	configure_comm.call(srv);
	
	return 0;
}

bool CoaxRosControl::setTimeout(unsigned int control_timeout_ms, unsigned int watchdog_timeout_ms)
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

void CoaxRosControl::coaxStateCallback(const coax_msgs::CoaxState::ConstPtr & message)
{
	battery_voltage = 0.8817*message->battery + 1.5299;
	coax_nav_mode = message->mode.navigation;
	
	if ((battery_voltage < 10.80) && !LOW_POWER_DETECTED){
		ROS_INFO("Battery Low!!! (%fV) Landing initialized",battery_voltage);
		LOW_POWER_DETECTED = true;
	}
	
	coax_state_age = 0;
}

void CoaxRosControl::coaxOdomCallback(const nav_msgs::Odometry::ConstPtr & message)
{

	// vicon jump detection ?
	
	// Read odometry values
	arma::colvec coax_state = arma::zeros(17);
	coax_state(0) = message->pose.pose.position.x;
	coax_state(1) = message->pose.pose.position.y;
	coax_state(2) = message->pose.pose.position.z;
	coax_state(3) = message->twist.twist.linear.x;
	coax_state(4) = message->twist.twist.linear.y;
	coax_state(5) = message->twist.twist.linear.z;
	double qx = message->pose.pose.orientation.x;
	double qy = message->pose.pose.orientation.y;
	double qz = message->pose.pose.orientation.z;
	double qw = message->pose.pose.orientation.w;
	coax_state(6) = atan2(2*(qw*qx+qy*qz),1-2*(qx*qx+qy*qy));
	coax_state(7) = asin(2*(qw*qy-qz*qx));
	coax_state(8) = atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz));
	coax_state(9) = message->twist.twist.angular.x;
	coax_state(10) = message->twist.twist.angular.y;
	coax_state(11) = message->twist.twist.angular.z;
	coax_state(12) = Omega_up;
	coax_state(13) = Omega_lo;
	
	// getting current time
	time_now = ros::Time::now().toSec();
	if (FIRST_RUN){
		time_prev = time_now;
		FIRST_RUN = 0;
	}
	//double delta_t = time_now - time_prev;
	
	// Compute rotation matrix
	arma::mat Rb2w = arma::zeros(3,3);
	Rb2w(0,0) = 1-2*qy*qy-2*qz*qz;
	Rb2w(0,1) = 2*qx*qy-2*qz*qw;
	Rb2w(0,2) = 2*qx*qz+2*qy*qw;
	Rb2w(1,0) = 2*qx*qy+2*qz*qw;
	Rb2w(1,1) = 1-2*qx*qx-2*qz*qz;
	Rb2w(1,2) = 2*qy*qz-2*qx*qw;
	Rb2w(2,0) = 2*qx*qz-2*qy*qw;
	Rb2w(2,1) = 2*qy*qz+2*qx*qw;
	Rb2w(2,2) = 1-2*qx*qx-2*qy*qy;
	
	// Estimate stabilizer bar orientation
	
	
	
	arma::colvec trajectory = arma::zeros(11);
	double control[4] = {0,0,0,0};	
	
	switch (CONTROL_MODE) {
		case CONTROL_START:
			setControls(control);
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
			controlFunction(control, coax_state, Rb2w, trajectory, model_params, control_params);
			
			// set control commands
			setControls(control);
			break;
		
		case CONTROL_LANDED:
			setControls(control);
			break;

		default:
			break;
	}
	
	raw_control_age = 0;
	time_prev = time_now;
}

void CoaxRosControl::setControls(double* control)
{
	if (control[0] > 1) {
		motor_up = 1;
	} else if (control[0] < 0) {
		motor_up = 0;
	} else {
		motor_up = control[0];
	}
	
	if (control[1] > 1) {
		motor_lo = 1;
	} else if (control[1] < 0) {
		motor_lo = 0;
	} else {
		motor_lo = control[1];
	}
	
	if (control[2] > 1) {
		servo_roll = 1;
	} else if (control[2] < -1) {
		servo_roll = -1;
	} else {
		servo_roll = control[2];
	}
	
	if (control[3] > 1) {
		servo_pitch = 1;
	} else if (control[3] < -1) {
		servo_pitch = -1;
	} else {
		servo_pitch = control[3];
	}
}

void CoaxRosControl::controlFunction(double* control, arma::colvec coax_state, arma::mat Rb2w, 
									 arma::colvec trajectory, model_params_t model_params, control_params_t control_params)
{
	
	// rotor speeds
	double Omega_up = coax_state(12);
	double Omega_lo = coax_state(13);
	
	// stabilizer bar direction
	double z_barx = coax_state(14);
	double z_bary = coax_state(15);
	double z_barz = coax_state(16);
	
	// Parameters
	double g = 9.81;
	double m = model_params.mass;
	//double Ixx = model_params.Ixx;
	//double Iyy = model_params.Iyy;
	//double Izz = model_params.Izz;
	//double d_up = model_params.d_up;
	//double d_lo = model_params.d_lo;
	//double k_springup = model_params.k_springup;
	//double k_springlo = model_params.k_springlo;
	double l_up = model_params.l_up;
	double l_lo = model_params.l_lo;
	double k_Tup = model_params.k_Tup;
	double k_Tlo = model_params.k_Tlo;
	double k_Mup = model_params.k_Mup;
	double k_Mlo = model_params.k_Mlo;
	//double Tf_motup = model_params.Tf_motup;
	//double Tf_motlo = model_params.Tf_motlo;
	//double Tf_up = model_params.Tf_up;
	double rs_mup = model_params.rs_mup;
	double rs_bup = model_params.rs_bup;
	double rs_mlo = model_params.rs_mlo;
	double rs_blo = model_params.rs_blo;
	double zeta_mup = model_params.zeta_mup;
	double zeta_bup = model_params.zeta_bup;
	double zeta_mlo = model_params.zeta_mlo;
	double zeta_blo = model_params.zeta_blo;
	double max_SPangle = model_params.max_SPangle;
	
	// Control Parameters
	double K_lqr[4][16] = {};
	
	// Upper thrust vector direction
	double z_Tupz = cos(l_up*acos(z_barz));
	arma::colvec z_Tup_p(3);
	double temp;
	if (z_Tupz < 1){
		temp = sqrt((1-z_Tupz*z_Tupz)/(z_barx*z_barx + z_bary*z_bary));
		z_Tup_p(0) = z_barx*temp;
		z_Tup_p(1) = z_bary*temp;
		z_Tup_p(2) = z_Tupz;
	}else {
		z_Tup_p(0) = 0;
		z_Tup_p(1) = 0;
		z_Tup_p(2) = 1;
	}
	double zeta = zeta_mup*Omega_up + zeta_bup;
	arma::colvec z_Tup(3);
	z_Tup(0) = cos(zeta)*z_Tup_p(0) - sin(zeta)*z_Tup_p(1);
	z_Tup(1) = sin(zeta)*z_Tup_p(0) + cos(zeta)*z_Tup_p(1);
	z_Tup(2) = z_Tup_p(2);
	
	double Omega_lo0 = sqrt(m*g/(k_Tup*k_Mlo/k_Mup + k_Tlo));
	double Omega_up0 = sqrt(k_Mlo/k_Mup*Omega_lo0*Omega_lo0);
	
	double a_up = -asin(z_Tup(2));
	double b_up = asin(z_Tup(1)/cos(a_up));
	
	
	arma::colvec pos_error(3);
	arma::colvec vel_error(3);
	
	pos_error = coax_state.subvec(0, 2) - trajectory.subvec(0, 2);
	vel_error = coax_state.subvec(3, 5) - trajectory.subvec(3, 5);
	pos_error = strans(Rb2w)*pos_error;
	vel_error = strans(Rb2w)*vel_error;
	
	double error[16];
	error[0] = pos_error(0);
	error[1] = pos_error(1);
	error[2] = pos_error(2);
	error[3] = vel_error(0);
	error[4] = vel_error(1);
	error[5] = vel_error(2);
	error[6] = coax_state(6);
	error[7] = coax_state(7);
	error[8] = coax_state(8) - trajectory(9);
	error[9] = coax_state(9);
	error[10] = coax_state(10);
	error[11] = coax_state(11) - trajectory(10);
	error[12] = coax_state(12) - Omega_up0;
	error[13] = coax_state(13) - Omega_lo0;
	error[14] = a_up;
	error[15] = b_up;
	
	if (error[8] > M_PI){
		error[8] = error[8] - 2*M_PI;
	}else if (error[8] < -M_PI){
		error[8] = error[8] + 2*M_PI;
	}
	
	double inputs[4] = {0,0,0,0};
	for (int i=0; i<4; i++) {
		for (int j=0; j<16; j++) {
			inputs[i] -= K_lqr[i][j]*error[j];
		}
	}
	
	// feed forward on rotor speeds
	control[0] = inputs[0] + (Omega_up0 - rs_bup)/rs_mup;
	control[1] = inputs[1] + (Omega_lo0 - rs_blo)/rs_mlo;
	
	// correct for phase lag of servo inputs
	double a_SP = inputs[3]*max_SPangle;
	double b_SP = inputs[4]*max_SPangle;
	arma::colvec z_SP(3);
	z_SP(0) = sin(b_SP);
	z_SP(1) = -sin(a_SP)*cos(b_SP);
	z_SP(2) = cos(a_SP)*cos(b_SP);
	double z_Tloz = cos(l_lo*acos(z_SP(2)));
	arma::colvec z_Tlo(3);
	if (z_Tloz < 1){
		temp = sqrt((1-z_Tloz*z_Tloz)/(z_SP(0)*z_SP(0) + z_SP(1)*z_SP(1)));
		z_Tlo(0) = z_SP(0)*temp;
		z_Tlo(1) = z_SP(1)*temp;
		z_Tlo(2) = z_Tupz;
	}else{
		z_Tlo(0) = 0;
		z_Tlo(1) = 0;
		z_Tlo(2) = 1;
	}
	zeta = zeta_mlo*Omega_lo + zeta_blo;
	arma::colvec z_Tlo_p(3);
	z_Tlo_p(0) = cos(zeta)*z_Tlo(0) - sin(zeta)*z_Tlo(1);
	z_Tlo_p(1) = sin(zeta)*z_Tlo(0) + cos(zeta)*z_Tlo(1);
	z_Tlo_p(2) = z_Tlo(2);
		
	double z_SPz = cos(1/l_lo*acos(z_Tlo_p(2)));
	if (z_SPz < 1){
		temp = sqrt((1-z_SPz*z_SPz)/(z_Tlo_p(0)*z_Tlo_p(0) + z_Tlo_p(1)*z_Tlo_p(1)));
		z_SP(0) = z_Tlo_p(0)*temp;
		z_SP(1) = z_Tlo_p(1)*temp;
		z_SP(2) = z_SPz;
	}else{
		z_SP(0) = 0;
		z_SP(1) = 0;
		z_SP(2) = 1;
	}
	double b_lo_des = asin(z_SP(0));
	double a_lo_des = asin(-z_SP(1)/cos(b_lo_des));
	
	control[2] = a_lo_des/max_SPangle;
	control[3] = b_lo_des/max_SPangle;
	
}

//===================
// Publisher
//===================

void CoaxRosControl::rawControlPublisher(unsigned int rate)
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
			ROS_INFO("Lost Zigbee connection for too long!!!");
		}
		
		raw_control_pub.publish(raw_control);
		raw_control_age += 1;
		coax_state_age += 1;
		
		// Estimate rotor speeds (do that here to have the same frequency as we apply inputs)
		double prev_Omega_up_des = model_params.rs_mup*motor_up + model_params.rs_bup;
		double prev_Omega_lo_des = model_params.rs_mlo*motor_lo + model_params.rs_blo;
		
		Omega_up = prev_Omega_up + 1/model_params.Tf_motup*(prev_Omega_up_des - prev_Omega_up)/rate;
		Omega_lo = prev_Omega_lo + 1/model_params.Tf_motlo*(prev_Omega_lo_des - prev_Omega_lo)/rate;
		prev_Omega_up = Omega_up;
		prev_Omega_lo = Omega_lo;
		
		ros::spinOnce();
		loop_rate.sleep();
	}
}

//===================
// Services
//===================

bool CoaxRosControl::setControlMode(coax_ros_control::SetControlMode::Request &req, coax_ros_control::SetControlMode::Response &out)
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
			roll_trim = 0;
			pitch_trim = 0;
			
			reachNavState(SB_NAV_STOP, 0.5);
			
			CONTROL_MODE = CONTROL_LANDED;
			break;
		
		default:
			ROS_INFO("Non existent or non valid state request!");
			out.result = 0;
	}
	
	return true;
}

//===================
// Parameter Setting
//===================

void CoaxRosControl::SetPlatform(int CoaX)
{
	COAX = CoaX;
}

void CoaxRosControl::SetMass(double mass)
{
	model_params.mass = mass;
}

void CoaxRosControl::SetInertia(double Ixx, double Iyy, double Izz)
{
	model_params.Ixx = Ixx;
	model_params.Iyy = Iyy;
	model_params.Izz = Izz;
}

void CoaxRosControl::SetRotorOffset(double d_up, double d_lo)
{
	model_params.d_up = d_up;
	model_params.d_lo = d_lo;
}

void CoaxRosControl::SetUpperRotorFollowingTime(double Tf_up)
{
	model_params.Tf_up = Tf_up;
}

void CoaxRosControl::SetRotorLinkageFactor(double l_up, double l_lo)
{
	model_params.l_up = l_up;
	model_params.l_lo = l_lo;
}

void CoaxRosControl::SetRotorSpringConstant(double k_springup, double k_springlo)
{
	model_params.k_springup = k_springup;
	model_params.k_springlo = k_springlo;
}

void CoaxRosControl::SetRotorThrustFactor(double k_Tup, double k_Tlo)
{
	model_params.k_Tup = k_Tup;
	model_params.k_Tlo = k_Tlo;
}

void CoaxRosControl::SetRotorMomentFactor(double k_Mup, double k_Mlo)
{
	model_params.k_Mup = k_Mup;
	model_params.k_Mlo = k_Mlo;
}

void CoaxRosControl::SetMotorFollowingTime(double Tf_motup, double Tf_motlo)
{
	model_params.Tf_motup = Tf_motup;
	model_params.Tf_motlo = Tf_motlo;
}

void CoaxRosControl::SetUpperRotorSpeedConversion(double rs_mup, double rs_bup)
{
	model_params.rs_mup = rs_mup;
	model_params.rs_bup = rs_bup;
}

void CoaxRosControl::SetLowerRotorSpeedConversion(double rs_mlo, double rs_blo)
{
	model_params.rs_mlo = rs_mlo;
	model_params.rs_blo = rs_blo;
}

void CoaxRosControl::SetUpperPhaseLag(double zeta_mup, double zeta_bup)
{
	model_params.zeta_mup = zeta_mup;
	model_params.zeta_bup = zeta_bup;
}

void CoaxRosControl::SetLowerPhaseLag(double zeta_mlo, double zeta_blo)
{
	model_params.zeta_mlo = zeta_mlo;
	model_params.zeta_blo = zeta_blo;
}

void CoaxRosControl::SetMaximumSwashPlateAngle(double max_SPangle)
{
	model_params.max_SPangle = max_SPangle;
}

void CoaxRosControl::load_model_params(ros::NodeHandle &n)
{
	
	double m;
	n.getParam("mass", m);
	SetMass(m);
	
	double Ixx, Iyy, Izz;
	n.getParam("inertia/Ixx", Ixx);
	n.getParam("inertia/Iyy", Iyy);
	n.getParam("inertia/Izz", Izz);
	SetInertia(Ixx, Iyy, Izz);
	
	double d_up, d_lo;
	n.getParam("offset/upper", d_up);
	n.getParam("offset/lower", d_lo);
	SetRotorOffset(d_up, d_lo);
	
	double l_up, l_lo;
	n.getParam("linkage_factor/upper", l_up);
	n.getParam("linkage_factor/lower", l_lo);
	SetRotorLinkageFactor(l_up, l_lo);
	
	double k_springup, k_springlo;
	n.getParam("spring_constant/upper", k_springup);
	n.getParam("spring_constant/lower", k_springlo);
	SetRotorSpringConstant(k_springup, k_springlo);
	
	double k_Tup, k_Tlo;
	n.getParam("thrust_factor/upper", k_Tup);
	n.getParam("thrust_factor/lower", k_Tlo);
	SetRotorThrustFactor(k_Tup, k_Tlo);
	
	double k_Mup, k_Mlo;
	n.getParam("moment_factor/upper", k_Mup);
	n.getParam("moment_factor/lower", k_Mlo);
	SetRotorMomentFactor(k_Mup, k_Mlo);
	
	double Tf_up;
	n.getParam("following_time/bar", Tf_up);
	SetUpperRotorFollowingTime(Tf_up);
	
	double Tf_motup, Tf_motlo;
	n.getParam("following_time/motors/upper", Tf_motup);
	n.getParam("following_time/motors/lower", Tf_motlo);
	SetMotorFollowingTime(Tf_motup, Tf_motlo);
	
	double rs_mup, rs_bup;
	n.getParam("speed_conversion/slope/upper", rs_mup);
	n.getParam("speed_conversion/offset/upper", rs_bup);
	SetUpperRotorSpeedConversion(rs_mup, rs_bup);
	
	double rs_mlo, rs_blo;
	n.getParam("speed_conversion/slope/lower", rs_mlo);
	n.getParam("speed_conversion/offset/lower", rs_blo);
	SetLowerRotorSpeedConversion(rs_mlo, rs_blo);
	
	double zeta_mup, zeta_bup;
	n.getParam("phase_lag/slope/upper", zeta_mup);
	n.getParam("phase_lag/offset/upper", zeta_bup);
	SetUpperPhaseLag(zeta_mup, zeta_bup);
	
	double zeta_mlo, zeta_blo;
	n.getParam("phase_lag/slope/lower", zeta_mlo);
	n.getParam("phase_lag/offset/lower", zeta_blo);
	SetLowerPhaseLag(zeta_mlo, zeta_blo);
	
	double max_SPangle;
	n.getParam("max_swashplate_angle", max_SPangle);
	SetMaximumSwashPlateAngle(max_SPangle);
	
	return;
}




int main(int argc, char** argv)
{
	ros::init(argc, argv, "coax_ros_control");
	
	ros::NodeHandle n("/coax_ros_control");
	
	CoaxRosControl api(n);
	
	ros::Duration(1.5).sleep(); // make sure coax_server has enough time to boot up
	api.configureComm(10, SBS_MODES | SBS_BATTERY); // configuration of sending back data from CoaX
	api.setTimeout(500, 5000);
	
	int CoaX;
	n.param("CoaX", CoaX, 56);
	api.SetPlatform(CoaX);
	
	api.load_model_params(n);
	
	int frequency;
	n.param("frequency", frequency, 100);
	
	api.rawControlPublisher(frequency);

	return(0);
}
