#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <coax_msgs/CoaxState.h>
#include <coax_msgs/CoaxReachNavState.h>
#include <coax_msgs/CoaxConfigureComm.h>
#include <coax_msgs/CoaxSetTimeout.h>
#include <coax_msgs/CoaxRawControl.h>
#include <coax_gumstix_control/CoaxFMdes.h>
#include <coax_gumstix_control/SetControlMode.h>

#include <com/sbapi.h>
#include <CoaxGumstixControl.h>


CoaxGumstixControl::CoaxGumstixControl(ros::NodeHandle & n)
{
	reach_nav_state = n.serviceClient<coax_msgs::CoaxReachNavState>("reach_nav_state");
	configure_comm = n.serviceClient<coax_msgs::CoaxConfigureComm>("configure_comm");
	set_timeout = n.serviceClient<coax_msgs::CoaxSetTimeout>("set_timeout");
		
	raw_control_pub = n.advertise<coax_msgs::CoaxRawControl>("rawcontrol",1);
	coax_info_pub = n.advertise<geometry_msgs::Quaternion>("info",1);
	coax_imu_pub = n.advertise<geometry_msgs::Quaternion>("imu",1);
	control_mode_pub = n.advertise<geometry_msgs::Quaternion>("control_mode",1);
	raw_control_ipc_pub = n.advertise<geometry_msgs::Quaternion>("rawcontrol_ipc",1);
	
	coax_fmdes_sub = n.subscribe("fmdes", 1, &CoaxGumstixControl::coaxFMCallback, this, hints_fmdes.udp());
	coax_odom_sub = n.subscribe("odom", 1, &CoaxGumstixControl::coaxOdomCallback, this, hints_odom.udp());
	coax_state_sub = n.subscribe("state", 1, &CoaxGumstixControl::coaxStateCallback, this);
	matlab_nav_mode_sub = n.subscribe("nav_mode", 1, &CoaxGumstixControl::matlabNavModeCallback, this);
	
	set_control_mode.push_back(n.advertiseService("set_control_mode", &CoaxGumstixControl::setControlMode, this));
	
	LOW_POWER_DETECTED = false;
	FIRST_RUN = true;
	MATLAB_ACTIVE = true;
	
	battery_voltage = 12.22;
	coax_state_age = 0;
	coax_nav_mode = 0;
	matlab_FM_age = 0;
	odom_age = 0;
	
	roll_trim = 0.0;
	pitch_trim = 0.0;
	motor_up = prev_motor_up = 0;
	motor_lo = prev_motor_lo = 0;
	servo_roll = 0;
	servo_pitch = 0;
	
	Omega_up = prev_Omega_up = 0;
	Omega_lo = prev_Omega_lo = 0;
	z_bar[0] = z_bar[1] = 0;
	z_bar[2] = 1;
	prev_z_bar[0] = prev_z_bar[1] = 0;
	prev_z_bar[2] = 1;
	
	imu_p = 0;
	imu_q = 0;
	imu_r = 0;
	imu_roll = 0;
	imu_pitch = 0;
	imu_yaw = 0;
	
	yaw_est = 0;
	yaw_drift = 0;
	yaw_drift_count = 0;
	YAW_DRIFT_COMP = true;
	
	FM_des[0] = 0;
	FM_des[1] = 0;
	FM_des[2] = 0;
	FM_des[3] = 0;
	
	position[0] = position[1] = position[2] = 0;
	
}
CoaxGumstixControl::~CoaxGumstixControl()
{

}
	
//===================
// Service Clients
//===================
	
bool CoaxGumstixControl::reachNavState(int des_state, float timeout)
{
	coax_msgs::CoaxReachNavState srv;
	srv.request.desiredState = des_state;
	srv.request.timeout = timeout;
	reach_nav_state.call(srv);
	
	return 0;
}

bool CoaxGumstixControl::configureComm(int frequency, int contents)
{
	coax_msgs::CoaxConfigureComm srv;
	srv.request.frequency = frequency;
	srv.request.contents = contents;
	configure_comm.call(srv);
	
	return 0;
}

bool CoaxGumstixControl::setTimeout(unsigned int control_timeout_ms, unsigned int watchdog_timeout_ms)
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

void CoaxGumstixControl::coaxStateCallback(const coax_msgs::CoaxState::ConstPtr & message)
{
	battery_voltage = 0.8817*message->battery + 1.5299;
	coax_nav_mode = message->mode.navigation;
	
	if ((battery_voltage < 10.80) && !LOW_POWER_DETECTED){
		ROS_INFO("Battery Low!!! (%fV)",battery_voltage);
		LOW_POWER_DETECTED = true;
	}
	
	imu_p = message->gyro[0];
	imu_q = -message->gyro[1];
	imu_r = -message->gyro[2];
	
	imu_roll = message->roll;
	imu_pitch = -message->pitch;
	imu_yaw = -message->yaw;
	
	coax_state_age = 0;
	
	// send battery state and navigation mode to Matlab
	geometry_msgs::Quaternion coax_info;
	coax_info.x = message->mode.navigation;
	coax_info.y = message->battery;
	coax_info.z = 0;
	coax_info.w = 0;
	
	coax_info_pub.publish(coax_info);
	
	// send body rates from IMU to Matlab
	geometry_msgs::Quaternion coax_imu;
	coax_imu.x = message->gyro[0];
	coax_imu.y = -message->gyro[1];
	coax_imu.z = -message->gyro[2];
	coax_imu.w = 0;
	
	coax_imu_pub.publish(coax_imu);
}

void CoaxGumstixControl::coaxOdomCallback(const nav_msgs::Odometry::ConstPtr & message)
{
	
	position[0] = message->pose.pose.position.x;
	position[1] = message->pose.pose.position.y;
	position[2] = message->pose.pose.position.z;
	
	double velocity[3];
	velocity[0] = message->twist.twist.linear.x;
	velocity[1] = message->twist.twist.linear.y;
	velocity[2] = message->twist.twist.linear.z;
	
	// Get orientation quaternion
	double qx = message->pose.pose.orientation.x;
	double qy = message->pose.pose.orientation.y;
	double qz = message->pose.pose.orientation.z;
	double qw = message->pose.pose.orientation.w;
	double Rb2w_vicon[3][3];
	double vicon_yaw;
	
	if (YAW_DRIFT_COMP) { // compute yaw drift here since we do not necessarily get vicon values at raw_control publishing frequency 
		// Compute rotation matrix from Vicon data
		Rb2w_vicon[0][0] = 1-2*qy*qy-2*qz*qz;
		Rb2w_vicon[0][1] = 2*qx*qy-2*qz*qw;
		Rb2w_vicon[0][2] = 2*qx*qz+2*qy*qw;
		Rb2w_vicon[1][0] = 2*qx*qy+2*qz*qw;
		Rb2w_vicon[1][1] = 1-2*qx*qx-2*qz*qz;
		Rb2w_vicon[1][2] = 2*qy*qz-2*qx*qw;
		Rb2w_vicon[2][0] = 2*qx*qz-2*qy*qw;
		Rb2w_vicon[2][1] = 2*qy*qz+2*qx*qw;
		Rb2w_vicon[2][2] = 1-2*qx*qx-2*qy*qy;
		
		// compute yaw angle from vicon Rb2w -> vicon_yaw
		vicon_yaw = atan2(-Rb2w_vicon[0][1],Rb2w_vicon[0][0]); // Skybotix Trafo !!!
		
		yaw_drift = imu_yaw - vicon_yaw;
		while (imu_yaw-yaw_drift > M_PI) {
			yaw_drift += 2*M_PI;
		}
		while (imu_yaw-yaw_drift < -M_PI) {
			yaw_drift -= 2*M_PI;
		}

		YAW_DRIFT_COMP = false;
	}
	
	// getting current time
	//time_now = ros::Time::now().toSec();
	if (FIRST_RUN){
		//time_prev = time_now;
		last_matactive_pos[0] = position[0];
		last_matactive_pos[1] = position[1];
		last_matactive_pos[2] = position[2];
		last_matactive_ori = atan2(2*qx*qy+2*qz*qw,1-2*qy*qy-2*qz*qz);
		FIRST_RUN = false;
	}
	//double delta_t = time_now - time_prev;
	
	// body rates
	double p = imu_p;//message->twist.twist.angular.x;
	double q = imu_q;//message->twist.twist.angular.y;
	double r = imu_r;
	
	double control[4] = {};
	double ori_error;
	double FM_des_damp[4];
	double FD_body[2];
	
	
	if (!MATLAB_ACTIVE) {
		if (LOW_POWER_DETECTED) {
			//Landing ...
			// control =
			// set control commands
			//setControls(control);
		} else if (coax_nav_mode != 0) {
			//stabilizing control ...
			FD_body[0] = control_params.K_pq_pitch*q;
			FD_body[1] = control_params.K_pq_roll*p;
			
			FM_des_damp[0] = -control_params.Kp_Fx*(position[0]-last_matactive_pos[0]) - control_params.Kd_Fx*(velocity[0]) - Rb2w[0][0]*FD_body[0] - Rb2w[0][1]*FD_body[1];
			FM_des_damp[1] = -control_params.Kp_Fy*(position[1]-last_matactive_pos[1]) - control_params.Kd_Fy*(velocity[1]) - Rb2w[1][0]*FD_body[0] - Rb2w[1][1]*FD_body[1];
			FM_des_damp[2] = -control_params.Kp_Fz*(position[2]-last_matactive_pos[2]) - control_params.Kd_Fz*(velocity[2]) + model_params.mass*9.81;
			
			ori_error = atan2(Rb2w[1][0],Rb2w[0][0]) - last_matactive_ori;
			while (ori_error > M_PI) {
				ori_error = ori_error - 2*M_PI;
			}
			while (ori_error < -M_PI) {
				ori_error = ori_error + 2*M_PI;
			}
			FM_des_damp[3] = -control_params.Kp_Mz*ori_error - control_params.Kd_Mz*r;
						
			controlFunction(control, FM_des_damp, model_params);
		}
		
		// set control commands
		setControls(control);
	}

	odom_age = 0;
	
	//time_prev = time_now;
}

//void CoaxGumstixControl::coaxFMCallback(const coax_gumstix_control::CoaxFMdes::ConstPtr & message)
void CoaxGumstixControl::coaxFMCallback(const geometry_msgs::Quaternion::ConstPtr & message)
{
	
	//Desired Forces and Yaw Moment
	//FM_des[0] = message->Fx_des;
	//FM_des[1] = message->Fy_des;
	//FM_des[2] = message->Fz_des;
	//FM_des[3] = message->Mz_des;
	FM_des[0] = message->x;
	FM_des[1] = message->y;
	FM_des[2] = message->z;
	FM_des[3] = message->w;
	
	last_matactive_pos[0] = position[0];
	last_matactive_pos[1] = position[1];
	last_matactive_pos[2] = position[2];
	last_matactive_ori = atan2(Rb2w[1][0],Rb2w[0][0]);
	
	matlab_FM_age = 0;
	MATLAB_ACTIVE = true;
}

void CoaxGumstixControl::matlabNavModeCallback(const std_msgs::Bool::ConstPtr & message)
{
	motor_up = 0;
	motor_lo = 0;
	servo_roll = 0;
	servo_pitch = 0;
	bool result;
	
	if (message->data) {
		result = reachNavState(SB_NAV_RAW, 0.5); // Navigation raw mode
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
	}
}
										
//===================
// Control functions
//===================

void CoaxGumstixControl::controlFunction(double* control, double* FM_des, model_params_t model_params)
{
	
	// Parameters
	//double g = 9.81;
	//double m = model_params.mass;
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
	
	// Upper thrust vector direction
	double z_Tupz = cos(l_up*acos(z_bar[2]));
	double z_Tup_p[3];
	double temp;
	if (z_Tupz < 1){
		temp = sqrt((1-z_Tupz*z_Tupz)/(z_bar[0]*z_bar[0] + z_bar[1]*z_bar[1]));
		z_Tup_p[0] = z_bar[0]*temp;
		z_Tup_p[1] = z_bar[1]*temp;
		z_Tup_p[2] = z_Tupz;
	}else {
		z_Tup_p[0] = 0;
		z_Tup_p[1] = 0;
		z_Tup_p[2] = 1;
	}
	
	double zeta = zeta_mup*Omega_up + zeta_bup;
	double z_Tup[3];
	z_Tup[0] = cos(zeta)*z_Tup_p[0] - sin(zeta)*z_Tup_p[1];
	z_Tup[1] = sin(zeta)*z_Tup_p[0] + cos(zeta)*z_Tup_p[1];
	z_Tup[2] = z_Tup_p[2];
	
	
	// Lower thrust vector direction
	double z_Tlo[3];
	if (Omega_lo < 10) {
		z_Tlo[0] = 0;
		z_Tlo[1] = 0;
		z_Tlo[2] = 1;
	} else {
		z_Tlo[0] = 1/(k_Tlo*Omega_lo*Omega_lo)*(Rb2w[0][0]*FM_des[0] + Rb2w[1][0]*FM_des[1]);
		z_Tlo[1] = 1/(k_Tlo*Omega_lo*Omega_lo)*(Rb2w[0][1]*FM_des[0] + Rb2w[1][1]*FM_des[1]);
		z_Tlo[2] = sqrt(1-z_Tlo[0]*z_Tlo[0]-z_Tlo[1]*z_Tlo[1]);
	}
	/*
	double c = 0.45;
	double a_lo = -c*(Rb2w[0][1]*FM_des[0] + Rb2w[1][1]*FM_des[1]);
	double b_lo = c*(Rb2w[0][0]*FM_des[0] + Rb2w[1][0]*FM_des[1]);
	z_Tlo[0] = cos(a_lo)*sin(b_lo);
	z_Tlo[1] = -sin(a_lo);
	z_Tlo[2] = cos(a_lo)*cos(b_lo);
	*/
	
	// correct for phase lag of servo inputs
	zeta = zeta_mlo*Omega_lo + zeta_blo;
	double z_Tlo_p[3];
	z_Tlo_p[0] = cos(zeta)*z_Tlo[0] - sin(zeta)*z_Tlo[1];
	z_Tlo_p[1] = sin(zeta)*z_Tlo[0] + cos(zeta)*z_Tlo[1];
	z_Tlo_p[2] = z_Tlo[2];
	
	double z_SP[3];
	double z_SPz = cos(1/l_lo*acos(z_Tlo_p[2]));
	if (z_SPz < 1){
		temp = sqrt((1-z_SPz*z_SPz)/(z_Tlo_p[0]*z_Tlo_p[0] + z_Tlo_p[1]*z_Tlo_p[1]));
		z_SP[0] = z_Tlo_p[0]*temp;
		z_SP[1] = z_Tlo_p[1]*temp;
		z_SP[2] = z_SPz;
	}else{
		z_SP[0] = 0;
		z_SP[1] = 0;
		z_SP[2] = 1;
	}
	double b_lo_des = asin(z_SP[0]);
	double a_lo_des = asin(-z_SP[1]/cos(b_lo_des));
	
	control[2] = a_lo_des/max_SPangle;
	control[3] = b_lo_des/max_SPangle;
	
	// New heave-yaw control
	double Fz_des = FM_des[2];
	double Mz_des = FM_des[3];
	
	double A = k_Tup/k_Mup*Mz_des*(Rb2w[2][0]*z_Tup[0] + Rb2w[2][1]*z_Tup[1] + Rb2w[2][2]*z_Tup[2]);
	double B = k_Tup/k_Mup*k_Mlo*(Rb2w[2][0]*z_Tup[0] + Rb2w[2][1]*z_Tup[1] + Rb2w[2][2]*z_Tup[2]) + k_Tlo*(Rb2w[2][0]*z_Tlo[0] + Rb2w[2][1]*z_Tlo[1] + Rb2w[2][2]*z_Tlo[2]);
	
	double Omega_lo_des = sqrt((A + Fz_des)/B);
	double Omega_up_des = sqrt((k_Mlo*Omega_lo_des*Omega_lo_des - Mz_des)/k_Mup);
	control[0] = (Omega_up_des - rs_bup)/rs_mup;
	control[1] = (Omega_lo_des - rs_blo)/rs_mlo;
	
}

void CoaxGumstixControl::setControls(double* control)
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

//===================
// Publisher
//===================

void CoaxGumstixControl::rawControlPublisher(unsigned int rate)
{
	ros::Rate loop_rate(rate);
	
	double delta_t = 1/rate;
	
	coax_msgs::CoaxRawControl raw_control;
	geometry_msgs::Quaternion raw_control_ipc;

	while(ros::ok())
	{		
		// compensate yaw angle with skybotix trafo -> yaw_est
		yaw_drift -= 0.0218534/rate; // compensate for constant drift rate
		while (imu_yaw-yaw_drift > M_PI) {
			yaw_drift += 2*M_PI;
		}
		while (imu_yaw-yaw_drift < -M_PI) {
			yaw_drift -= 2*M_PI;
		}
		yaw_est = imu_yaw - yaw_drift;
		
		// Compute Rb2w from imu euler angles here using imu_roll, imu_pitch, yaw_est
		Rb2w[0][0] = cos(imu_pitch)*cos(yaw_est);
		Rb2w[0][1] = -cos(imu_pitch)*sin(yaw_est);
		Rb2w[0][2] = sin(imu_pitch);
		Rb2w[1][0] = cos(imu_roll)*sin(yaw_est) + cos(yaw_est)*sin(imu_roll)*sin(imu_pitch);
		Rb2w[1][1] = cos(imu_roll)*cos(yaw_est) - sin(yaw_est)*sin(imu_roll)*sin(imu_pitch);
		Rb2w[1][2] = -cos(imu_pitch)*sin(imu_roll);
		Rb2w[2][0] = sin(imu_roll)*sin(yaw_est) - cos(yaw_est)*cos(imu_roll)*sin(imu_pitch);
		Rb2w[2][1] = sin(imu_roll)*cos(yaw_est) + sin(yaw_est)*cos(imu_roll)*sin(imu_pitch);
		Rb2w[2][2] = cos(imu_roll)*cos(imu_pitch);
		
		// body rates
		double p = imu_p;//message->twist.twist.angular.x;
		double q = imu_q;//message->twist.twist.angular.y;
		double r = imu_r;
		
		// Estimate stabilizer bar orientation
		double b_z_bardotz = 1/model_params.Tf_up*acos(prev_z_bar[2])*sqrt(prev_z_bar[0]*prev_z_bar[0] + prev_z_bar[1]*prev_z_bar[1]);
		double b_z_bardot[3] = {0,0,0};
		if (b_z_bardotz > 0){
			double temp = prev_z_bar[2]*b_z_bardotz/(prev_z_bar[0]*prev_z_bar[0]+prev_z_bar[1]*prev_z_bar[1]);
			b_z_bardot[0] = -prev_z_bar[0]*temp;
			b_z_bardot[1] = -prev_z_bar[1]*temp;
			b_z_bardot[2] = b_z_bardotz;
		}
		
		z_bar[0] = prev_z_bar[0] + (r*prev_z_bar[1] - q*prev_z_bar[2] + b_z_bardot[0])*delta_t;
		z_bar[1] = prev_z_bar[1] + (-r*prev_z_bar[0] + p*prev_z_bar[2] + b_z_bardot[1])*delta_t;
		z_bar[2] = prev_z_bar[2] + (q*prev_z_bar[0] - p*prev_z_bar[1] + b_z_bardot[2])*delta_t;
		
		double norm_z_bar = sqrt(z_bar[0]*z_bar[0] + z_bar[1]*z_bar[1] + z_bar[2]*z_bar[2]);
		z_bar[0] = z_bar[0]/norm_z_bar;
		z_bar[1] = z_bar[1]/norm_z_bar;
		z_bar[2] = z_bar[2]/norm_z_bar;
		
		prev_z_bar[0] = z_bar[0];
		prev_z_bar[1] = z_bar[1];
		prev_z_bar[2] = z_bar[2];
		
		double control[4] = {};
		double FM_des_damp[4];
		double FD_body[2];
		
		
		if (MATLAB_ACTIVE) {
			
			// roll/pitch damping
			FD_body[0] = control_params.K_pq_pitch*q;
			FD_body[1] = control_params.K_pq_roll*p;
			
			FM_des_damp[0] = FM_des[0] - Rb2w[0][0]*FD_body[0] - Rb2w[0][1]*FD_body[1];
			FM_des_damp[1] = FM_des[1] - Rb2w[1][0]*FD_body[0] - Rb2w[1][1]*FD_body[1];
			FM_des_damp[2] = FM_des[2];
			FM_des_damp[3] = FM_des[3];
			
			// Compute control inputs
			controlFunction(control, FM_des_damp, model_params);
			
			// Voltage compensation
			double volt_compUp = (12.22 - battery_voltage)*0.0279;
			double volt_compLo = (12.22 - battery_voltage)*0.0287;
			if (control[0] > 0.05 || control[1] > 0.05){
				control[0] = control[0] + volt_compUp;
				control[1] = control[1] + volt_compLo;
			}
			
			// set control commands
			setControls(control);			
		}
		
		if (odom_age < 50) {
			raw_control.motor1 = motor_up;
			raw_control.motor2 = motor_lo;
			raw_control.servo1 = servo_roll + roll_trim;
			raw_control.servo2 = servo_pitch + pitch_trim;
		} else {// no more new Odometry information from Vicon
			raw_control.motor1 = 0;
			raw_control.motor2 = 0;
			raw_control.servo1 = 0;
			raw_control.servo2 = 0;
		}
		
		raw_control_ipc.x = raw_control.motor1;
		raw_control_ipc.y = raw_control.motor2;
		raw_control_ipc.z = raw_control.servo1;
		raw_control_ipc.w = raw_control.servo2;
		
		raw_control_pub.publish(raw_control);
		raw_control_ipc_pub.publish(raw_control_ipc);
		
		if ((coax_state_age > 0.5*rate) && (coax_state_age <= 0.5*rate + 1)) {
			ROS_INFO("No more Data from CoaX Board!!!");
		}
		if (matlab_FM_age > 20 && MATLAB_ACTIVE) { // Matlab not active anymore -> swit$
			MATLAB_ACTIVE = false;
			//ROS_INFO("Matlab is not active!!!");
		}
		if ((matlab_FM_age > 3) && (matlab_FM_age < 100)){ // For network testing
			ROS_INFO("Matlab value age [%d]",matlab_FM_age);
		}
		if ((odom_age > 3) && (odom_age < 100)){ // For network testing
			ROS_INFO("Vicon value age [%d]",odom_age);
		}
		
		if (coax_state_age < 1000) {
			coax_state_age += 1;
		}
		if (matlab_FM_age < 1000) {
			matlab_FM_age += 1;
		}
		if (odom_age < 1000) {
			odom_age += 1;
		}
		if (yaw_drift_count++ > 10) {
			YAW_DRIFT_COMP = true;
			yaw_drift_count = 0;
		}
		
		// Estimate rotor speeds (do that here to have the same frequency as we apply inputs)
		double prev_Omega_up_des = model_params.rs_mup*prev_motor_up + model_params.rs_bup;
		double prev_Omega_lo_des = model_params.rs_mlo*prev_motor_lo + model_params.rs_blo;
		
		Omega_up = prev_Omega_up + 1/model_params.Tf_motup*(prev_Omega_up_des - prev_Omega_up)/rate;
		Omega_lo = prev_Omega_lo + 1/model_params.Tf_motlo*(prev_Omega_lo_des - prev_Omega_lo)/rate;
		prev_Omega_up = Omega_up;
		prev_Omega_lo = Omega_lo;
		prev_motor_up = raw_control.motor1;
		prev_motor_lo = raw_control.motor2;
		
		ros::spinOnce();
		loop_rate.sleep();
	}
}

//===================
// Services
//===================

bool CoaxGumstixControl::setControlMode(coax_gumstix_control::SetControlMode::Request &req, coax_gumstix_control::SetControlMode::Response &out)
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
		}
	}
	
	out.result = 1;
	return true;
}

//===================
// Parameter Setting
//===================

void CoaxGumstixControl::SetPlatform(int CoaX)
{
	COAX = CoaX;
}

void CoaxGumstixControl::SetMass(double mass)
{
	model_params.mass = mass;
}

void CoaxGumstixControl::SetInertia(double Ixx, double Iyy, double Izz)
{
	model_params.Ixx = Ixx;
	model_params.Iyy = Iyy;
	model_params.Izz = Izz;
}

void CoaxGumstixControl::SetRotorOffset(double d_up, double d_lo)
{
	model_params.d_up = d_up;
	model_params.d_lo = d_lo;
}

void CoaxGumstixControl::SetUpperRotorFollowingTime(double Tf_up)
{
	model_params.Tf_up = Tf_up;
}

void CoaxGumstixControl::SetRotorLinkageFactor(double l_up, double l_lo)
{
	model_params.l_up = l_up;
	model_params.l_lo = l_lo;
}

void CoaxGumstixControl::SetRotorSpringConstant(double k_springup, double k_springlo)
{
	model_params.k_springup = k_springup;
	model_params.k_springlo = k_springlo;
}

void CoaxGumstixControl::SetRotorThrustFactor(double k_Tup, double k_Tlo)
{
	model_params.k_Tup = k_Tup;
	model_params.k_Tlo = k_Tlo;
}

void CoaxGumstixControl::SetRotorMomentFactor(double k_Mup, double k_Mlo)
{
	model_params.k_Mup = k_Mup;
	model_params.k_Mlo = k_Mlo;
}

void CoaxGumstixControl::SetMotorFollowingTime(double Tf_motup, double Tf_motlo)
{
	model_params.Tf_motup = Tf_motup;
	model_params.Tf_motlo = Tf_motlo;
}

void CoaxGumstixControl::SetUpperRotorSpeedConversion(double rs_mup, double rs_bup)
{
	model_params.rs_mup = rs_mup;
	model_params.rs_bup = rs_bup;
}

void CoaxGumstixControl::SetLowerRotorSpeedConversion(double rs_mlo, double rs_blo)
{
	model_params.rs_mlo = rs_mlo;
	model_params.rs_blo = rs_blo;
}

void CoaxGumstixControl::SetUpperPhaseLag(double zeta_mup, double zeta_bup)
{
	model_params.zeta_mup = zeta_mup;
	model_params.zeta_bup = zeta_bup;
}

void CoaxGumstixControl::SetLowerPhaseLag(double zeta_mlo, double zeta_blo)
{
	model_params.zeta_mlo = zeta_mlo;
	model_params.zeta_blo = zeta_blo;
}

void CoaxGumstixControl::SetMaximumSwashPlateAngle(double max_SPangle)
{
	model_params.max_SPangle = max_SPangle;
}

void CoaxGumstixControl::SetForceGains(double Kp_Fx, double Kp_Fy, double Kp_Fz, double Kd_Fx, double Kd_Fy, double Kd_Fz)
{
	control_params.Kp_Fx = Kp_Fx;
	control_params.Kp_Fy = Kp_Fy;
	control_params.Kp_Fz = Kp_Fz;
	
	control_params.Kd_Fx = Kd_Fx;
	control_params.Kd_Fy = Kd_Fy;
	control_params.Kd_Fz = Kd_Fz;
}

void CoaxGumstixControl::SetYawMomentGains(double Kp_Mz, double Kd_Mz)
{
	control_params.Kp_Mz = Kp_Mz;
	control_params.Kd_Mz = Kd_Mz;
}

void CoaxGumstixControl::SetRateDampingGains(double K_pq_roll, double K_pq_pitch)
{
	control_params.K_pq_roll = K_pq_roll;
	control_params.K_pq_pitch = K_pq_pitch;
}

void CoaxGumstixControl::load_model_params(ros::NodeHandle &n)
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

void CoaxGumstixControl::load_control_params(ros::NodeHandle &n)
{
	double Kp_Fx;
	double Kp_Fy;
	double Kp_Fz;
	
	double Kd_Fx;
	double Kd_Fy;
	double Kd_Fz;
	
	double Kp_Mz;
	double Kd_Mz;
	double K_pq_roll;
	double K_pq_pitch;
	n.getParam("force/proportional/x",Kp_Fx);
	n.getParam("force/proportional/y",Kp_Fy);
	n.getParam("force/proportional/z",Kp_Fz);
	
	n.getParam("force/differential/x",Kd_Fx);
	n.getParam("force/differential/y",Kd_Fy);
	n.getParam("force/differential/z",Kd_Fz);
	
	n.getParam("yawmoment/proportional",Kp_Mz);
	n.getParam("yawmoment/differential",Kd_Mz);
	n.getParam("rate_damping/roll",K_pq_roll);
	n.getParam("rate_damping/pitch",K_pq_pitch);
	SetForceGains(Kp_Fx, Kp_Fy, Kp_Fz, Kd_Fx, Kd_Fy, Kd_Fz);
	SetYawMomentGains(Kp_Mz, Kd_Mz);
	SetRateDampingGains(K_pq_roll,K_pq_pitch);
}



int main(int argc, char** argv)
{
	ros::init(argc, argv, "coax_gumstix_control");
	
	ros::NodeHandle n("/coax_gumstix_control");
	
	CoaxGumstixControl api(n);
	
	ros::Duration(1.5).sleep(); // make sure coax_server has enough time to boot up
	
	int comm_freq;
	n.param("comm_freq", comm_freq, 100);
	api.configureComm(comm_freq, SBS_MODES | SBS_BATTERY | SBS_GYRO | SBS_RPY); // configuration of sending back data from CoaX
	api.setTimeout(500, 5000);
	
	int CoaX;
	n.param("CoaX", CoaX, 56);
	api.SetPlatform(CoaX);
	
	api.load_model_params(n);
	
	api.load_control_params(n);
	
	int pub_freq;
	n.param("pub_freq", pub_freq, 100);
	
	api.rawControlPublisher(pub_freq);

	return(0);
}
