#ifndef __COAX_ROS_CONTROL__
#define __COAX_ROS_CONTROL__

typedef struct
{
	double mass;
	double Ixx, Iyy, Izz;
	double d_up, d_lo;
	double k_springup, k_springlo;
	double l_up, l_lo;
	double k_Tup, k_Tlo;
	double k_Mup, k_Mlo;
	double Tf_motup, Tf_motlo;
	double Tf_up;
	double rs_mup, rs_bup;
	double rs_mlo, rs_blo;
	double zeta_mup, zeta_bup;
	double zeta_mlo, zeta_blo;
	double max_SPangle;
} model_params_t;

typedef struct
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
	
	double K_lqr[4][16];
} control_params_t;


class CoaxGumstixControl
{
public:
	CoaxGumstixControl(ros::NodeHandle&);
	~CoaxGumstixControl();

	bool reachNavState(int des_state, float timeout);
	bool configureComm(int frequency, int contents);
	bool setTimeout(unsigned int control_timeout_ms, unsigned int watchdog_timeout_ms);
	
	void coaxStateCallback(const coax_msgs::CoaxState::ConstPtr & message);
	void coaxOdomCallback(const nav_msgs::Odometry::ConstPtr & message);
	void coaxFMCallback(const geometry_msgs::Quaternion::ConstPtr & message);
	void matlabNavModeCallback(const std_msgs::Bool::ConstPtr & message);
	
	void controlFunction(double* control, double* FM_des, model_params_t model_params);
	void setControls(double* control);
	
	void rawControlPublisher(unsigned int rate);
	
	bool setControlMode(coax_gumstix_control::SetControlMode::Request &req, coax_gumstix_control::SetControlMode::Response &out);
		
	void SetPlatform(int CoaX);
	void SetMass(double mass);
	void SetInertia(double Ixx, double Iyy, double Izz);
	void SetRotorOffset(double d_up, double d_lo);
	void SetUpperRotorFollowingTime(double Tf_up);
	void SetRotorLinkageFactor(double l_up, double l_lo);
	void SetRotorSpringConstant(double k_springup, double k_springlo);
	void SetRotorThrustFactor(double k_Tup, double k_Tlo);
	void SetRotorMomentFactor(double k_Mup, double k_Mlo);
	void SetMotorFollowingTime(double Tf_motup, double Tf_motlo);
	void SetUpperRotorSpeedConversion(double rs_mup, double rs_bup);
	void SetLowerRotorSpeedConversion(double rs_mlo, double rs_blo);
	void SetUpperPhaseLag(double zeta_mup, double zeta_bup);
	void SetLowerPhaseLag(double zeta_mlo, double zeta_blo);
	void SetMaximumSwashPlateAngle(double max_SPangle);
	void SetForceGains(double Kp_Fx, double Kp_Fy, double Kp_Fz, double Kd_Fx, double Kd_Fy, double Kd_Fz);
	void SetYawMomentGains(double Kp_Mz, double Kd_Mz);
	void SetRateDampingGains(double K_pq_roll, double K_pq_pitch);
	void load_model_params(ros::NodeHandle &n);
	void load_control_params(ros::NodeHandle &n);
	
private:
	
	ros::ServiceClient reach_nav_state;
	ros::ServiceClient configure_comm;
	ros::ServiceClient set_timeout;
	
	ros::Publisher raw_control_pub;
	ros::Publisher coax_info_pub;
	ros::Publisher coax_imu_pub;
	ros::Publisher control_mode_pub;
	ros::Publisher raw_control_ipc_pub;
	
	ros::Subscriber coax_fmdes_sub;
	ros::Subscriber coax_odom_sub;
	ros::Subscriber coax_state_sub;
	ros::Subscriber matlab_nav_mode_sub;
	
	std::vector<ros::ServiceServer> set_control_mode;
	
	ros::TransportHints hints_fmdes;
	ros::TransportHints hints_odom;
	
	model_params_t model_params;
	control_params_t control_params;
	
	bool LOW_POWER_DETECTED;
	bool FIRST_RUN;
	bool MATLAB_ACTIVE;
	
	int COAX;
	
	int coax_state_age;
	int coax_nav_mode;
	int matlab_FM_age;
	int odom_age;
	
	double roll_trim;
	double pitch_trim;
	double motor_up;
	double motor_lo;
	double servo_roll;
	double servo_pitch;
	
	double time_now;
	double time_prev;
	
	double battery_voltage;
	double Omega_up;
	double Omega_lo;
	double prev_Omega_up;
	double prev_Omega_lo;
	double z_bar[3];
	double prev_z_bar[3];
	double prev_motor_up;
	double prev_motor_lo;
	
	double imu_p;
	double imu_q;
	double imu_r;
	double imu_roll;
	double imu_pitch;
	double imu_yaw;
	
	double yaw_est;
	double yaw_drift;
	int yaw_drift_count;
	bool YAW_DRIFT_COMP;
	
	double Rb2w[3][3];
	double FM_des[4];
	double position[3];
	double last_matactive_pos[3];
	double last_matactive_ori;

};

#endif