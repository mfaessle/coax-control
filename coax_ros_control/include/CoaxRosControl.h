#ifndef __COAX_ROS_CONTROL__
#define __COAX_ROS_CONTROL__


#define CONTROL_LANDED 0 // State when helicopter has landed successfully
#define CONTROL_START 1 // Start / Takeoff
#define CONTROL_TRIM 2 // Trim Servos
#define CONTROL_HOVER 3 // Hover
#define CONTROL_GOTOPOS 4 // Go to Position
#define CONTROL_TRAJECTORY 5 // Follow Trajectory
#define CONTROL_LANDING 6 // Landing maneuver

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
	double acc[3];
} model_params_t;

typedef struct
{
	double K_lqr[4][16];
} control_params_t;


class CoaxRosControl
{
public:
	CoaxRosControl(ros::NodeHandle&);
	~CoaxRosControl();

	bool reachNavState(int des_state, float timeout);
	bool configureComm(int frequency, int contents);
	bool setTimeout(unsigned int control_timeout_ms, unsigned int watchdog_timeout_ms);
	
	void coaxStateCallback(const coax_msgs::CoaxState::ConstPtr & message);
	void coaxOdomCallback(const nav_msgs::Odometry::ConstPtr & message);
	
	void controlFunction(double* control, arma::colvec coax_state, arma::mat Rb2w, 
						 arma::colvec trajectory, model_params_t model_params, control_params_t control_params);
	void setControls(double* control);
	
	void rawControlPublisher(unsigned int rate);
	
	bool setControlMode(coax_ros_control::SetControlMode::Request &req, coax_ros_control::SetControlMode::Response &out);
	
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
	void load_model_params(ros::NodeHandle &n);
	
private:
	
	ros::ServiceClient reach_nav_state;
	ros::ServiceClient configure_comm;
	ros::ServiceClient set_timeout;
	
	ros::Publisher raw_control_pub;
	
	ros::Subscriber coax_odom_sub;
	ros::Subscriber coax_state_sub;
	
	std::vector<ros::ServiceServer> set_control_mode;
	
	model_params_t model_params;
	control_params_t control_params;
	
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
	double time_now;
	double time_prev;
	bool FIRST_RUN;
	double Omega_up;
	double Omega_lo;
	double prev_Omega_up;
	double prev_Omega_lo;
	double z_bar[3];
	double prev_z_bar[3];
	double start_position[3];
	double start_orientation;
	double start_time;
	double IDLE_TIME;
	double RISE_TIME;
	double RISE_VELOCITY;
	double START_HEIGHT;

};

#endif