#include <ros/ros.h>
#include <coax_msgs/CoaxState.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>

class ManualSysid
{
protected:
	
	ros::Publisher coax_data_pub;
	
	ros::Subscriber coax_state_sub;
	
public:
	
	ManualSysid(ros::NodeHandle & n)
	{

		coax_data_pub = n.advertise<geometry_msgs::Pose>("coax_data",1);
		
		coax_state_sub = n.subscribe("state", 1, &ManualSysid::coaxStateCallback, this);

	}
	~ManualSysid(){
	}
	
	//===================
	// Callback Functions
	//===================
	
	void coaxStateCallback(const coax_msgs::CoaxState::ConstPtr & message)
	{
		/*
		Make sure in the main function of coax_interface that the raw control commands are included in the coax state message
		 -> SBS_O_ATTITUDE and SBS_O_ALTITUDE must be included in the communication configuration
		 -> api.configureComm(100, ... SBS_O_ATTITUDE | SBS_O_ALTITUDE);
		*/
		geometry_msgs::Pose coax_data;
		coax_data.position.x = message->gyro[0];
		coax_data.position.y = -message->gyro[1];
		coax_data.position.z = -message->gyro[2];
		coax_data.orientation.x = message->o_attitude[2]; // upper motor
		coax_data.orientation.y = message->o_altitude; // lower motor
		coax_data.orientation.z = message->o_attitude[0]; // roll servo
		coax_data.orientation.w = message->o_attitude[1]; // pitch servo

		coax_data_pub.publish(coax_data);
	}

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "manual_sysid");
	
	ros::NodeHandle n("/manual_sysid");
	
	ManualSysid api(n);

	ros::spin();
	
	return(0);
}
