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
		
		geometry_msgs::Pose coax_data;
		coax_data.position.x = message->gyro[0];
		coax_data.position.y = -message->gyro[1];
		coax_data.position.z = -message->gyro[2];
		coax_data.orientation.x = message->rcChannel[0];
		coax_data.orientation.y = message->rcChannel[2];
		coax_data.orientation.z = message->rcChannel[4];
		coax_data.orientation.w = message->rcChannel[6];

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
