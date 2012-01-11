#include "ros/ros.h"
#include <geometry_msgs/Quaternion.h>

int fm_age = 0;

void fmCallback(const geometry_msgs::Quaternion::ConstPtr& msg)
{
	fm_age = 0;
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "fm_subscriber");
	
	ros::NodeHandle n;
	

	ros::Subscriber sub = n.subscribe("fmdes", 1, fmCallback);
	
	
	
	ros::Rate loop_rate(100);

	while (ros::ok())
	{
		
		fm_age += 1;
		
		if ((fm_age > 1) && (fm_age < 100)) {
			ROS_INFO("No fm received! age=[%d]",fm_age);
		}
		
		
		ros::spinOnce();
		
		loop_rate.sleep();
	}
	
	return 0;
}