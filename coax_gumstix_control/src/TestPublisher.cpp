#include "ros/ros.h"
#include <stdlib.h>
#include <geometry_msgs/Quaternion.h>

int main(int argc, char **argv){
	
	ros::init(argc, argv, "fm_publisher");
	
	ros::NodeHandle n;
	
	ros::Publisher fm_pub = n.advertise<geometry_msgs::Quaternion>("fmdes", 1);
	
	ros::Rate loop_rate(100);
	
	while (ros::ok())
	{

		geometry_msgs::Quaternion fm_des;
		
		fm_des.x = 0.001*(rand() % 100);
		fm_des.y = -0.002*(rand() % 100);
		fm_des.z = 0.1*(rand() % 100);
		fm_des.w = 0.0000002*(rand() % 100);
		
		
		fm_pub.publish(fm_des);
		
		ros::spinOnce();
		
		loop_rate.sleep();
	}
}