#include "ros/ros.h"
#include "coax_msgs/CoaxSetRawControl.h"
#include "coax_msgs/CoaxSetControl.h"
#include "coax_msgs/CoaxRawControl.h"
#include "coax_msgs/CoaxControl.h"
#include "coax_msgs/CoaxConfigureControl.h"
#include "coax_msgs/CoaxReachNavState.h"

class CoaxTrajectoryControl
{
protected:
	
	
public:
	CoaxControl(ros::NodeHandle & n)
	{
		setRawControl = n.serviceClient<coax_msgs::CoaxSetRawControl>("coax_server/set_raw_control");
		setControl = n.serviceClient<coax_msgs::CoaxSetControl>("coax_server/set_control");
		configControlMode = n.serviceClient<coax_msgs::CoaxConfigureControl>("coax_server/configure_control");
		reachNavState = n.serviceClient<coax_msgs::CoaxReachNavState>("coax_server/reach_nav_state");
		control_pub = n.advertise<coax_msgs::CoaxControl>("coax_server/control",10);
	}
	~CoaxControl(){
	}

};