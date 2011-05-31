#include "ros/ros.h"
#include "coax_msgs/CoaxSetRawControl.h"
#include "coax_msgs/CoaxControl.h"
#include "coax_msgs/CoaxSetControl.h"
#include "coax_msgs/CoaxConfigureControl.h"
#include "coax_msgs/CoaxReachNavState.h"

class CoaxControl
{
  protected:
  
  ros::ServiceClient setRawControl;
  ros::ServiceClient setControl;
  ros::ServiceClient configControlMode;
  ros::ServiceClient reachNavState;
  ros::Publisher control_pub;

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
  
  bool reach_nav_state(int des_state, float timeout)
  {
    coax_msgs::CoaxReachNavState srv;
    srv.request.desiredState = des_state;
    srv.request.timeout = timeout;
    if(reachNavState.call(srv)){
      ROS_INFO("Set nav_state to: %d, Result: %d", des_state, srv.response.result);
    }else{
      ROS_INFO("Failed to call service reach_nav_state");
    }

    return 0;
  }

  bool config_control(int rollMode, int pitchMode, int yawMode, int altitudeMode)
  {
    coax_msgs::CoaxConfigureControl srv;
    srv.request.rollMode = rollMode;
    srv.request.pitchMode = pitchMode;
    srv.request.yawMode = yawMode;
    srv.request.altitudeMode = altitudeMode;

    if(configControlMode.call(srv)){
      ROS_INFO("Control Modes: roll[%d] pitch[%d] yaw[%d] altitude[%d]", rollMode, pitchMode, yawMode, altitudeMode);
      ROS_INFO("Result: %d", srv.response.result);
    }else{
      ROS_INFO("Failed to call service configure_control");
    }

    return 0;
  }

  bool set_control(float roll, float pitch, float yaw, float altitude)
  {
    coax_msgs::CoaxSetControl srv;
    srv.request.roll = roll;
    srv.request.pitch = pitch;
    srv.request.yaw = yaw;
    srv.request.altitude = altitude;
    
    if(setControl.call(srv)){
      ROS_INFO("Control set to: roll[%f] pitch[%f] yaw[%f] altitude[%f]", roll, pitch, yaw, altitude);
      ROS_INFO("Result: %d", srv.response.result);
    }else{
      ROS_INFO("Failed to call service set_control");
    }

    return 0;
  }

  bool set_raw_control(float motor1, float motor2, float servo1, float servo2)
  {
    coax_msgs::CoaxSetRawControl srv;
    srv.request.motor1 = motor1;
    srv.request.motor2 = motor2;
    srv.request.servo1 = servo1;
    srv.request.servo2 = servo2;

    if(setRawControl.call(srv)){
      ROS_INFO("Inputs set to: m1[%f] m2[%f] s1[%f] s2[%f]", motor1, motor2, servo1, servo2);
      ROS_INFO("Result: %d", srv.response.result);
    }else{
      ROS_INFO("Failed to call service set_raw_control");
    }

    return 0;
  }

  void run(unsigned int rate)
  {
    ros::Rate loop_rate(rate);
    while (ros::ok())
    {
      coax_msgs::CoaxControl control;
      control.roll = 0.5;
      control.pitch = 0;
      control.yaw = 0;
      control.altitude = 0;
      control_pub.publish(control);
      ros::spinOnce();
      loop_rate.sleep();
    }
  }
	
};

int main(int argc, char **argv)
{

  ros::init(argc, argv, "coax_control");
  if(argc != 6)
  {
    ROS_INFO("usage: set(_raw)_control [motor1] [motor2] [servo1] [servo2]");
    return 1;
  }

  ros::NodeHandle n;

  CoaxControl api(n);

  //  api.config_control(1,1,1,1);

  api.reach_nav_state(1,5);
  
 
  //  api.set_control(atof(argv[1]), atof(argv[2]), atof(argv[3]), atof(argv[4]));
  //  api.set_raw_control(atof(argv[1]),atof(argv[2]),atof(argv[3]),atof(argv[4]));
  api.run(50);

  //  api.reach_nav_state(0,1);

  return 0;
}
