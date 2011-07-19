#ifndef __ROSCOAX__
#define __ROSCOAX__
#include <ros/ros.h>
#include <tf/tf.h>
#include <armadillo>

#include <nav_msgs/Odometry.h>
#include <coax_msgs/CoaxRawControl.h>

#include "CoaXModel.h"

class ROSCoaX
{
public:
  ROSCoaX(CoaXModel *model_, ros::NodeHandle &parent, const std::string &name_)
  {
    model = model_;
    name = name_;

    frame_id = name_;

    ros::NodeHandle n(parent, name);

    double odometry_rate;
    if (parent.getParam("rates/odometry", odometry_rate))
      {
        odometry_timer =
          parent.createTimer(ros::Rate(odometry_rate).expectedCycleTime(),
                             &ROSCoaX::odometry_callback, this);
        odometry_pub = n.advertise<nav_msgs::Odometry>("odom", 100);
      }

    cmd_sub = n.subscribe("cmd", 10, &ROSCoaX::CmdCallback, this,
                          ros::TransportHints().tcp().tcpNoDelay());
  }
  ~ROSCoaX()
  {
    if (odometry_timer.isValid())
      odometry_timer.stop();
  }

  void SetFrameId(const std::string &frame_id_)
  {
    frame_id = frame_id_;
  }

  void odometry_callback(const ros::TimerEvent& e)
  {
    ROS_DEBUG("Sending Odometry");

    double x, y, z;
    double roll, pitch, yaw;
    double vx, vy, vz;
    double wx, wy, wz;

    model->GetXYZ(x, y, z);
    model->GetRotation(roll, pitch, yaw);
    model->GetWorldLinearVelocity(vx, vy, vz);
    model->GetBodyAngularVelocity(wx, wy, wz);

    double c_r = cos(roll);
    double s_r = sin(roll);
    double c_p = cos(pitch);
    double s_p = sin(pitch);
    double c_y = cos(yaw);
    double s_y = sin(yaw);

    arma::mat Rb2w(3, 3);
    Rb2w(0, 0) = c_p*c_y;
    Rb2w(0, 1) = s_r*s_p*c_y - c_r*s_y;
    Rb2w(0, 2) = c_r*s_p*c_y + s_r*s_y;

    Rb2w(1, 0) = c_p*s_y;
    Rb2w(1, 1) = s_r*s_p*s_y + c_r*c_y;
    Rb2w(1, 2) = c_r*s_p*s_y - s_r*c_y;

    Rb2w(2, 0) = -s_p;
    Rb2w(2, 1) = s_r*c_p;
    Rb2w(2, 2) = c_r*c_p;

    btMatrix3x3 r(Rb2w(0, 0), Rb2w(0, 1), Rb2w(0, 2),
                  Rb2w(1, 0), Rb2w(1, 1), Rb2w(1, 2),
                  Rb2w(2, 0), Rb2w(2, 1), Rb2w(2, 2));

    btQuaternion quat;
    r.getRotation(quat);
    quat.normalize();

    // *** Load up and send the Odometry message
    odom_msg.header.frame_id = "/map";
    odom_msg.child_frame_id = frame_id;
    odom_msg.header.stamp = e.current_real;
    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.z = z;
    tf::quaternionTFToMsg(quat, odom_msg.pose.pose.orientation);
    odom_msg.twist.twist.linear.x = vx;
    odom_msg.twist.twist.linear.y = vy;
    odom_msg.twist.twist.linear.z = vz;
    odom_msg.twist.twist.angular.x = wx;
    odom_msg.twist.twist.angular.y = wy;
    odom_msg.twist.twist.angular.z = wz;

    odometry_pub.publish(odom_msg);
  }

  void CmdCallback(const coax_msgs::CoaxRawControl::ConstPtr& msg)
  {
    model->SetCommand(msg->motor1, msg->motor2,
                      msg->servo1, msg->servo2);
  }

private:
  std::string name;
  CoaXModel* model;

  ros::Subscriber cmd_sub;
  ros::Publisher odometry_pub;
  ros::Timer odometry_timer;

  std::string frame_id;

  nav_msgs::Odometry odom_msg;
};
#endif
