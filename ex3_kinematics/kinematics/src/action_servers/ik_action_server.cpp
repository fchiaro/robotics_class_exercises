#include <ros/ros.h>
#include <inverse_kinematics_action.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "IK_server");

  kinematics::InverseKinematicsAction action;
  
  ROS_INFO("IK server initialized");

  ros::spin();

  return 0;
}