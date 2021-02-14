#include <ros/ros.h>
#include <kinematics_msgs/forward_kinematics.h>
#include <kinematics_msgs/forward_kinematicsRequest.h>
#include <kinematics_msgs/forward_kinematicsResponse.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Geometry>
#include <geometry_msgs/Pose.h>


// bool return type is required by ROS
bool compute_fk(kinematics_msgs::forward_kinematicsRequest &req, kinematics_msgs::forward_kinematicsResponse &resp)
{        
    ROS_INFO_STREAM("Requested forward kinematics for '" << req.link << "' link");

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));

    if(!(req.joints_state.empty() || req.joint_group.empty()))
    {
        kinematic_state->setJointGroupPositions(req.joint_group, req.joints_state);
    }
    kinematic_state->updateLinkTransforms();

    const Eigen::Isometry3d& link_fk = kinematic_state->getGlobalLinkTransform(req.link);
    geometry_msgs::Pose pose_msg;
    tf::poseEigenToMsg(link_fk, pose_msg);

    resp.pose.pose = pose_msg;
    resp.pose.header.frame_id = kinematic_model->getModelFrame();
    resp.pose.header.stamp = ros::Time::now();

    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "FK_server");
    ros::NodeHandle node_handle;

    ros::ServiceServer service = node_handle.advertiseService("forward_kinematics", compute_fk);
    ROS_INFO("FK server initialized");
    ros::spin();

    return 0;
}