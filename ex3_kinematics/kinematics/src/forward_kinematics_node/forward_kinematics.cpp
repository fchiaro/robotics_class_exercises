#include <ros/ros.h>
#include <kinematics_msgs/forward_kinematics.h>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "forward_kinematics_client");
    ros::NodeHandle nh;

    // Get parameters from parameter server
    std::vector<double> joints_state;
    const std::vector<double> JOINTS_STATE_DEFAULT = {0.0, 0.5, 0.0, 0.0, 0.0, 0.0};
    if(!ros::param::param<std::vector<double>>("/joints_state", joints_state, JOINTS_STATE_DEFAULT))
    {
        ROS_WARN("Could not load user-defined joints state, going with the default one...");
    }

    std::string joint_group; 
    if(!ros::param::param<std::string>("/joint_group", joint_group, "all_joints"))
    {
        ROS_WARN("Could not load user-defined joint group, going with the default one...");
    }

    std::string desired_link; 
    if(!ros::param::param<std::string>("/desired_link", desired_link, "flange"))
    {
        ROS_WARN("Could not load user-defined link, going with the default one...");
    }

    // My forward kinematics
    ros::ServiceClient custom_fk_client = nh.serviceClient<kinematics_msgs::forward_kinematics>("forward_kinematics");
    kinematics_msgs::forward_kinematics custom_fk_service_handler;
    custom_fk_service_handler.request.link = desired_link;
    custom_fk_service_handler.request.joint_group = joint_group;
    custom_fk_service_handler.request.joints_state = joints_state;
    
    if(custom_fk_client.call(custom_fk_service_handler))
    {
        ROS_INFO_STREAM("Custom forward kinematics for '" << custom_fk_service_handler.request.link 
                        <<"' link:\n" << custom_fk_service_handler.response);
    }
    else
    {
        ROS_ERROR("Failed to obtain forward kinematics from dedicated server!");
        return 1;
    }

    // move_group's forward kinematics
    ros::ServiceClient moveit_fk_client = nh.serviceClient<moveit_msgs::GetPositionFK>("compute_fk");
    moveit_msgs::GetPositionFK moveit_fk_service_handler;

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup(joint_group);
    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

    moveit_fk_service_handler.request.fk_link_names.push_back(desired_link);
    moveit_fk_service_handler.request.header.frame_id = kinematic_model->getModelFrame();
    moveit_fk_service_handler.request.robot_state.joint_state.name = joint_names;

    std::vector<double> joint_values;
    if(custom_fk_service_handler.request.joints_state.empty() || custom_fk_service_handler.request.joint_group.empty())
    {
        // Custom server will compute FK wrt available joint values
        kinematic_state->updateLinkTransforms();
        kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    }
    else
    {
        joint_values = joints_state;
    }
    
    moveit_fk_service_handler.request.robot_state.joint_state.position = joint_values;

    if(moveit_fk_client.call(moveit_fk_service_handler))
    {
        ROS_INFO_STREAM("Moveit forward kinematics for '" << moveit_fk_service_handler.request.fk_link_names[0] <<"' link:\n" 
                        << moveit_fk_service_handler.response.pose_stamped[0].pose);
    }
    else
    {
        ROS_ERROR("Failed to obtain forward kinematics from moveit server!");
        return 1;
    }
    
    return 0;
}
