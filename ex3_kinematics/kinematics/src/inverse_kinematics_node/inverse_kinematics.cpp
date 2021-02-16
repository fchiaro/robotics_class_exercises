#include <ros/ros.h>
#include <kinematics_msgs/InverseKinematicsAction.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <sensor_msgs/JointState.h>


void goalDeliveredHandler()
{
    ROS_INFO("[IK Action Client] Goal delivered to IK server!");
}

std::string stringifyIKSolution(moveit_msgs::RobotState solution)
{
    int n_joints = solution.joint_state.position.size();

    std::ostringstream message;

    message << "[";

    for(int i=0; i < n_joints; i++)
    {
        message << solution.joint_state.position[i];

        if(i != n_joints - 1)
            message << ", ";
    }

    message << "]";

    return message.str();
}

void feedbackHandler(const kinematics_msgs::InverseKinematicsFeedbackConstPtr & feedback)
{
    ROS_INFO_STREAM("[IK Action Client] Received IK solution:\n" << stringifyIKSolution(feedback->ik_solution));
}

void goalCompletionHandler(const actionlib::SimpleClientGoalState & state, const kinematics_msgs::InverseKinematicsResultConstPtr & result)
{
    std::ostringstream message;
    float SOLUTIONS_PUBLISHING_INTERVAL = 2.0; // seconds

    if(state == actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED)
    {
        int num_of_solutions = result->ik_solutions.size();

        message << "Found " << num_of_solutions << " IK solutions:\n";

        for(int i=0; i < num_of_solutions; i++)
        {
            message << stringifyIKSolution(result->ik_solutions[i]) << "\n";
            
        }

        ROS_INFO_STREAM("[IK Action Client] " << message.str());

        ros::NodeHandle nh;

        // Substitute the joint state publisher in order to visualize solution in RViz
        ros::Publisher joint_state_publisher = nh.advertise<sensor_msgs::JointState>("joint_states", 1);

        // Load the robot model
        robot_model_loader::RobotModelLoader robot_model_loader("robot_description");

        // Get the robot kinematic model
        robot_model::RobotModelConstPtr kinematic_model = robot_model_loader.getModel();

        // Get the planning group name from the parameter server
        std::string planning_group_name;
        if(!ros::param::param<std::string>("/joint_group", planning_group_name, "all_joints"))
        {
            ROS_WARN("[IK Action Client] Could not load user-defined joint group, going with the default one...");
        }

        // Get the planning group
        const robot_state::JointModelGroup * joint_model_group = kinematic_model->getJointModelGroup(planning_group_name);

        // Robot model and planning group are needed for these variables
        sensor_msgs::JointState joint_state_msg;
        joint_state_msg.name = joint_model_group->getVariableNames();

        ROS_INFO("[IK Action Client] Publishing solutions...");

        ros::Duration sleep_time(SOLUTIONS_PUBLISHING_INTERVAL);

        for(int i=0; i < num_of_solutions; i++)
        {
            sleep_time.sleep();

            joint_state_msg.position = result->ik_solutions[i].joint_state.position;
            joint_state_msg.header.stamp = ros::Time::now();

            joint_state_publisher.publish(joint_state_msg);
        }       

        ROS_INFO("[IK Action Client] Solution published!");        
    }
    else
    {
        ROS_INFO("[IK Action Client] Goal was aborted by server!");
    }    
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "inverse_kinematics_client");
    
    actionlib::SimpleActionClient<kinematics_msgs::InverseKinematicsAction> action_client("ik_server", true);

    ROS_INFO("[IK Action Client] Waiting for server...");
    action_client.waitForServer();

    float REQUEST_TIMEOUT = 30.0;
    
    kinematics_msgs::InverseKinematicsGoal goal;

    // The following position and orientation should correspond to the robot with the upper arm directed toward observer's left
    goal.target_pose.position.x = 0.008;
    goal.target_pose.position.y = -1.0;
    goal.target_pose.position.z = 1.6;

    tf2::Quaternion target_orientation(0.47, -0.449, 0.525, 0.546);
    // target_orientation.setRPY(0.0, 0.0, 0.0);

    goal.target_pose.orientation = tf2::toMsg(target_orientation);
    
    ROS_INFO("[IK Action Client] Server found, sending request...");
    action_client.sendGoal(goal, &goalCompletionHandler, &goalDeliveredHandler, &feedbackHandler);

    if(!action_client.waitForResult(ros::Duration(REQUEST_TIMEOUT))){
        ROS_ERROR("[IK Action Client] Could not find IK solutions in time!");    
    }
        
    ros::spin();

    return 0;
}