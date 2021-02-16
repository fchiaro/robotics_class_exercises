#ifndef KINEMATICS_ACTION
#define KINEMATICS_ACTION

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <kinematics_msgs/InverseKinematicsAction.h>

namespace kinematics
{

class InverseKinematicsAction
{

    private:
        robot_model_loader::RobotModelLoader robot_model_loader_;
        robot_model::RobotModelPtr kinematic_model_;
        robot_state::RobotStatePtr kinematic_state_;
        std::string joint_group_;
        std::vector<std::vector<double>> visited_solutions_;

        void compute_ik_(const kinematics_msgs::InverseKinematicsGoalConstPtr &goal);
        bool isSolutionNew_(std::vector<double> &solution);
        std::vector<double> generateRandomJointsConfig_();
        void normalizeJointPositions_(std::vector<double> &solution);

        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<kinematics_msgs::InverseKinematicsAction> action_server_;

    public:
        InverseKinematicsAction();
};

}

#endif