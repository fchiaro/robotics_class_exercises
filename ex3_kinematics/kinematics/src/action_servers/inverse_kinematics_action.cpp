#include <inverse_kinematics_action.h>
#include <moveit/robot_state/conversions.h>
#include <angles/angles.h>

kinematics::InverseKinematicsAction::InverseKinematicsAction():
    action_server_(nh_, "ik_server", boost::bind(&kinematics::InverseKinematicsAction::compute_ik_, this, _1), false)
    // robot_model_loader_("robot_description"),
    // kinematic_model_(robot_model_loader_.getModel())
{
    action_server_.start();
};

void kinematics::InverseKinematicsAction::compute_ik_(const kinematics_msgs::InverseKinematicsGoalConstPtr &goal)
{
    ROS_INFO("[IK Action Server] Received IK request - Loading required data...");

    // Get planning group name
    if(!ros::param::param<std::string>("/joint_group", joint_group_, "all_joints"))
    {
        ROS_WARN("[IK Action Server] Could not load user-defined joint group, going with the default one...");
    }

    // Build robot description objects (fresh data on each call)
    robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>("robot_description");
    kinematic_model_ = robot_model_loader_->getModel();

    ROS_INFO("[IK Action Server] Data loaded, computing IK solutions...");

    // Go down to KinematicsBase in order to be able to set seed, so that it's possible to look for multiple solutions
    const kinematics::KinematicsBaseConstPtr ik_solver = kinematic_model_->getJointModelGroup(joint_group_)->getSolverInstance();

    kinematics_msgs::InverseKinematicsResult result;
    int n_attempts = 0;

    
    // Timeout in SRDF is 0.005 seconds, so 1000 attempts take at most 5 seconds
    while(n_attempts < 10000 && ros::ok())
    {
        std::vector<double> initial_joints_config = generateRandomJointsConfig_();
        std::vector<double> solution;
        moveit_msgs::MoveItErrorCodes res;
        
        ik_solver->getPositionIK(goal->target_pose, initial_joints_config, solution, res);

        if(res.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
        {
            
            // Need to normalize to be able to understand that joint positions which differ by 2*pi are the same position
            normalizeJointPositions_(solution);  

            if(isSolutionNew_(solution))
            {
                visited_solutions_.push_back(solution);

                // Create temporary robot state, to avoid overwriting real state
                moveit::core::RobotState temp_robot_state(kinematic_model_);
                temp_robot_state.setVariablePositions(solution);

                kinematics_msgs::InverseKinematicsFeedback feedback;

                moveit::core::robotStateToRobotStateMsg(temp_robot_state, feedback.ik_solution);

                action_server_.publishFeedback(feedback);

                result.ik_solutions.push_back(feedback.ik_solution);

            }
        }

        n_attempts++;
    }

    if(visited_solutions_.size() == 0){
        ROS_INFO("[IK Action Server] Could not find any IK solution for the requested pose!");
        action_server_.setAborted(result);
    }
    else
    {
        ROS_INFO_STREAM("[IK Action Server] Found " << visited_solutions_.size() << " IK solutions for the requested pose!");   
        action_server_.setSucceeded(result);
    }

    visited_solutions_.resize(0);

}

std::vector<double> kinematics::InverseKinematicsAction::generateRandomJointsConfig_()
{
    std::vector<double> joints_config;
    
    std::vector<std::string> joint_names = kinematic_model_->getVariableNames();

    for(int i=0; i < joint_names.size(); i++)
    {
        double lower_limit = kinematic_model_->getURDF()->getJoint(joint_names[i])->limits->lower;
        double upper_limit = kinematic_model_->getURDF()->getJoint(joint_names[i])->limits->upper;

        double interval = upper_limit-lower_limit;
        joints_config.push_back((double)std::rand()/RAND_MAX * interval + lower_limit);
    }

    return joints_config;
}

void kinematics::InverseKinematicsAction::normalizeJointPositions_(std::vector<double> &solution){
    for(int i=0; i < solution.size(); i++)
    {
        if (kinematic_model_->getJointModelGroup(joint_group_)->getActiveJointModels()[i]->getType() == robot_model::JointModel::REVOLUTE)
        {
            solution[i] = angles::normalize_angle(solution[i]);
        }
    }
}

bool kinematics::InverseKinematicsAction::isSolutionNew_(std::vector<double> &solution)
{
    for(int i=0; i < visited_solutions_.size(); i++)
    {
        bool are_solutions_equal = true;

        for(int j=0; j < visited_solutions_[i].size() && are_solutions_equal; j++)
        {
            double diff;

            // Watch out for solver tolerance and different angles representing the same angular distance
            if(kinematic_model_->getJointModelGroup(joint_group_)->getActiveJointModels()[j]->getType() == robot_model::JointModel::REVOLUTE)
            {
                diff = angles::shortest_angular_distance(visited_solutions_[i][j], solution[j]);
            }
            else
            {
                diff = visited_solutions_[i][j] - solution[j];
            }

            if(std::fabs(diff) > 1e-3)
                are_solutions_equal = false;
        }

        if(are_solutions_equal)
            return false;
    }

    return true;
}