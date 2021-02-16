# Kinematics

- Implements a service server that computes the direct kinematics of a robot and a service client that uses this service and prints the solution to stdout, together with the results obtained on the same input by the service /compute_fk of the move_group node.
- Implements an action server that computes all the inverse kinematic solutions of a robot (one by one) and an action client that uses this action and prints the solutions to stdout (one by one, as their are received). The action server does not send the same solution twice and stops when all solutions have been found. At that time, they are returned all together. The client also publishes the IK solution for the visualization in RViz.

## Structure

The action server is implemented through a node and a class, as shown in the actionlib tutorials. The other elements are simple ROS nodes.

The `default.yaml` file can be used to load custom configuration information for the forward kinematics node.

## Usage

Launch `demo.launch` from `fanuc_moveit_config` package so that robot description information is loaded on the parameter server, and then run either the forward kinematics server and client (`rosrun kinematics forward_kinematics_server_node`, `rosrun kinematics forward_kinematics_node` respectively) or the inverse kinematics action server (`rosrun kinematics inverse_kinematics_server_node`) and client (`rosrun kinematics inverse_kinematics_node`).
