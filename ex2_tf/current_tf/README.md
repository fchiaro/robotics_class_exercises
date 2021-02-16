# CURRENT TF

This package provides a node that listens for TFs transmitted and computes the TF of the end-effector in all the reference frames of all joints of the robot of which TFs are being transmitted. From the TF, computes the translational vector, the rotation matrix, the Euler angles and the axis-angle representation.

## Structure

Functionalities for getting a transformation between frames and transforming it between several represetations are implemented in a class, that is instantiated and used by the main node (`current_tf_listener_node`).

## Usage

A simple way to see this software in action is by launching `roslaunch fanuc_moveit_config demo.launch` and then executing the listener node (`rosrun current_tf current_tf_listener_node`).
