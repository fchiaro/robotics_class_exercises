#include <current_tf_listener.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/utils.h>


int main(int argc, char** argv){

    ros::init(argc, argv, "current_tf_listener");

    ros::NodeHandle nh;

    tf_listener::TfListener current_tf_listener;

    geometry_msgs::Transform tf;
    geometry_msgs::Vector3 trans_vect;
    tf_listener::Matrix3x3Msg rot_mat;
    geometry_msgs::Vector3 euler_angles;
    tf_listener::AxisAngle axis_angle;
    ros::V_string frames; // varying string

    ros::Rate rate(0.5); // repeat the loop every 2 seconds

    // Get a list of all available frames
    current_tf_listener.getFramesList(frames);

    while (ros::ok())
    {
        // Iterate over all frames
        for(int i=0; i<frames.size(); i++){

            // Skipping base_link ("extra link") and flange (makes no sense to compute transform of flage wrt to itself)
            if(frames[i] == "base_link" || frames[i] == "flange")
            {
                continue;
            }

            try
            {
                tf = current_tf_listener.getTf(frames[i], "flange");
                trans_vect = current_tf_listener.getTranslationalVector(tf);
                rot_mat = current_tf_listener.getRotationMatrix(tf);
                euler_angles = current_tf_listener.getEulerAngles(tf);
                axis_angle = current_tf_listener.getAxisAngle(tf);
            }catch (tf2::TransformException &exception) {
                ROS_WARN("%s", exception.what());
                ros::Duration(1.0).sleep();
                continue;
            }

            ROS_INFO_STREAM("flange WRT " << frames[i] << "\n" << "TF\n" << tf
                            << "\n" << "TRANSLATIONAL VECTOR\n" << trans_vect
                            << "\n" << "ROTATION MATRIX\n" << rot_mat[0].x << " " << rot_mat[0].y << " " << rot_mat[0].z << "\n"
                                                                << rot_mat[1].x << " " << rot_mat[1].y << " " << rot_mat[1].z << "\n"
                                                                << rot_mat[2].x << " " << rot_mat[2].y << " " << rot_mat[2].z << "\n"
                            << "\n" << "EULER ANGLES (RPY, must be executed YXZ) - radians\n" << euler_angles
                            << "\n" << "AXIS-ANGLE: theta (radians)\n" << axis_angle.theta << "\nAxis-angle: r\n" << axis_angle.r);
            
        }
    
        rate.sleep();
        if(frames.size() == 0)
        {
            current_tf_listener.getFramesList(frames);
        }
        
    }
    
    return 0;
}