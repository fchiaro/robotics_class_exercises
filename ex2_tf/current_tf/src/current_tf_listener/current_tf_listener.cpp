#include <current_tf_listener.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


/**
 * Implementation of the TfListener class defined in the corresponding header file.
 */


tf_listener::TfListener::TfListener():
    tf_listener(tf_buffer)
{
};

/**
 * Possible exceptions tf2::TransformException 
 */  
geometry_msgs::Transform tf_listener::TfListener::getTf(const std::string& target_frame, const std::string& source_frame)
{
    geometry_msgs::TransformStamped transform_stamped;
    transform_stamped = tf_buffer.lookupTransform(target_frame, source_frame, ros::Time(0));

    return transform_stamped.transform;
}

/**
 * Produces a std::vector of all available frame ids.
 */
void tf_listener::TfListener::getFramesList(ros::V_string& buffer)
{
    tf_buffer._getFrameStrings(buffer);
}

geometry_msgs::Vector3 tf_listener::TfListener::getTranslationalVector(geometry_msgs::Transform& reference_transform)
{
    // Return only the transformation, without timestamp
    return reference_transform.translation;
}


tf_listener::Matrix3x3Msg tf_listener::TfListener::getRotationMatrix(geometry_msgs::Transform& reference_transform)
{
    /* 
    * Convert the transform message to another transform type, in order to use a function which allows to directly
    * retrieve the rotation matrix.
    */
    tf2::Transform transf;
    tf2::fromMsg(reference_transform, transf);

    // Get the rotation matrix
    tf2::Matrix3x3 mat(transf.getBasis());

    // Put the matrix into a message
    tf_listener::Matrix3x3Msg msg_mat;
    for (int i=0; i<3; i++)
    {
        msg_mat[i] = tf2::toMsg(mat.getRow(i));
    }

    return msg_mat;
}


/**
 * Returns YXZ Euler angles.
 * 
 * yaw around Z axis
 * pitch around Y axis
 * roll around X axis
 */ 
geometry_msgs::Vector3 tf_listener::TfListener::getEulerAngles(geometry_msgs::Transform& reference_transform)
{
    // Change type of transform, for reasons similar to the previous ones
    tf2::Transform transf;
    tf2::fromMsg(reference_transform, transf);

    // Get Euler angles
    tf2Scalar yaw, pitch, roll;
    transf.getBasis().getEulerYPR(yaw, pitch, roll);

    // Put them into a vector
    tf2::Vector3 out_vect;
    out_vect.setX(roll);
    out_vect.setY(pitch);
    out_vect.setZ(yaw);

    // Put the vector into a message
    geometry_msgs::Vector3 out_vect_msg;
    out_vect_msg = tf2::toMsg(out_vect);

    return out_vect_msg;
}

tf_listener::AxisAngle tf_listener::TfListener::getAxisAngle(geometry_msgs::Transform& reference_transform)
{
    // Transform type conversion
    tf2::Transform transf;
    tf2::fromMsg(reference_transform, transf);

    // Get the rotation quaternion
    tf2::Quaternion quat;
    quat = transf.getRotation();

    // Extract information and put it into the custom struct
    tf_listener::AxisAngle ret_val;
    ret_val.theta = quat.getAngle();
    ret_val.r = tf2::toMsg(quat.getAxis());

    return ret_val;
}