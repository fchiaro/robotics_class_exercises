#ifndef TF_LISTENER
#define TF_LISTENER

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Transform.h>

namespace tf_listener
{

// Convenience structure to represent in consistent way axis-angle rotation representation
struct AxisAngle{
    geometry_msgs::Vector3 r;
    tf2Scalar theta;
};

// Definition of custom message type consisting in a 3x3 matrix
typedef std::array<geometry_msgs::Vector3,3> Matrix3x3Msg;


// This class offers functionalities to listen for transfer frames and get several representations of them.
class TfListener
{
    private:
        tf2_ros::Buffer tf_buffer;
        tf2_ros::TransformListener tf_listener;
    
    public:
        TfListener();
        geometry_msgs::Transform getTf(const std::string& target_frame, const std::string& source_frame);
        geometry_msgs::Vector3 getTranslationalVector(geometry_msgs::Transform& reference_transform);
        tf_listener::Matrix3x3Msg getRotationMatrix(geometry_msgs::Transform& reference_transform);
        geometry_msgs::Vector3 getEulerAngles(geometry_msgs::Transform& reference_transform);
        AxisAngle getAxisAngle(geometry_msgs::Transform& reference_transform);
        void getFramesList(ros::V_string& buffer);
};

}

#endif