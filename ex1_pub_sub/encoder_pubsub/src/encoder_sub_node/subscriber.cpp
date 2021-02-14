#include "ros/ros.h"
#include "encoder_pubsub_msgs/Reading.h"

void print_encoder_reading(const encoder_pubsub_msgs::Reading& msg){
    ROS_INFO_STREAM("Joints positions:");
    for(int i = 0; i < msg.joints_positions.size(); i++){
        ROS_INFO_STREAM("Joint " << (i+1) << ": " << msg.joints_positions[i]);
    }
}

int main(int argc, char** argv){

    ros::init(argc, argv, "encoder_readings_subscriber");
    ros::NodeHandle nh;
    ros::Subscriber encoder_sub = nh.subscribe("/encoder_readings", 1, print_encoder_reading);
    ros::spin();
    return 0;
}