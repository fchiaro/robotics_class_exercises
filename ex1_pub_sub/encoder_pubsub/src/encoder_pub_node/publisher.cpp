#include "ros/ros.h"
#include "encoder_pubsub_msgs/Reading.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "encoder_readings_publisher");
    ros::NodeHandle nh;
    ros::Publisher encoder_publisher =
        nh.advertise<encoder_pubsub_msgs::Reading>("/encoder_readings", 1);
    ros::Rate rate(0.2);
    encoder_pubsub_msgs::Reading message;
    
    message.joints_positions = {0.5,0.6,0.7,0.8,0.9,1.0};

    while (ros::ok())
    {    
        encoder_publisher.publish(message);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}