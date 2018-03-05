#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include <opencv2/opencv.hpp>
#include "tf/transform_broadcaster.h"

float speed = 0;
float steering = 0;

void callback_speed(const std_msgs::Int16::ConstPtr& msg)
{
    speed = -0.0005626384*msg->data;
    //This value was obtained by making a linear regression from experimental data
    std::cout << "Received speed [m/s]: " << speed << std::endl;
}

void callback_steering(const std_msgs::Int16::ConstPtr& msg)
{
    steering = (msg->data - 100)/80*26*M_PI/180;
    std::cout << "Received steering: " << steering << std::endl;
}

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING ODOMETRY CALCULATOR NODE..." << std::endl;
    ros::init(argc, argv, "odometry_calculator");
    ros::NodeHandle n;
    ros::Subscriber sub_speed    = n.subscribe("/manual_control/speed", 1, callback_speed);
    ros::Subscriber sub_steering = n.subscribe("/manual_control/steering", 1, callback_steering);
    ros::Rate loop(30);

    tf::TransformBroadcaster br;
    tf::Transform t;

    //Variables for the differential equation model;
    float car_x;
    float car_y;
    float car_t;
    
    
    while(ros::ok())
    { 
	ros::spinOnce();
	loop.sleep();
    }
    return 0;
}
