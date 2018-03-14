#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int16.h"

#define Kp 50

int16_t steering;

void callback_right_line(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    float A = msg->data[0];
    float B = msg->data[1];
    float C = msg->data[2];
    //La imagen homografeada es de 640x700
    float angle_error = atan(B/A);
    float dist_error = (fabs(A*160 + B*120 +C)/sqrt(A*A + B*B) - 90)/120;;
    steering = (int16_t)(100 + Kp*dist_error);
    std::cout << "Found line: " << A << "\t" << B << "\t" << C << std::endl;
    std::cout << "Angle error= " << angle_error << std::endl;
}

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING LANE TRACKING BY MARCOSOFT..."  << std::endl;
    ros::init(argc, argv, "lane_tracking");
    ros::NodeHandle n;
    ros::Publisher  pub_steering   = n.advertise<std_msgs::Int16>("/manual_control/steering", 1);
    ros::Subscriber sub_lane_right = n.subscribe("/rightLine", 1, callback_right_line);
    ros::Rate loop(20);
    std_msgs::Int16 msg_steering;

    while(ros::ok())
    {
	msg_steering.data = steering;
	pub_steering.publish(msg_steering);
	ros::spinOnce();
	loop.sleep();
    }
    return 0;
}
