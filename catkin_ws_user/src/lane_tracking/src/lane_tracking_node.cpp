#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"

void callback_right_line(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    float A = msg->data[0]/1000.0;
    float B = msg->data[1]/1000.0;
    float C = msg->data[2]/1000.0;
    float angle_error = atan(B/A);
    float dist_error = fabs(C)/sqrt(A*A + B*B);
    std::cout << "Found line: " << A << "\t" << B << "\t" << C << std::endl;
    std::cout << "Angle error= " << angle_error << std::endl;
}

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING LANE TRACKING BY MARCOSOFT..."  << std::endl;
    ros::init(argc, argv, "lane_tracking");
    ros::NodeHandle n;
    ros::Subscriber sub_lane_right = n.subscribe("/rightLine", 1, callback_right_line);
    ros::Rate loop(20);

    while(ros::ok())
    {
	ros::spinOnce();
	loop.sleep();
    }
    return 0;
}
