#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int16.h"

float K_dist  = 0.5;
float K_angle = 16.0;
float K_brake = 1.0;
int max_speed  = 800;
int turn_speed = 400;
int dist_to_lane = 90;

int16_t steering = 100;;
int16_t speed = 0;

void callback_right_line(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    
    float A = msg->data[0];
    float B = msg->data[1];
    float C = msg->data[2];
    if (A==0 && B==0)
        return;
    //La imagen homografeada es de 640x700
    float angle_error = atan(B/A);
    float dist_error = (fabs(A*160 + B*120 +C)/sqrt(A*A + B*B) - dist_to_lane);
    steering = (int16_t)(100 + K_dist * dist_error + K_angle * angle_error);
    speed    = (int16_t)(-(max_speed - K_brake * fabs(angle_error) * (max_speed - turn_speed)));
    std::cout << "Found line: " << A << "\t" << B << "\t" << C << std::endl;
    std::cout << "Angle error= " << angle_error << std::endl;
}

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING LANE TRACKING BY MARCOSOFT..."  << std::endl;
    ros::init(argc, argv, "lane_tracking");
    ros::NodeHandle n;
    
    
    n.param<float>("K_dist", K_dist, 0.5);
    n.param<float>("K_angle", K_angle, 16.0);
    n.param<float>("K_brake", K_brake, 1.0);
    n.param<int>("max_speed", max_speed, 800);
    n.param<int>("turn_speed", turn_speed, 400);
    n.param<int>("dist_to_lane", dist_to_lane, 90);
    
    std::cout << "K_dist=" << K_dist << "\tK_angle=" << K_angle << "\tK_brake=" << K_brake << "\tmax_speed=";
    std::cout << max_speed << "\tturn_speed=" << turn_speed << "\tdist_to_lane=" << dist_to_lane << std::endl;
    ros::Publisher  pub_steering   = n.advertise<std_msgs::Int16>("/manual_control/steering", 1);
    ros::Publisher  pub_speed      = n.advertise<std_msgs::Int16>("/manual_control/speed", 1);
    ros::Subscriber sub_lane_right = n.subscribe("/rightLine", 1, callback_right_line);
    ros::Rate loop(20);
    std_msgs::Int16 msg_steering;
    std_msgs::Int16 msg_speed;
    

    while(ros::ok())
    {
	msg_steering.data = steering;
	msg_speed.data    = speed;
	pub_steering.publish(msg_steering);
	pub_speed.publish(msg_speed);
	ros::spinOnce();
	loop.sleep();
    }
    return 0;
}
