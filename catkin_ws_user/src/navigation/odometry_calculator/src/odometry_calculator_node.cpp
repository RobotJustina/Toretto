#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include <opencv2/opencv.hpp>
#include "tf/transform_broadcaster.h"

float speed = 0;
float steering = 0;
float l_r = 0.13; //Physical parameters. Check paper Kong, Pfeiffer, Schildbach, Borelli.
float l_f = 0.13; //Kinematic and dynamic vehicle models for autonomous driving control design

void callback_speed(const std_msgs::Int16::ConstPtr& msg)
{
    speed = -0.0005626384*msg->data;
    //This value was obtained by making a linear regression from experimental data
  //  std::cout << "Received speed [m/s]: " << speed << std::endl;
}

void callback_steering(const std_msgs::Int16::ConstPtr& msg)
{
    steering = -(msg->data - 100.0)/80.0*26.0*M_PI/180.0;
  //  std::cout << "Received steering: " << steering << std::endl;
}

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING ODOMETRY CALCULATOR NODE..." << std::endl;
    ros::init(argc, argv, "odometry_calculator");
    ros::NodeHandle n;
    ros::Subscriber sub_speed    = n.subscribe("/manual_control/speed", 1, callback_speed);
    ros::Subscriber sub_steering = n.subscribe("/manual_control/steering", 1, callback_steering);

    tf::TransformBroadcaster br;
    tf::Transform t;

    //Variables for the differential equation model;

    float car_x = 0;
    float car_y = 0;
    float car_t = 0;
    int simul_freq = 30; //Inverse of the simul step

    ros::Rate loop(simul_freq);
    while(ros::ok())
    {
	float beta = atan(l_r/(l_r + l_f)*tan(steering));
	float x_dot = speed * cos(car_t + beta);
	float y_dot = speed * sin(car_t + beta);
	float t_dot = speed / l_r * sin(beta);

	car_x += x_dot * 1.0/simul_freq;
	car_y += y_dot * 1.0/simul_freq;
	car_t += t_dot * 1.0/simul_freq;

	t.setOrigin(tf::Vector3(car_x, car_y, 0));
	tf::Quaternion q;
	q.setRPY(0, 0, car_t);
	t.setRotation(q);
	br.sendTransform(tf::StampedTransform(t, ros::Time::now(), "odom", "base_link"));
	ros::spinOnce();
	loop.sleep();
    }
    return 0;
}
