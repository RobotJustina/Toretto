#include <ros/ros.h>
#include <iostream>
#include <std_msgs/String.h>
#include <math.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <math.h>
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "nav_msgs/Path.h"

#define Kp 50.0/100.0
#define Ka 3.0

#define PI 3.14159265
#define E 2.7182818284590
#define Steering_max = 0.57

using namespace std;

std_msgs::Int16 msg_steering;
std_msgs::Int16 msg_speed;
std_msgs::Int16 speed_obj;

int16_t steering;

bool object=false,objectR=false,objectL=false; //object detected
float l_obj=0, r_obj=0;

void callback_right_line(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
        //From MARCOSOFT
        //Tracks line via angle and pixel position.

        float A = msg->data[0];
        float B = msg->data[1];
        float C = msg->data[2];
        //La imagen homografeada es de 640x700
        float angle_error = atan(B/A);
        float dist_error = (fabs(A*160 + B*120 +C)/sqrt(A*A + B*B) - 90);
        steering = (int16_t)(100 + Kp*dist_error + Ka * angle_error * 10);
        std::cout << "Found line: " << A << "\t" << B << "\t" << C << std::endl;
        std::cout << "Angle error= " << angle_error << std::endl;
}


void Callback_object(const std_msgs::Int16::ConstPtr& msg)
{
        cout << "object true" << "\n";
        object=true;
        speed_obj.data = msg->data;

}


void Callback_objectR(const std_msgs::Float32::ConstPtr& msg)
{
        //cout << "object right true" << "\n";

        if(msg->data < 0.50) {
                objectR=true;
        }
        r_obj = msg->data;

}


void Callback_objectL(const std_msgs::Float32::ConstPtr& msg)
{
        //cout << "object left true" << "\n";

        if(msg->data < 0.25) {
                objectL=true;
        }
        l_obj = msg->data;

}



int main(int argc, char** argv)
{
        ros::init(argc, argv, "navigation");
        ros::NodeHandle n;
        int nextstate = 0;
        //int i=0;

        //ros::Subscriber position_subscriber = n.subscribe("/path", 1, Callback_path);

        ros::Publisher speeds_pub = n.advertise<std_msgs::Int16>("/manual_control/speed", 1);
        ros::Publisher steering_pub = n.advertise<std_msgs::Int16>("/manual_control/steering", 1);
        ros::Publisher light_fr_pub = n.advertise<std_msgs::String>("/manual_control/lights", 1);

        ros::Subscriber object_subscriber = n.subscribe("/object_detection/speed", 1, Callback_object);
        ros::Subscriber objectL_subscriber = n.subscribe("/object_detection/left", 1, Callback_objectL);
        ros::Subscriber objectR_subscriber = n.subscribe("/object_detection/right", 1, Callback_objectR);
        ros::Rate loop_rate(10);
        while (ros::ok())
        {
                switch (nextstate) {

                case 0:
                        msg_speed.data=-200;
                        msg_steering.data = 90;
                        break;

                default:
                        cout << "Error undefined state"<< "\n";
                        break;
                }
                speeds_pub.publish(msg_speed);
                steering_pub.publish(msg_steering);
                loop_rate.sleep();
        }
        msg_speed.data=00;
        speeds_pub.publish(msg_speed);

}
