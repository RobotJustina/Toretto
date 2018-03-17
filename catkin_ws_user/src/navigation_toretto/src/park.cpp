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


#define MAX_DIST_TO_OBJ 0.45

#define PI 3.14159265
#define E 2.7182818284590
#define Steering_max = 0.57

using namespace std;

std_msgs::Int16 msg_steering;
std_msgs::Int16 msg_speed;
std_msgs::Int16 speed_obj;

int16_t steering_call=90;
//TODO use local variables.
bool object=false,objectR=false,objectL=false,objectF=false; //object detected
float l_obj=0, r_obj=0, f_obj=0;


float Kp_nav =50.0/100.0;
float Ka_nav =3.0;
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
        steering_call = (int16_t)(100 + Kp_nav*dist_error + Ka_nav * angle_error * 10);
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
        if((msg->data < 0.40) && (msg->data >0.01)) //while node starts a few messages will be received with data=0 must ignore
        {
                objectR=true;
        }
        else{
                objectR=false;
        }
        r_obj = msg->data;
        //cout << "object right "<<r_obj << "\n";

}


void Callback_objectL(const std_msgs::Float32::ConstPtr& msg)
{
        //cout << "object left true" << "\n";

        if(msg->data < 0.25) {
                objectL=true;
        }
        else{
                objectL=false;
        }
        l_obj = msg->data;
        //cout << "object left "<<l_obj << "\n";
}


void Callback_objectF(const std_msgs::Float32::ConstPtr& msg)
{
        //cout << "object left true" << "\n";

        if(msg->data < 0.25) {
                objectF=true;
        }
        else{
                objectF=false;
        }
        f_obj = msg->data;
        //cout << "object left "<<l_obj << "\n";
}




int main(int argc, char** argv)
{
        ros::init(argc, argv, "navigation");
        ros::NodeHandle n;
        int state = 0;
        //int i=0;

        ros::Subscriber position_subscriber = n.subscribe("/rightLine", 1, callback_right_line);

        ros::Publisher speeds_pub = n.advertise<std_msgs::Int16>("/manual_control/speed", 1);
        ros::Publisher steering_pub = n.advertise<std_msgs::Int16>("/manual_control/steering", 1);
        ros::Publisher light_fr_pub = n.advertise<std_msgs::String>("/manual_control/lights", 1);

        //ros::Subscriber object_subscriber = n.subscribe("/object_detection/speed", 1, Callback_object);
        ros::Subscriber objectF_subscriber = n.subscribe("/object_detection/front", 1, Callback_objectF);
        ros::Subscriber objectL_subscriber = n.subscribe("/object_detection/left", 1, Callback_objectL);
        ros::Subscriber objectR_subscriber = n.subscribe("/object_detection/right", 1, Callback_objectR);

        float kp_park;
        float vu_r,vu_l, vu_m;
        int max_steer_left, max_steer_right, mid_steer_right;
        n.param<float>("vu_r",vu_r,2.35);
        n.param<float>("vu_l",vu_l,2.2);
        n.param<float>("vu_m",vu_m,0.8);
        n.param<int>("max_steer_left",max_steer_left,10);
        n.param<int>("max_steer_right",max_steer_right,170);
        n.param<int>("mid_steer_right",mid_steer_right,130);
        n.param<float>("kp_park",kp_park,2.0);
        n.param<float>("Kp_nav",Kp_nav,50.0/100.0);
        n.param<float>("Ka_nav",Ka_nav,3.0);
        while (ros::ok())
        {
                ros::Rate loop_rate(10);
                float vu=0;
                float time_s=0, t_100=3.4;

                switch (state) {

                case 0:
                        std::cout << "[State: 0]  Config" << '\n';
                        state=1;
                        break;
                case 1:
                        std::cout << "[State: 1] Cruising" << '\n';
                        msg_speed.data=-200;
                        speeds_pub.publish(msg_speed);

                        // msg_steering.data = 90;
                        // steering_pub.publish(msg_steering);
                        msg_steering.data= steering_call;
                        steering_pub.publish(msg_steering);

                        if (objectR)
                        {

                                state=2;

                        }

                        break;
                case 2:
                        printf("[State: %d] Object @ right\n", state);
                        msg_steering.data= steering_call;
                        steering_pub.publish(msg_steering);

                        if (!objectR)
                        {
                                state=3;
                        }

                        break;
                case 3:
                        printf("[State: %d] Space detected right\n", state);
                        msg_steering.data= steering_call;
                        steering_pub.publish(msg_steering);

                        if (objectR)
                        {
                                msg_speed.data=0;
                                speeds_pub.publish(msg_speed);
                                sleep(1);
                                state=4;
                        }

                        break;


                case 4:
                        printf("[State: %d] Object @ right, Space ended\n", state);
                        vu=vu_r;
                        msg_steering.data=max_steer_right; //-50 orig
                        steering_pub.publish(msg_steering);
                        cout << "Pub Steering :" << msg_steering.data << "\n";
                        msg_speed.data=100;
                        speeds_pub.publish(msg_speed);
                        cout << "Pub speed :" << msg_speed.data << "\n";
                        time_s=(t_100*100/msg_speed.data);
                        time_s=time_s*vu;
                        if (time_s < 0) {
                                time_s=-1*time_s;
                        }
                        cout << "Time case 0: "<< time_s << "\n";
                        sleep(time_s);
                        msg_speed.data=00;
                        speeds_pub.publish(msg_speed);
                        sleep(1);
                        state = 5;
                        break;

                case 5:
                        printf("[State: %d] Changing reverse angle  \n", state);

                        vu=vu_l;
                        msg_steering.data=max_steer_left; //290 orig
                        steering_pub.publish(msg_steering);
                        cout << "Pub Steering :" << msg_steering.data << "\n";
                        msg_speed.data=100;
                        speeds_pub.publish(msg_speed);
                        cout << "Pub speed :" << msg_speed.data << "\n";
                        time_s=(t_100*100/msg_speed.data);
                        time_s=time_s*vu;
                        if (time_s < 0) {
                                time_s=-1*time_s;
                        }
                        cout << "Time case 1: "<< time_s << "\n";
                        sleep(time_s);
                        msg_speed.data=00;
                        speeds_pub.publish(msg_speed);
                        sleep(1);
                        state = 6;
                        break;

                case 6:
                        printf("[State: %d] Reverse stopped \n", state);
                        printf("\tComencing advance\n");
                        vu=vu_m;
                        msg_steering.data=mid_steer_right; //120 orig
                        steering_pub.publish(msg_steering);
                        cout << "Pub Steering :" << msg_steering.data << "\n";
                        msg_speed.data=-100;
                        speeds_pub.publish(msg_speed);
                        cout << "Pub speed :" << msg_speed.data << "\n";

                        time_s=(t_100*100/msg_speed.data);
                        time_s=time_s*vu;
                        if (time_s < 0) {
                                time_s=-1*time_s;
                        }
                        cout << "Time case 2: "<< time_s << "\n";
                        sleep(time_s);

                        state = 7;
                        break;

                case 7:
                        printf("[State: %d] Inside park space \n", state);
                        printf("Adjusting\n" );
                        //Ctrl

                        if(f_obj<=MAX_DIST_TO_OBJ)
                        {
                                msg_speed.data=0;
                                speeds_pub.publish(msg_speed);
                                state=8;
                        }
                        else
                        {
                                msg_speed.data=-100;
                                speeds_pub.publish(msg_speed);
                        }

                        msg_steering.data=kp_park*(r_obj-0.13)+90; //90 deg offset
                        steering_pub.publish(msg_steering);

                        break;
                case 8:
                        printf("[State: %d] End park \n", state);
                        msg_speed.data=0;
                        speeds_pub.publish(msg_speed);
                        sleep(3);
                        break;
                default:

                        cout << "Error undefined state"<< "\n";
                        msg_speed.data=0;
                        speeds_pub.publish(msg_speed);
                        msg_steering.data=90;
                        steering_pub.publish(msg_steering);
                        break;
                }

                ros::spinOnce();
                loop_rate.sleep();
        }
        // msg_speed.data=00;
        // speeds_pub.publish(msg_speed);

}
