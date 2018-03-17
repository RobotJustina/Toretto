#include <ros/ros.h>
#include <iostream>
#include <std_msgs/String.h>
#include <math.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
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

int16_t steering_call=90;

bool object=false,objectR=false,objectL=false; //object detected
bool cross=false;
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
        steering_call = (int16_t)(100 + Kp*dist_error + Ka * angle_error * 10);
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


        if(msg->data < 0.40) {
                objectR=true;
        }
        else{
                objectR=false;
        }
        r_obj = msg->data;
        //cout << "object right "<<r_obj << "\n";

}

void Callback_cross(const std_msgs::Bool::ConstPtr& msg)
{
        cout << "Cross" << "\n";

        if(msg->data == true) {
                cross=true;
        }
        else{
                cross=false;
        }

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


int main(int argc, char** argv)
{
        ros::init(argc, argv, "navigation");
        ros::NodeHandle n;
        int state = 0;
        int j=0;

        ros::Subscriber position_subscriber = n.subscribe("/rightLine", 1, callback_right_line);

        ros::Publisher speeds_pub = n.advertise<std_msgs::Int16>("/manual_control/speed", 1);
        ros::Publisher steering_pub = n.advertise<std_msgs::Int16>("/manual_control/steering", 1);
        ros::Publisher light_fr_pub = n.advertise<std_msgs::String>("/manual_control/lights", 1);

        ros::Subscriber object_subscriber = n.subscribe("/object_detection/speed", 1, Callback_object);
        ros::Subscriber objectL_subscriber = n.subscribe("/object_detection/left", 1, Callback_objectL);
        ros::Subscriber objectR_subscriber = n.subscribe("/object_detection/right", 1, Callback_objectR);

        float vu_r,vu_l, vu_m;
        int max_steer_left, max_steer_right, mid_steer_right;
        n.param<float>("vu_r",vu_r,2.35);
        n.param<float>("vu_l",vu_l,2.2);
        n.param<float>("vu_m",vu_m,0.8);
        n.param<int>("max_steer_left",max_steer_left,10);
        n.param<int>("max_steer_right",max_steer_right,170);
        n.param<int>("mid_steer_right",mid_steer_right,130);
        while (ros::ok())
        {
                ros::Rate loop_rate(10);
                float vu=0;
                float time_s=0, t_100=3.4;

                switch (state) {
                case 0:
                        std::cout << "[State: 0]  Config" << '\n';
                        msg_speed.data=-350;
                        speeds_pub.publish(msg_speed);
                        state=1;
                        break;
                case 1:
                        std::cout << "[State: 1] Cruising" << '\n';
                        if (object)
                        {
                                printf("[State: %d] Object @ front\n", state);
                                speeds_pub.publish(speed_obj);
                                //Match obj speed and if static overtake
                                cout << "j: " << j << "\n";
                                cout << "Speed Object: " << speed_obj.data <<"\n";
                                if(speed_obj.data == 0) {
                                        j++;
                                }
                                else{
                                        j=0;
                                }
                                if (j > 40) {
                                        cout << "Overtaking \n";

                                        state= 2;
                                        j=0;
                                }

                        }
                        else{
                                msg_speed.data=-350;
                                speeds_pub.publish(msg_speed);
                                cout << "Speed: " << msg_speed.data <<"\n";
                                //Use line tracking to steer
                                msg_steering.data= steering_call;
                                steering_pub.publish(msg_steering);

                                if(cross)
                                {
                                        //HEre goes code to handle intersection
                                }

                        }
                        break;

                case 2: //turn left to overtake obstacle
                        printf("[State: %d] Turning left\n", state);
                        msg_steering.data=max_steer_left;
                        steering_pub.publish(msg_steering);
                        msg_speed.data=-100;
                        vu=1.9;
                        time_s=(t_100*100/msg_speed.data);
                        if (time_s < 0) {
                                time_s=-time_s;
                        }
                        time_s=time_s*vu*1000000;
                        cout << "Time evasion 1: "<< time_s << "\n";
                        usleep(time_s);

                        speeds_pub.publish(msg_speed);
                        sleep(1);
                        state=3;
                        break;

                case 3: //turn right to enter left lane
                        printf("[State: %d] Turning right\n", state);
                        vu=2;
                        msg_steering.data=max_steer_right; // orig 20
                        steering_pub.publish(msg_steering);
                        msg_speed.data=-100;
                        time_s=(t_100*100/msg_speed.data);
                        if (time_s < 0) {
                                time_s=time_s* -1;
                        }
                        time_s=time_s*vu*1000000;
                        cout << "Time evasion 2: "<< time_s << "\n";
                        usleep(time_s);

                        speeds_pub.publish(msg_speed);
                        sleep(1);
                        state=4;


                        break;
                case 4: //we are now in the left lane
                        printf("[State: %d] Onleft line, following line\n", state);
                        msg_steering.data= steering_call;
                        steering_pub.publish(msg_steering);
                        if (!objectR)
                        {
                                state=6;
                        }
                        break;
                case 6: //objcect celar return to right lane
                        printf("[State: %d]Turning to right lane\n", state);
                        vu=1.6;
                        msg_steering.data=max_steer_right;
                        steering_pub.publish(msg_steering);
                        msg_speed.data=-100;
                        speeds_pub.publish(msg_speed);

                        time_s=(t_100*100/msg_speed.data);
                        if (time_s < 0) {
                                time_s=time_s* -1;
                        }
                        time_s=time_s*vu*1000000;
                        cout << "Time evasion 1: "<< time_s << "\n";
                        ros::spinOnce();
                        usleep(time_s);
                        msg_speed.data=-100;
                        speeds_pub.publish(msg_speed);
                        sleep(1);
                        state=7;


                        break;
                case 7:
                        printf("[State: %d]Turning left to match right lane\n", state);
                        vu=0.7;
                        msg_steering.data=50;
                        steering_pub.publish(msg_steering);
                        msg_speed.data=-100;
                        speeds_pub.publish(msg_speed);

                        time_s=(t_100*100/msg_speed.data);

                        if (time_s < 0) {
                                time_s=time_s* -1;
                        }
                        time_s=time_s*vu*1000000;

                        usleep(time_s);
                        msg_speed.data=-100;
                        speeds_pub.publish(msg_speed);
                        sleep(1);
                        state=8;
                        break;
                case 8:
                        printf("[State: %d] On right line, following line again\n", state);
                        msg_steering.data= steering_call;
                        steering_pub.publish(msg_steering);
                        state=0;
                        break;
                default:

                        cout << "Error undefined state"<< "\n";
                        msg_speed.data=0;
                        speeds_pub.publish(msg_speed);
                        break;
                }

                ros::spinOnce();
                loop_rate.sleep();
        }
        return 0;
}
