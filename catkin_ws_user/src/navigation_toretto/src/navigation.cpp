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
std_msgs::Int16 stop_flag;


bool objectF=false,objectR=false,objectL=false; //object detected
bool cross=false;
float l_obj=0, r_obj=0, f_obj=0;
bool shutdown=false;

#define filter_order 10

float K_dist  = 0.5;
float K_angle = 16.0;
float K_brake = 1.0;
int max_speed  = 800;
int turn_speed = 400;
int dist_to_lane = 100;

int16_t steering = 100;;
int16_t speed = 0;

std::vector<float> filtered_theta;

void callback_right_line(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
        float A = msg->data[0];
        float B = msg->data[1];
        float C = msg->data[2];
        if (A==0 && B==0)
                return;
        //La imagen homografeada es de 640x700
        float angle_error = atan(B/A);
        filtered_theta.erase(filtered_theta.begin());
        filtered_theta.push_back(angle_error);
        float avg=0;
        for (int i = 0; i < filter_order; i++) {
                avg+=filtered_theta[i];
        }
        avg/=filter_order; //same 10 as in main
        angle_error=avg;
        float dist_error = (fabs(A*160 + B*190 +C)/sqrt(A*A + B*B) - dist_to_lane);
        steering = (int16_t)(100 + K_dist * dist_error + K_angle * angle_error);
        speed    = (int16_t)(-(max_speed - K_brake * fabs(angle_error) * (max_speed - turn_speed)));
        // std::cout << "Found line: " << A << "\t" << B << "\t" << C << std::endl;
        // std::cout << "Angle error= " << angle_error << std::endl;

}

void Callback_object(const std_msgs::Int16::ConstPtr& msg)
{

        speed_obj.data = msg->data;
        cout << "object true " << msg->data<<"\n";

}


void Callback_objectR(const std_msgs::Float32::ConstPtr& msg)
{
        if((msg->data < 0.40) && (msg->data >0.01)) {
                objectR=true;
        }
        else{
                objectR=false;
        }
        r_obj = msg->data;
        cout << "object right "<<r_obj << "\n";

}

void Callback_cross(const std_msgs::Bool::ConstPtr& msg)
{
        //cout << "Cross" << "\n";

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

void Callback_objectF(const std_msgs::Float32::ConstPtr& msg)
{
        //cout << "object left true" << "\n";

        if((msg->data < 1.00) && (msg->data >0.01)) {
                objectF=true;
        }
        else{
                objectF=false;
        }
        f_obj = msg->data;
        //cout << "object left "<<l_obj << "\n";
}

void Callback_stop(const std_msgs::Int16::ConstPtr& msg)
{
        if(msg->data==1)
        {
                printf("!!!!!!Requesting shutdown!!!!!\n");
                shutdown=true;
        }
        else if(msg->data==0)
        {
                printf("!!!!!!Disabling shutdown!!!!!\n");
                shutdown=false;
        }

}


int main(int argc, char** argv)
{
        ros::init(argc, argv, "navigation");
        ros::NodeHandle n;


        ros::Subscriber position_subscriber = n.subscribe("/rightLine", 1, callback_right_line);
        //ros::Subscriber cross_subscriber = n.subscribe("/cross", 1, Callback_cross);

        ros::Publisher speeds_pub = n.advertise<std_msgs::Int16>("/manual_control/speed", 1);
        ros::Publisher steering_pub = n.advertise<std_msgs::Int16>("/manual_control/steering", 1);
        ros::Publisher light_fr_pub = n.advertise<std_msgs::String>("/manual_control/lights", 1);
        ros::Publisher stop_line_det_pub = n.advertise<std_msgs::Int16>("/manual_control/stop", 1);

        //ros::Subscriber stop_subscriber = n.subscribe("/manual_control/stop", 1, Callback_stop);
        ros::Subscriber object_subscriber = n.subscribe("/object_detection/speed", 1, Callback_object);
        ros::Subscriber objectL_subscriber = n.subscribe("/object_detection/left", 1, Callback_objectL);
        ros::Subscriber objectR_subscriber = n.subscribe("/object_detection/right", 1, Callback_objectR);
        ros::Subscriber objectF_subscriber = n.subscribe("/object_detection/front", 1, Callback_objectF);

        int max_steer_left, max_steer_right, mid_steer_right;

        n.param<float>("K_dist", K_dist, 0.5);
        n.param<float>("K_angle", K_angle, 16.0);
        n.param<float>("K_brake", K_brake, 1.0);
        n.param<int>("max_speed", max_speed, 800);
        n.param<int>("turn_speed", turn_speed, 400);
        n.param<int>("dist_to_lane", dist_to_lane, 90);

        n.param<int>("max_steer_left",max_steer_left,10);
        n.param<int>("max_steer_right",max_steer_right,170);
        n.param<int>("max_steer_right",max_steer_right,170);

        //vu means vuelta
        float vu_l1,vu_r1,vu_r2, vu_l2;
        n.param<float>("vu_l1",vu_l1,1.9);
        n.param<float>("vu_r1",vu_r1,2);
        n.param<float>("vu_r2",vu_r2,1.6);
        n.param<float>("vu_l2",vu_l2,0.7);

        int state = 0;
        int j=0;

        for (int i=0; i<filter_order; i++)
        {
                filtered_theta.push_back(0.0);
        }

        while (ros::ok())
        {
                ros::Rate loop_rate(10);
                float vu=0;
                float time_s=0, t_100=3.4;

                // if (shutdown)
                // {
                //         printf("Shutdown\n" );
                //         msg_speed.data=0;
                //         speeds_pub.publish(msg_speed);
                //         continue;
                // }

                switch (state) {
                case 0:
                        std::cout << "[State: 0]  Config" << '\n';
                        msg_speed.data=speed;
                        stop_flag.data = 0;
                        stop_line_det_pub.publish(stop_flag);
                        speeds_pub.publish(msg_speed);
                        msg_steering.data=90;
                        steering_pub.publish(msg_steering);
                        state=1;
                        break;
                case 1:
                        std::cout << "[State: 1] Cruising" << '\n';
                        if (objectF)
                        {
                                printf("[State: %d] Object @ front\n", state);
                                speeds_pub.publish(speed_obj);
                                stop_flag.data=1;
                                stop_line_det_pub.publish(stop_flag);
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

                        else if(cross)
                        {
                                printf("[State: %d] Found crosssing @ front, waiting\n", state);
                                msg_speed.data=0;
                                speeds_pub.publish(msg_speed);
                                cout << "j: " << j << "\n";
                                if(!objectF)
                                {
                                        j++;

                                }
                                else{
                                        j=0;
                                }

                                if (j>=40)
                                {

                                        state=11;

                                        j=0;
                                        // std::cout << "Waited:  " <<j<< '\n';
                                        // std::cout << j << '\n';
                                }

                        }
                        else {
                                msg_speed.data=speed;
                                speeds_pub.publish(msg_speed);
                                cout << "Speed Car: " << msg_speed.data <<"\n";
                                msg_steering.data=steering;
                                steering_pub.publish(msg_steering);
                                stop_flag.data=0;
                                stop_line_det_pub.publish(stop_flag);

                        }

                        break;

                case 2: //turn left to overtake obstacle
                        stop_flag.data = 1;
                        stop_line_det_pub.publish(stop_flag); // stops line detection to avoid obstacle.
                        printf("[State: %d] Turning left\n", state);
                        msg_steering.data=max_steer_left;
                        steering_pub.publish(msg_steering);
                        msg_speed.data=-100;
                        vu=vu_l1;
                        time_s=(t_100*100/msg_speed.data);
                        if (time_s < 0) {
                                time_s=-time_s;
                        }
                        speeds_pub.publish(msg_speed);
                        time_s=time_s*vu*1000000;
                        cout << "Time evasion 1: "<< time_s << "\n";
                        usleep(time_s);


                        sleep(1);
                        state=3;
                        break;

                case 3: //turn right to enter left lane
                        printf("[State: %d] Turning right\n", state);
                        stop_flag.data = 1;
                        stop_line_det_pub.publish(stop_flag); // stops line detection to avoid obstacle.
                        vu=vu_r1;
                        msg_steering.data=max_steer_right; // orig 20
                        steering_pub.publish(msg_steering);
                        msg_speed.data=-100;
                        time_s=(t_100*100/msg_speed.data);
                        if (time_s < 0) {
                                time_s=time_s* -1;
                        }
                        speeds_pub.publish(msg_speed);
                        time_s=time_s*vu*1000000;
                        cout << "Time evasion 2: "<< time_s << "\n";
                        usleep(time_s);
                        sleep(1);
                        state=4;


                        break;
                case 4: //we are now in the left lane
                        printf("[State: %d] On left lane, following line, looking for obstacle\n", state);
                        msg_steering.data=steering;
                        stop_flag.data = 1;
                        stop_line_det_pub.publish(stop_flag); // stops line detection to avoid obstacle.
                        steering_pub.publish(msg_steering);
                        msg_speed.data=speed;
                        speeds_pub.publish(msg_speed);

                        if (objectR)
                        {
                                state=5;
                        }
                        break;
                case 5:         //we are now in the left lane
                        printf("[State: %d] On left line, Found obstacle right\n", state);
                        stop_flag.data = 1;
                        stop_line_det_pub.publish(stop_flag); // stops line detection to avoid obstacle.
                        steering_pub.publish(msg_steering);
                        msg_speed.data=speed;
                        speeds_pub.publish(msg_speed);
                        if (!objectR)
                        {
                                state=6;
                        }
                        break;
                case 6: //object cleared return to right lane
                        printf("[State: %d]Turning to right lane\n", state);
                        vu=vu_r2;
                        stop_line_det_pub.publish(stop_flag); // stops line detection to avoid obstacle.
                        msg_steering.data=max_steer_right;
                        steering_pub.publish(msg_steering);
                        msg_speed.data=-100;

                        time_s=(t_100*100/msg_speed.data);
                        if (time_s < 0) {
                                time_s=time_s* -1;
                        }
                        time_s=time_s*vu*1000000;
                        speeds_pub.publish(msg_speed);

                        cout << "Time evasion 3: "<< time_s << "\n";
                        usleep(time_s);
                        sleep(1);
                        state=7;


                        break;
                case 7:
                        printf("[State: %d]Turning left to match right lane\n", state);
                        vu=vu_l2;
                        stop_line_det_pub.publish(stop_flag); // stops line detection to avoid obstacle.
                        msg_steering.data=max_steer_left;
                        steering_pub.publish(msg_steering);
                        msg_speed.data=-100;
                        speeds_pub.publish(msg_speed);

                        time_s=(t_100*100/msg_speed.data);

                        if (time_s < 0) {
                                time_s=time_s* -1;
                        }
                        time_s=time_s*vu*1000000;
                        cout << "Time evasion : "<< time_s << "\n";

                        usleep(time_s);

                        sleep(1);
                        state=8;
                        break;
                case 8:
                        printf("[State: %d] On right line, following line again\n", state);
                        stop_flag.data = 0;
                        stop_line_det_pub.publish(stop_flag); // continues line detection to avoid obstacle.
                        msg_steering.data=steering;
                        steering_pub.publish(msg_steering);
                        state=0;
                        break;

                case 11:
                        printf("[State: %d] Crossing follow line again\n", state);
                        msg_steering.data=steering;
                        steering_pub.publish(msg_steering);
                        msg_speed.data=speed;
                        speeds_pub.publish(msg_speed);
                        if(!cross) {
                                state=12; //crossing crossed return to cruising
                        }
                        break;
                case 12:
                        printf("[State: %d] Inside crossing\n", state);
                        msg_steering.data=steering;
                        steering_pub.publish(msg_steering);

                        msg_speed.data=speed;
                        speeds_pub.publish(msg_speed);

                        if(cross) {
                                state=13;                 //crossing crossed return to cruising
                        }
                        break;
                case 13:
                        printf("[State: %d] Exiting crossing\n", state);
                        msg_steering.data=steering;
                        steering_pub.publish(msg_steering);
                        msg_speed.data=speed;
                        speeds_pub.publish(msg_speed);
                        if(!cross) {
                                state=1;         //crossing crossed return to cruising
                        }
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
