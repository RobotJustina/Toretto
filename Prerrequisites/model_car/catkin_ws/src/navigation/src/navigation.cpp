#include <ros/ros.h>
#include <iostream>
#include <std_msgs/String.h>
#include <math.h>
#include <sstream>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <math.h>
#include "std_msgs/Int32MultiArray.h"
#include "nav_msgs/Path.h"
#include <time.h>

#define PI 3.14159265
#define E 2.7182818284590
#define Steering_max = 0.57

using namespace std;

sensor_msgs::LaserScan laser;
std::vector<float> laser_ranges;
std_msgs::Int16 speed;
std_msgs::Int16 speed_obj;
std_msgs::Int16 speed_rev;

std_msgs::Int16 steering;
std_msgs::String lights_fr;


bool object=false,objectR=false,objectL=false, rev=false, evasion=false, b_stop=false, cross=false, cross_a=false, cross_b=false;

int i=0,j=0,k=0, l=0, vmax=1000,steering_0=120, steering_1=120, steering_prom=120, steering_prom_1=120;
int steering_2=120, steering_3=120, limite=15;
int nextstate=0;
int speed_var;
clock_t time_i=0,time_f=0;

float time_s=0, t_100=3.4;
float x_i=0, y_i=0, theta_i=0;
float x_i1=0, y_i1=0, theta_i1=0;
float err_theta=0;
float V=0, Vmax=2;
float alfa=1, vy=0, vx=0;
float l_obj=0, r_obj=0;
float en=0, en_1=0, Kp=0.03, Rn=0;

ros::Publisher speeds_pub;
ros::Publisher steering_pub;
ros::Publisher light_fr_pub;

//P controller
float navigation(){

        //theta_i1= atan((x_i-x_i1)/(y_i - y_i1));
        theta_i1=0.0026*x_i;
        theta_i1=theta_i1* -1;

        /*if (x_i > 0){
            theta_i1=(theta_i1/*-1.5708);
           }
           else
           {
            theta_i1=(theta_i1+1.5708);
           }*/

        return theta_i1;
        /*
            err_theta = theta_i1 - theta_i;

            V= Vmax*pow(E,((pow(err_theta,2)/ 2* alfa))*-1);

            cout << "t: " << theta_i1 << "\n";
            cout << "V: " << V << "\n";

            vx = Vmax * cos(theta_i1);
            vy = Vmax * sin(theta_i1);

            y_i1 = vy + y_i1;
            x_i1 = vx + x_i1;

            cout << "xi: " << x_i1 << "\n";
         */
}


//Curva muy pronunciada
void giro(float theta)
{
        time_s=(t_100*100/speed.data)* -1;
        steering.data=290;
        steering_pub.publish(steering);
        cout << "Steering: " << steering.data <<"\n";
        cout << "Time: " << time_s <<"\n";
        sleep(time_s);
        steering.data=120;
        steering_pub.publish(steering);
        cout << "Steering: " << steering.data <<"\n";

}

void Callback_path(const nav_msgs::Path& path)
{
        //time_f=clock()-time_f;
        //cout << "Time J: " << (float)time_f/CLOCKS_PER_SEC << "\n";
        //time_i=clock();

        if(evasion) {
                cout <<"Evasión \n";
        }

        else{

                float theta;
                x_i=0;
                y_i=0;


                if (path.poses.size() > 25) {
                        cout << "10 \n";
                        for (int i=0; i< 25; i++) { //15

                                //cout << "x_i: " << path.poses[i].pose.position.x << "\n";
                                x_i += path.poses[i].pose.position.x; //Pixel measurement
                                y_i += path.poses[i].pose.position.y;

                                //cout << "x_i: " << path.poses[i].pose.position.x << "\n";
                                //cout << "x_i: " << path.poses[i].pose.position.x << "\n";
                                /*if (abs(path.poses[i].pose.position.x - x_i1) > 200){
                                    cout << "Es mayor por: " << abs(path.poses[i].pose.position.x - x_i1)<< "\n";
                                    cout << "Path: " << path.poses[i].pose.position.x << "\n";
                                    cout << "x_i1: " << x_i1 << "\n";
                                    limite++;
                                   }
                                   else{
                                    x_i += path.poses[i].pose.position.x;
                                    y_i += path.poses[i].pose.position.y;
                                   }
                                   cout << "x_i: " << path.poses[i].pose.position.x << "\n";*/
                        }

                        x_i=x_i/25;
                        y_i=y_i/25;
                }

                else{
                        for (int i=0; i< path.poses.size(); i++) { //15

                                //cout << "Menos de 15 \n";
                                x_i += path.poses[i].pose.position.x;
                                y_i += path.poses[i].pose.position.y;

                                //cout << "x_i: " << path.poses[i].pose.position.x << "\n";
                                //cout << "x_i: " << path.poses[i].pose.position.x << "\n";
                                /*if (abs(path.poses[i].pose.position.x - x_i1) > 200){
                                    cout << "Es mayor por: " << abs(path.poses[i].pose.position.x - x_i1)<< "\n";
                                    cout << "Path: " << path.poses[i].pose.position.x << "\n";
                                    cout << "x_i1: " << x_i1 << "\n";
                                    limite++;
                                   }
                                   else{
                                    x_i += path.poses[i].pose.position.x;
                                    y_i += path.poses[i].pose.position.y;
                                   }
                                   cout << "x_i: " << path.poses[i].pose.position.x << "\n";*/
                        }

                        x_i=x_i/path.poses.size();
                        y_i=y_i/path.poses.size();

                }


                //x_i1=x_i;
                x_i=x_i-225;
                y_i=299-y_i;
                x_i=x_i*0.7;
                //y_i=300-y_i;


                cout << "x_i prom: "<< x_i << "\n";
                cout << "y_i prom: "<< y_i << "\n";

                theta= navigation();


                //steering_3=steering_2;
                //steering_2=steering_1;
                steering_1=steering_0;
                //steering_prom_1=steering_prom;
                //steering_prom=(steering_1+steering_0)/2;
                en_1 = en;

                //if (theta > 0.05 || theta < -0.05){

                cout << "theta: " << theta << "\n";
                steering_0=((170/0.57)*theta)+120;
                cout << "Steering 0: " << steering_0 << "\n";
                if (steering_0 > 200) {
                        steering_0=200;
                }

                if (steering_0 < 40) {
                        steering_0=40;
                }

                /*}

                   else{
                    cout << "theta: " << theta << "\n";
                    steering_0=120;
                   }*/

                if (path.poses.size() < 5) {
                        steering_0=-30;
                }

                if (abs(steering_0  - steering_1) > 200) {
                        cout << "gran cambio \n";
                        steering_0=-40;
                }

                //Control
                //en=steering_prom - steering_prom_1;
                en=steering_0 - steering_1;
                Rn=Kp*(en - en_1);
                steering.data=steering_1+Rn;
                //steering.data=steering_0;


                cout << "Rn: " << Rn << "\n";
                //cout << "Steering 1: " << steering_1 << "\n";
                //steering.data= (steering_0+steering_1)/2;
                cout << "Steering 0: " << steering_0 << "\n";
                cout << "Steering 1: " << steering_1 << "\n";

                if (steering.data > 280) {
                        steering.data=280;
                }

                if (steering.data < -40) {
                        steering.data=-40;
                }

                if (steering.data > 220 || steering.data < 30) {
                        steering.data=steering.data*0.85;
                }
/*
    if (path.poses.size() < 5)
       {

        steering.data=-40;

       } */
                steering.data=((steering.data-170)*0.75)+170;
                cout << "Steering data: " << steering.data << "\n";
                steering_pub.publish(steering);

                cout << "Steering data: " << steering.data << "\n";
                steering_pub.publish(steering);
                usleep(250000);

                //time_i=clock()-time_i;
                //cout << "Time E: " << (float)time_i/CLOCKS_PER_SEC << "\n";
        }
}



void Callback_object(const std_msgs::Int16::ConstPtr& msg)
{
        if(msg->data < 100) {
                cout << "object true" << "\n";
                object=true;
                speed_obj.data = msg->data;
        }
        else{
                object=false;
                speed_obj.data = msg->data;
        }

}

void Callback_stop(const std_msgs::Int16::ConstPtr& msg)
{

        if(msg->data == 1) {
                cout << "Stop" << "\n";
                b_stop=true;
        }


}

void Callback_objectR(const std_msgs::Float32::ConstPtr& msg)
{
        cout << "object right true" << "\n";

        if(msg->data < 0.35) {
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

void Callback_speed(const std_msgs::Int16::ConstPtr& msg)
{
        cout << "Speed change" << "\n";
        speed_var= msg->data;

}

int main(int argc, char** argv)
{
        ros::init(argc, argv, "navigation");
        ros::NodeHandle n;

        speeds_pub = n.advertise<std_msgs::Int16>("/manual_control/speed", 1);
        steering_pub = n.advertise<std_msgs::Int16>("/manual_control/steering", 1);

        //speeds_pub = n.advertise<std_msgs::Int16>("/controlN/speed", 1);
        //steering_pub = n.advertise<std_msgs::Int16>("/controlN/steering", 1);

        light_fr_pub = n.advertise<std_msgs::String>("/manual_control/lights", 1);

        ros::Subscriber position_subscriber = n.subscribe("/right", 1, Callback_path);
        ros::Subscriber object_subscriber = n.subscribe("/object_detection/speed", 1, Callback_object);
        ros::Subscriber objectL_subscriber = n.subscribe("/object_detection/left", 1, Callback_objectL);
        ros::Subscriber objectR_subscriber = n.subscribe("/object_detection/right", 1, Callback_objectR);
        ros::Subscriber stop_subscriber = n.subscribe("/manual_control/stop", 1, Callback_stop);

        ros::Subscriber speed_subscriber = n.subscribe("/manual_control/speed_auto", 1, Callback_speed);

        ros::Subscriber cross_subscriber = n.subscribe("/cross", 1, Callback_cross);


        while (ros::ok())
        {

                ros::Rate loop_rate(10);
                float vu=0;
                ros::spinOnce();

                if (b_stop) {
                        cout << "stop \n";
                        speed.data=0;
                        speeds_pub.publish(speed);
                        //steering.data=120;
                        //steering_pub.publish(speed);
                        b_stop=false;
                }

                else{
                        switch (nextstate) {

                        case 0:
                                cout<<"Case 0 \n";
                                lights_fr.data="diL";
                                light_fr_pub.publish(lights_fr);
                                ros::spinOnce();
                                j=0;
                                object=false;
                                cross=false;
                                cross_a=false;
                                object=false;


                                //steering.data=120;
                                //steering_pub.publish(steering);
                                nextstate=0;
                                //sleep(1);
                                //ros::spinOnce();
                                //steering.data=120;
                                //steering_pub.publish(steering);
                                break;

                        case 1: ///Detecto algo enfrente, del laser
                                lights_fr.data="diL";
                                light_fr_pub.publish(lights_fr);
                                cout << "case 1 \n";
                                if(object) {
                                        cout << "object true case 1 \n";
                                        speeds_pub.publish(speed_obj);
                                        cout << "j: " << j << "\n";
                                        cout << "Speed Object: " << speed_obj.data <<"\n";
                                        //object=false;
                                        if(speed_obj.data == 0) {
                                                j++;
                                        }
                                        else{
                                                j=0;
                                        }

                                        if (j > 40) {
                                                cout << "Evasion True \n";
                                                evasion=true;
                                                nextstate= 2;
                                                j=0;
                                        }

                                }

                                else{
                                        /*
                                           //Control
                                            //en=steering_prom - steering_prom_1;
                                            en=steering_0 - steering_1;
                                            Rn=Kp*(en - en_1);
                                            //steering.data=steering_prom_1+Rn;
                                            steering.data=steering_1+Rn;


                                            cout << "Rn: " << Rn << "\n";
                                            //cout << "Steering 1: " << steering_1 << "\n";
                                            //steering.data= (steering_0+steering_1)/2;
                                            cout << "Steering 0: " << steering_0 << "\n";
                                            cout << "Steering 1: " << steering_1 << "\n";

                                            if (steering.data > 280){
                                                    steering.data=280;
                                                }

                                            if (steering.data < -40){
                                                    steering.data=-40;
                                                }

                                            if (steering.data > 220 || steering.data < 30){
                                                steering.data=steering.data*0.85;
                                            }

                                            cout << "Steering data: " << steering.data << "\n";
                                            steering_pub.publish(steering);
                                         */

                                        speed.data=-350;
                                        speeds_pub.publish(speed);
                                        cout << "Speed: " << speed.data <<"\n";
                                        evasion=false;
                                        j=0;

                                        cout << "cross b" << cross_b << "\n";
                                        //if(!cross_b){
                                        if (cross) {
                                                cout<<"cruzando";
                                                cross_a=true;
                                        }

                                        if(cross_a) {
                                                if(!cross) {
                                                        nextstate=10;
                                                        speed.data=0;
                                                        speeds_pub.publish(speed);
                                                        cross_a=false;
                                                }
                                        }
                                        //    }
                                        //else{
                                        //	cout<<"Segunda linea \n";
                                        //}

                                }



                                break;

                        case 2: //Evasion der
                                cout << "Case 2 \n";
                                lights_fr.data="le";
                                light_fr_pub.publish(lights_fr);

                                steering.data=290;
                                steering_pub.publish(steering);
                                speed.data=-100;
                                speeds_pub.publish(speed);
                                //vu=2.4;
                                vu=1.9;
                                time_s=(t_100*100/speed.data);
                                if (time_s < 0) {
                                        time_s=time_s* -1;
                                }
                                time_s=time_s*vu*1000000;
                                cout << "Time evasion 1: "<< time_s << "\n";
                                ros::spinOnce();
                                usleep(time_s);
                                speed.data=0;
                                speeds_pub.publish(speed);
                                sleep(1);
                                nextstate=3;
                                object=false;
                                //seguir linea

                                break;
                        case 3: //Evasion izq
                                vu=2;
                                steering.data=20;
                                steering_pub.publish(steering);
                                speed.data=-100;
                                speeds_pub.publish(speed);
                                time_s=(t_100*100/speed.data);
                                if (time_s < 0) {
                                        time_s=time_s* -1;
                                }
                                time_s=time_s*vu*1000000;
                                cout << "Time evasion 2: "<< time_s << "\n";
                                usleep(time_s);
                                speed.data=0;
                                speeds_pub.publish(speed);
                                sleep(1);
                                j=0;
                                nextstate=4;
                                evasion=false;
                                break;

                        case 4: //cambio de carril
                                cout << "Case 4 \n";
                                speed.data=-200;
                                speeds_pub.publish(speed);

                                //Segir linea

                                l++;
                                if (l>5) {
                                        nextstate=5;
                                        l=0;
                                }

                                /*vu=2.7;
                                   steering.data=20;
                                   steering_pub.publish(steering);
                                   speed.data=-100;
                                   speeds_pub.publish(speed);
                                   time_s=(t_100*100/speed.data);
                                   if (time_s < 0){
                                    time_s=time_s*-1;
                                   }
                                   time_s=time_s*vu*1000000;
                                   cout << "Time evasion 2: "<< time_s << "\n";
                                   usleep(time_s);
                                   speed.data=0;
                                   speeds_pub.publish(speed);
                                   sleep(1);
                                   j=0;
                                   nextstate=4;*/

                                break;

                        case 5:
                                cout << "Case 5 \n";
                                lights_fr.data="diL";
                                light_fr_pub.publish(lights_fr);

                                steering.data=120;
                                steering_pub.publish(steering);
                                speed.data=-200;
                                speeds_pub.publish(speed);
                                cout << "ObjectR: "<< objectR << "\n";

                                if (!objectR) {
                                        j++;
                                }
                                else{
                                        j=0;
                                }
                                if (j > 2)
                                {
                                        nextstate=6;
                                        evasion=true;
                                        j=0;
                                }

                                cout << "J: "<< j<< "\n";

                                objectR=false;

                                break;

                        case 6:
                                cout << "Case 6 \n";
                                lights_fr.data="ri";
                                light_fr_pub.publish(lights_fr);

                                steering.data=-50;
                                steering_pub.publish(steering);
                                speed.data=-100;
                                speeds_pub.publish(speed);
                                vu=1.6;
                                time_s=(t_100*100/speed.data);
                                if (time_s < 0) {
                                        time_s=time_s* -1;
                                }
                                time_s=time_s*vu*1000000;
                                cout << "Time evasion 1: "<< time_s << "\n";
                                ros::spinOnce();
                                usleep(time_s);
                                speed.data=0;
                                speeds_pub.publish(speed);
                                sleep(1);
                                nextstate=7;

                                break;

                        case 7:
                                cout << "Case 7 \n";
                                vu=0.7;
                                steering.data=230;
                                steering_pub.publish(steering);
                                speed.data=-100;
                                speeds_pub.publish(speed);
                                time_s=(t_100*100/speed.data);

                                if (time_s < 0) {
                                        time_s=time_s* -1;
                                }
                                time_s=time_s*vu*1000000;

                                usleep(time_s);
                                speed.data=0;
                                speeds_pub.publish(speed);
                                sleep(1);
                                nextstate=8;
                                evasion=false;
                                break;

                        case 8:
                                cout << "Case 8 \n";
                                l++;
                                speed.data=-200;
                                speeds_pub.publish(speed);

                                if (l>5) {
                                        nextstate=9;
                                        l=0;
                                }
                                /*
                                   vu=1.5;
                                   steering.data=230;
                                   steering_pub.publish(steering);
                                   speed.data=-100;
                                   speeds_pub.publish(speed);
                                   time_s=(t_100*100/speed.data);

                                   if (time_s < 0){
                                    time_s=time_s*-1;
                                   }
                                   time_s=time_s*vu*1000000;

                                   usleep(time_s);
                                   speed.data=0;
                                   speeds_pub.publish(speed);
                                   sleep(1);
                                   nextstate=7;*/


                                break;

                        case 9:
                                cout << "Case 9 \n";
                                lights_fr.data="diL";
                                light_fr_pub.publish(lights_fr);

                                //steering.data=120;
                                //steering_pub.publish(steering);
                                //speed.data=-100;
                                //speeds_pub.publish(speed);

                                nextstate = 0;
                                break;

                        case 10:

                                cout << "Case 10 cross\n";
                                lights_fr.data="stop";
                                light_fr_pub.publish(lights_fr);

                                speed.data=0;
                                speeds_pub.publish(speed);
                                object=false;
                                objectR=false;
                                objectL=false;

                                nextstate = 11;
                                break;

                        case 11:

                                cout << "Case 11 time \n";
                                sleep(5);
                                if(!object && !objectR && !objectL) {
                                        lights_fr.data="diL";
                                        light_fr_pub.publish(lights_fr);
                                        vu=6;
                                        steering.data=125;
                                        steering_pub.publish(steering);
                                        speed.data=-150;
                                        speeds_pub.publish(speed);
                                        time_s=(t_100*100/speed.data);

                                        if (time_s < 0) {
                                                time_s=time_s* -1;
                                        }
                                        time_s=time_s*vu*1000000;
                                        usleep(time_s);
                                        speed.data=0;
                                        speeds_pub.publish(speed);

                                        ros::spinOnce();
                                        nextstate=0;
                                }

                                else{
                                        sleep(1);
                                        cout << "Object: " << object << "\n";
                                        cout << "ObjectR: " << objectR << "\n";
                                        cout << "ObjectL: " << objectL << "\n";
                                        object=false;
                                        objectR=false;
                                        objectL=false;
                                }

                                break;

                        default:
                                cout <<"error \n";
                                break;
                        }
                }
                loop_rate.sleep();

        }

}
