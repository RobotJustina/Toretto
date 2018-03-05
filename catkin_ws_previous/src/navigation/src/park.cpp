#include <ros/ros.h>
#include <iostream>
#include <std_msgs/String.h>
#include <math.h>       
#include <sstream>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <math.h>
#include "std_msgs/Int32MultiArray.h"
#include "nav_msgs/Path.h"

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

bool object=false,objectR=false,objectL=false, rev=false, park=false;

float caja=0;

int i=0,j=0, vmax=1000,steering_0=120, steering_1=120, steering_prom=120, steering_prom_1=120;
int nextstate=0;

float time_s=0, t_100=3.4, kp=0.05;
float x_i=0, y_i=0, theta_i=0;
float x_i1=0, y_i1=0, theta_i1=0;
float err_theta=0;
float V=0, Vmax=2;
float alfa=1, vy=0, vx=0;
float l_obj=0, r_obj=0;
float en=0, en_1=0, Kp=0.5, Rn=0;


ros::Publisher speeds_pub;
ros::Publisher steering_pub;
ros::Publisher light_fr_pub;

float navigation(){

        //theta_i1= atan((x_i-x_i1)/(y_i - y_i1));
        theta_i1=0.0026*x_i;
        theta_i1=theta_i1*-1;
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

void Callback_path(const nav_msgs::Path& path)
{
    if(park){
        cout<<"park \n";
    }

    else{


        float theta;
    x_i=0;
    y_i=0;


    if (path.poses.size() > 25){
            cout << "10 \n"; 
            for (int i=0; i< 25; i++){ //15

                //cout << "x_i: " << path.poses[i].pose.position.x << "\n";
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

            x_i=x_i/25;
            y_i=y_i/25;
    }

    else{
        for (int i=0; i< path.poses.size(); i++){ //15

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
    x_i=x_i-450;
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
    
    if (theta > 0.05 || theta < -0.05){
        
        cout << "theta: " << theta << "\n";
        steering_0=((170/0.57)*theta)+120;
        cout << "Steering 0: " << steering_0 << "\n";
        if (steering_0 > 280){
            steering_0=280;
        }

        if (steering_0 < -40){
            steering_0=-40;
        }

    }

    else{
        cout << "theta: " << theta << "\n";
        steering_0=120;
    }

    if (path.poses.size() < 5){
           steering_0=-30;
    }

    if (abs(steering_0  - steering_1) > 200){
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

    if (steering.data > 280){
            steering.data=280;
        }

    if (steering.data < -40){
            steering.data=-40;
        }

    if (steering.data > 220 || steering.data < 30){
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
      cout << "object true" << "\n";
      object=true;
      speed_obj.data = msg->data ;
                
}


void Callback_objectR(const std_msgs::Float32::ConstPtr& msg)
{
      //cout << "object right true" << "\n";
        
      if(msg->data < 0.50){
            objectR=true;
      }
      r_obj = msg->data ;
                
}


void Callback_objectL(const std_msgs::Float32::ConstPtr& msg)
{
      //cout << "object left true" << "\n";
      
      if(msg->data < 0.25){
            objectL=true;
      }
      l_obj = msg->data ;
                
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "navigation");       
    ros::NodeHandle n;   
    int nextstate = 0;
    i=0;

   	ros::Subscriber position_subscriber = n.subscribe("/path", 1, Callback_path);
    
    speeds_pub = n.advertise<std_msgs::Int16>("/manual_control/speed", 1);
    steering_pub = n.advertise<std_msgs::Int16>("/manual_control/steering", 1);
    light_fr_pub = n.advertise<std_msgs::String>("/manual_control/lights", 1);

    ros::Subscriber object_subscriber = n.subscribe("/object_detection/speed", 1, Callback_object);
    ros::Subscriber objectL_subscriber = n.subscribe("/object_detection/left", 1, Callback_objectL);
    ros::Subscriber objectR_subscriber = n.subscribe("/object_detection/right", 1, Callback_objectR);
    
    while (ros::ok())
    {    
        ros::Rate loop_rate(10); 
        ros::spinOnce(); 
        float vu=0;
    
        switch (nextstate){

            case 0:
                cout << "Case init f \n";
                
                lights_fr.data="fr";
                light_fr_pub.publish(lights_fr);

                //steering.data=120;
                //steering_pub.publish(steering);
                speed.data=-200;
                speeds_pub.publish(speed);
                sleep(1);
                ros::spinOnce(); 
                speed.data=-200;
                speeds_pub.publish(speed);
                lights_fr.data="fr";
                light_fr_pub.publish(lights_fr);
                lights_fr.data="ri";
                light_fr_pub.publish(lights_fr);
                sleep(1);
                
                nextstate=1;
                break;

            case 1:
                lights_fr.data="fr";
                light_fr_pub.publish(lights_fr);
                
                cout << "Case 0 \n";
                cout << "esp caja 0 \n";
                if (objectR){
                    i++;
                        if(i > 3){
                        nextstate=2;
                        objectR=false;
                        i=0;
                        }
                    }
                else{

                }    
                break;    

            case 2:    
                cout << "Case 1 \n";
                cout << "caja 1 \n";
                cout << "objectR: "<< objectR << " \n";
                
                if (!objectR){
                    i++;
                    if(i > 3){
                    nextstate=3;
                    i=0;
                    }
                }
                objectR=false;    
                break;

            case 3:    
                cout << "Case 2 \n";
                cout << "esp caja 1 \n";
                if (objectR){
                    objectR=false;  
                    speed.data=0;
                    speeds_pub.publish(speed);
                    ros::spinOnce(); /*
                    speed.data=-100;
                   	sleep(1);
                    vu=0.5;
                    time_s=(t_100*100/speed.data);
                    if (time_s < 0){
                       time_s=time_s*-1;
                    }
                    time_s=time_s*vu*1000000;    
                    cout << "Time case 3: "<< time_s << "\n";
                    speeds_pub.publish(speed);    
                    usleep(time_s);
                    speed.data=0;
                    speeds_pub.publish(speed);*/
                    lights_fr.data="stop";
                    light_fr_pub.publish(lights_fr);
                    sleep(1);
                    nextstate=4;                     
                }
                break;
            
            case 4:

                park=true;
                vu=2.35;
                cout << "Case 4 \n";
                steering.data=-50;
                steering_pub.publish(steering);
                cout << "Pub Steering :" << steering.data << "\n";
                speed.data=100;
                speeds_pub.publish(speed);
                lights_fr.data="diL";
                light_fr_pub.publish(lights_fr);
                lights_fr.data="re";
                light_fr_pub.publish(lights_fr);
                lights_fr.data="fr";
                light_fr_pub.publish(lights_fr);
                                
                cout << "Pub speed :" << steering.data << "\n";
                
                time_s=(t_100*100/speed.data);
                time_s=time_s*vu;
                if (time_s < 0){
                    time_s=time_s*-1;
                }
                cout << "Time case 0: "<< time_s << "\n";
                sleep(time_s);
                speed.data=0;
                speeds_pub.publish(speed);
                sleep(1);
                
                nextstate=5;
                break;

            case 5:    

                vu=2.2;
                cout << "Case 5 \n";
                steering.data=290;
                steering_pub.publish(steering);
                speed.data=100;
                speeds_pub.publish(speed);
                time_s=(t_100*100/speed.data);
                time_s=time_s*vu;
                if (time_s < 0){
                    time_s=time_s*-1;
                }

                cout << "Time case 1: "<< time_s << "\n";
                sleep(time_s);
                speed.data=0;
                speeds_pub.publish(speed);
                sleep(1);
                
                nextstate=6;
                break;

            case 6:    

                vu=0.8;
                cout << "Case 6 \n";
                steering.data=120;
                steering_pub.publish(steering);
                lights_fr.data="diL";
                light_fr_pub.publish(lights_fr);
                lights_fr.data="fr";
                light_fr_pub.publish(lights_fr);
                
                sleep(1);
                speed.data=-100;
                speeds_pub.publish(speed);
                time_s=(t_100*100/speed.data);
                cout << "Time case 2: "<< time_s << "\n";
                time_s=time_s*vu;
                
                if (time_s < 0){
                    time_s=time_s*-1;
                }

                cout << "Time case 2: "<< time_s << "\n";
                sleep(time_s);
                
                nextstate=7;
                break;

            case 7:    

                cout << "End park \n";
                speed.data=0;
                speeds_pub.publish(speed);
                sleep(3);
                lights_fr.data="diL";
                light_fr_pub.publish(lights_fr);
                
                break;
                
        }
       loop_rate.sleep();
    }

    
}
