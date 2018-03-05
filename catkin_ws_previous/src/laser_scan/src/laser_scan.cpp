#include <ros/ros.h>
#include <iostream>
#include <std_msgs/String.h>
#include <math.h>       
#include <sstream>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
using namespace std;

sensor_msgs::LaserScan laser;
std::vector<float> laser_ranges;
std::vector<float> park_ranges;
std_msgs::Int16 speed;
std_msgs::Float32 obsR;
std_msgs::Float32 obsL;

bool init=false, park_b=false;

int i=0, vmax=300;
float obs_1=0, obs_2=0,obs_3=0, obs_4=0, obs_5=0,obs_6=0, aux=0,dmin=0.75,dmax=1.25,obs=0;
float obsL_0=0,obsL_1=0,obsL_2=0,obsR_0=0,obsR_1=0,obsR_2=0;

void Callback_laser(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    laser_ranges.clear();
    for(size_t i=0; i < msg->ranges.size(); i++)
    {
       laser_ranges.push_back(msg->ranges[i]);
       //cout<<i<<" - " << laser_ranges[i] << "\n ";
    }    

    if(laser_ranges[0] < 10)
        obs_1=laser_ranges[0];
    else
        obs_1=10;
    if(laser_ranges[1] < 10)
        obs_2=laser_ranges[1];
    else
        obs_2=10;
    if(laser_ranges[359] < 10)
        obs_3=laser_ranges[359];
    else
        obs_3=10;

    if(laser_ranges[358] < 10)
        obs_4=laser_ranges[358];
    else
        obs_4=10;
    if(laser_ranges[2] < 10)
        obs_5=laser_ranges[2];
    else
        obs_5=10;
    if(laser_ranges[357] < 10)
        obs_6=laser_ranges[357];
    else
        obs_6=10;


    if(laser_ranges[268] < 10) 
       obsL_0=laser_ranges[268];
    if(laser_ranges[269] < 10) 
        obsL_1=laser_ranges[269];
    if(laser_ranges[270] < 10) 
        obsL_2=laser_ranges[270];
    

    if(laser_ranges[88] < 10) 
        obsR_0=laser_ranges[88];
    if(laser_ranges[89] < 10) 
        obsR_1=laser_ranges[89];
    if(laser_ranges[90] < 10) 
        obsR_2=laser_ranges[90];

    for(int i=270; i < 359 ; i++)
    {
        //cout <<i << "- "<< laser_ranges[i]<<"\n"; 
        if (laser_ranges[i] > 0.3 && laser_ranges[i] < 10){
      //  cout <<i << " - Park \n" ; 
        }
    }
    
    park_b=false;
    /*
    if(laser_ranges[256] < 10) 
        caja_3=laser_ranges[256];
    if(laser_ranges[257] < 10) 
        caja_4=laser_ranges[257];
    if(laser_ranges[258] < 10) 
        caja_5=laser_ranges[258];
    */
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "object_detection"); 
    ros::NodeHandle n;
    ros::Subscriber laser_subscriber;

    laser_subscriber = n.subscribe<sensor_msgs::LaserScan>("/scan", 100, Callback_laser);
    ros::Publisher speeds_pub = n.advertise<std_msgs::Int16>("/object_detection/speed", 1);
    ros::Publisher objectL_pub = n.advertise<std_msgs::Float32>("/object_detection/left", 1);
    ros::Publisher objectR_pub = n.advertise<std_msgs::Float32>("/object_detection/right", 1);
        

    while (ros::ok())
    {    
        ros::Rate loop_rate(10); 
        obs=(obs_1+obs_2+obs_3)/3;  
        obsL.data=(obsL_0+obsL_1+obsL_2)/3; 
        obsR.data=(obsR_0+obsR_1+obsR_2)/3;

        cout << "obsR: " << obsR.data << "\n";
        cout << "obsL: " << obsL.data << "\n";
        cout << "obs: " << obs << "\n";

        ros::spinOnce(); 
        
        if (obs <= 1.25){
                    aux =(obs-dmin)*100;
                    if (aux <= 0)
                        aux=0;
                    //cout <<"aux: "<< aux << "\n";
                    cout <<"vmax: "<< vmax << "\n";    
                    speed.data = (vmax/((dmax-dmin)*100) * aux)*-1;
                     cout <<"speed : "<< speed.data << "\n";    
                    if(speed.data < 10 && speed.data > -10){
                                speed.data=0;    
                                }

                    cout <<"speed obj: "<< speed.data << "\n";    
                    
                    speeds_pub.publish(speed);                

        }

        if (obsR.data < 0.5){
            objectR_pub.publish(obsR);  
        }

        if (obsL.data < 0.5){
            objectL_pub.publish(obsL);  
        }       
        
        loop_rate.sleep();
    }
}
