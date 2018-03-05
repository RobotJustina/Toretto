#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include "nav_msgs/Path.h"

nav_msgs::Path pathCenter;
nav_msgs::Path pathRight;
nav_msgs::Path pathLeft;

int pointsCenter = 1;
int pointsRight  = 2;

bool newPathData = false;

void callbackCenter(const nav_msgs::Path::ConstPtr& c)
{
    pathCenter=*c; 
    newPathData = true;
}

void callbackRight(const nav_msgs::Path::ConstPtr& r)
{
    pathRight=*r;
    newPathData = true;
}

void callbackLeft(const nav_msgs::Path::ConstPtr& l)
{
    pathLeft=*l;
    newPathData = true;
}

bool getPathSlope(nav_msgs::Path& path, int N, float& slope)
{
    if(path.poses.size() < N)
        return false;
    
    float sumXY = 0;
    float sumX  = 0;
    float sumY  = 0;
    float sumX2 = 0;
    for(int i=0; i < N; i++)
    {
        sumXY += path.poses[i].pose.position.x * path.poses[i].pose.position.y;
        sumX  += path.poses[i].pose.position.y;
        sumY  += path.poses[i].pose.position.x;
        sumX2 += path.poses[i].pose.position.y;
    }
    
    float m = (sumXY - sumX*sumY / N) / (sumX2 - sumX*sumX/N);
    slope = m;
    return true;
}

bool getPathMeanX(nav_msgs::Path& path, int N, float& meanX)
{
    if(path.poses.size() < N)
        return false;

    meanX = 0;
    for(int i=0; i < N; i++)
        meanX += path.poses[i].pose.position.x;
    meanX /= N;
    return true;
}

int main(int argc, char** argv)
{
    std::cout <<"********************Navigation Started*********************"<<std::endl;
    ros::init(argc,argv,"good_control");
    ros::NodeHandle n;
    ros::Publisher pub_speed = n.advertise<std_msgs::Int16>( "/manual_control/speed",1);
    ros::Publisher pub_steering = n.advertise<std_msgs::Int16>( "/manual_control/steering",1);
    ros::Subscriber sub_center = n.subscribe("/center",1,callbackCenter);
    ros::Subscriber sub_right = n.subscribe("/right",1,callbackRight);
    ros::Subscriber sub_left = n.subscribe("/left",1,callbackLeft);
    ros::Rate loop(10);
    
    std_msgs::Int16 msg_speed; 
    std_msgs::Int16 msg_steering;
    ros::topic::waitForMessage<nav_msgs::Path>("/center",ros::Duration(60));
    ros::topic::waitForMessage<nav_msgs::Path>("/right",ros::Duration(60));
    ros::topic::waitForMessage<nav_msgs::Path>("/left",ros::Duration(60));
    
    float idealCenter = 167;
    float idealRight  = 473;
    float imageCenter = 320;

    int pointsCenter = 1;
    int pointsRight  = 2;
    
    int nextState  = 0;
    int noData = 0;
    
    while(ros::ok())
    {
        if(!newPathData)
        {
            if(++noData > 10)
            {
                noData = 0;
                nextState = 100;
            }
        }
        else 
            noData = 0;
        newPathData = false;

        switch(nextState)
        {
        case 0:
            std::cout << "Moving in a straight line" << std::endl;
            nextState = 10;
            break;
        case 10:
            float meanCenter = 0;
            float meanRight  = 0;
            float error;
            if (pathCenter.poses.size() >= pointsCenter)
            {
                for(int i=0; i < pointsCenter; i++)
                    meanCenter += pathCenter.poses[i].pose.position.x;
                meanCenter /= pointsCenter;
            }
            if(pathRight.poses.size() > pointsRight)
            {
                for(int i=0; i < pointsRight; i++)
                    meanRight += pathRight.poses[i].pose.position.x;
                meanRight /= pointsRight;
            }
            if(meanCenter > 0 &&  meanRight > 0)
                error = (meanCenter + meanRight)/2.0 - imageCenter;
            else if(meanCenter > 0)
                error = meanCenter - idealCenter;
            else if(meanRight > 0)
                error = meanRight - idealRight;
            else
                std::cout << "Warning!! No lines detected" << std::endl;
        }
        
        
        pub_speed.publish(msg_speed);
        pub_steering.publish(msg_steering);
        
        ros::spinOnce();
        loop.sleep();
    }
    msg_speed.data=0;
    pub_speed.publish(msg_speed);
    ros::spinOnce();
    
    return 0;
}
