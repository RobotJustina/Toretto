#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include "nav_msgs/Path.h"

nav_msgs::Path pathCenter;
nav_msgs::Path pathRight;
nav_msgs::Path pathLeft;

bool newPathData = false;

void callbackCenter(const nav_msgs::Path::ConstPtr& c)
{
	pathCenter=*c; 
}

void callbackRight(const nav_msgs::Path::ConstPtr& r)
{
	pathRight=*r;
}

void callbackLeft(const nav_msgs::Path::ConstPtr& l)
{
	pathLeft=*l;
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
	ros::Rate loop(30);

	std_msgs::Int16 msg_speed; 
	std_msgs::Int16 msg_steering;
	ros::topic::waitForMessage<nav_msgs::Path>("/center",ros::Duration(60));
	ros::topic::waitForMessage<nav_msgs::Path>("/right",ros::Duration(60));
	ros::topic::waitForMessage<nav_msgs::Path>("/left",ros::Duration(60));

	float idealCenter = 84;
	//float idealCenter = 177;
	float idealRight  = 236;
	float lastError = 0;
	while(ros::ok())
	{
		if(!newPathData)
		{
			ros::spinOnce();
			loop.sleep();
		}
		newPathData = false;

		msg_speed.data= -500;
		float lane=0;
		float error=0;
		if (pathCenter.poses.size() > 0 && pathRight.poses.size() >1)
		{
			float meanCenter = pathCenter.poses[0].pose.position.x;
			float meanRight = (pathRight.poses[0].pose.position.x + pathRight.poses[1].pose.position.x)/2;
			lane = (meanCenter+meanRight)/2 ;
			error = lane - 162.5;	
			//std::cout<<"Mean center: "<<meanCenter<<"Mean right: "<<meanRight<<std::endl;
		}
		else if(pathCenter.poses.size() > 0)
		{
		    lane = pathCenter.poses[0].pose.position.x;
		    error = lane - idealCenter;
		}
		else if(pathRight.poses.size() > 1)
		{
			lane= (pathRight.poses[0].pose.position.x + pathRight.poses[1].pose.position.x)/2;
			error = lane - idealRight;
		}/*else if(pathLeft.poses.size() > 1)
		{
			lane= (pathLeft.poses[0].pose.position.x + pathLeft.poses[1].pose.position.x)/2;
			error= lane - idealRight;
		}*/
		else{
			std::cout<<"Warning no lines detected :'("<<std::endl;
			msg_speed.data = -500;
		}

		if(pathRight.poses.size() > 9)
		{
			float sumXY = 0;
			float sumX  = 0;
			float sumY  = 0;
			float sumX2 = 0;
			for(int i=0; i < 10; i++)
			{
				sumXY += pathRight.poses[i].pose.position.x * pathRight.poses[i].pose.position.y;
				sumX  += pathRight.poses[i].pose.position.y;
				sumY  += pathRight.poses[i].pose.position.x;
				sumX2 += pathRight.poses[i].pose.position.y;
			}

			float m = (sumXY - sumX*sumY / 10) / (sumX2 - sumX*sumX/10);
			//std::cout << "Right line slope: " << m << std::endl;
			if (m<-0.05)
			{
				std::cout<<"Left slope detected "<<std::endl;
			}else if (m>0.05)
			{
				std::cout<<"Right slope detected"<<std::endl;
			}
			if(fabs(m) >= 0.05 && msg_speed.data < 0)
				msg_speed.data = -500;
		}
		
		if(fabs(error - lastError) < 50){
			msg_steering.data= 120 - ( int16_t)error/2;
			if (msg_steering.data < 40){
				msg_steering.data=40;
			}
			else if (msg_steering.data > 200){
				msg_steering.data=200;
			}
		}
		lastError = error;
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
