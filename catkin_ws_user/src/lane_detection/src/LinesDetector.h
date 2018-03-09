#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>


class LinesDetector
{


public:
	int flagInverted; 
	

	LinesDetector();
	cv::Mat segmentationLines(cv::Mat image, std::vector<geometry_msgs::PoseStamped> &poses);
	//cv::Mat segmentationLines2(cv::Mat image, std::vector<geometry_msgs::PoseStamped> &poses);
		
};