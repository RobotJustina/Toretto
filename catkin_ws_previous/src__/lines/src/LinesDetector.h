#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
//#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/features2d/features2d.hpp"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>


class LinesDetector
{


public:
	int flagInverted; 
	int thresh,thresh2;
	int thrlines, minLineLength , maxLineGap;
	LinesDetector();
	cv::Mat segmentationLines(cv::Mat image, std::vector<geometry_msgs::PoseStamped> &center, std::vector<geometry_msgs::PoseStamped> &lf , std::vector<geometry_msgs::PoseStamped> &rg);
	void imageSave(cv::Mat image);
	cv::Mat homography(cv::Mat image, cv::Mat matrixH);
		
};
