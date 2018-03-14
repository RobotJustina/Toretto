#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <algorithm> 
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <string>
#include <std_msgs/Float32MultiArray.h>


class LinesDetector
{


public:
	int flagInverted; 
	bool debug;

	LinesDetector(bool debug);
	cv::Mat segmentationLines(cv::Mat image, std_msgs::Float32MultiArray &lRight,std_msgs::Float32MultiArray &lLeft);
	void transformMatrix(cv::Mat img);

private:
	
	float distance(cv::Point2f point1,cv::Point2f point2);
	float distanceX(cv::Point2f point1,cv::Point2f point2);
	std::vector<cv::Point2f> peakHistrogram(cv::Mat image);
	void linesVector(std::vector<cv::Point2f> p,std_msgs::Float32MultiArray &l, int cols, cv::Mat &drawing,cv::Scalar color);

		
};