#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32.h>

cv::Mat extract_lane(cv::Mat image,  int lowValThr,int highValThr,std::vector<geometry_msgs::PoseStamped> &poses_right, std::vector<geometry_msgs::PoseStamped> &poses_left);
float calculate_lane_angle(std::vector<geometry_msgs::PoseStamped> &poses);
cv::Point getAverageCenterLanePosition(std::vector<geometry_msgs::PoseStamped> &poses);
void draw_angle_arrows(cv::Mat &image, cv::Point origin, int tam, float theta);
