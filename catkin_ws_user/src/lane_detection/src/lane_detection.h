#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>


class lane_extractor {
private:
int hough_thr;
double minLen;
double gapLen;
int lowValThr, highValThr;
int canny_thr_low, canny_thr_high;
std_msgs::Float32MultiArray msg_direction;
public:
lane_extractor( int hough_thr,double minLen, double gapLen, int lowValThr,int highValThr, int canny_thr_low, int canny_thr_high);
std_msgs::Float32MultiArray extract_right_lane_hough(cv::Mat &image,cv::Mat &viz);
std_msgs::Float32MultiArray extract_left_lane_hough(cv::Mat &image,cv::Mat &viz);
void get_borders(cv::Mat &image, cv::Mat &edges ,bool color);
};


cv::Mat extract_lane(cv::Mat image,  int lowValThr,int highValThr,std::vector<geometry_msgs::PoseStamped> &poses_right, std::vector<geometry_msgs::PoseStamped> &poses_left);
cv::Mat extract_lane_angle(cv::Mat image,  int lowValThr,int highValThr,std_msgs::Float32MultiArray &angle_right, std_msgs::Float32MultiArray &angle_left);
float calculate_lane_angle(std::vector<geometry_msgs::PoseStamped> &poses);
cv::Point getAverageCenterLanePosition(std::vector<geometry_msgs::PoseStamped> &poses);
void draw_angle_arrows(cv::Mat &image, cv::Point origin, int tam, float theta);
