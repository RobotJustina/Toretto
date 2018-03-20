#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <std_msgs/Bool.h>


class CrossDetector {
private:
int hough_thr;
double minLen;
double gapLen;
int lowValThr, highValThr;
int canny_thr_low, canny_thr_high;

public:
CrossDetector( int hough_thr,double minLen, double gapLen, int lowValThr,int highValThr, int canny_thr_low, int canny_thr_high);
void segmentationCross(cv::Mat edges, cv::Mat &imageCross,  bool &cross);
void get_borders(cv::Mat &image, cv::Mat &edges, bool color );
};
