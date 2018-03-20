#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <std_msgs/Bool.h>



class CrossDetector
{


public:
	int flagInverted; 
	

	CrossDetector();
	cv::Mat segmentationCross(cv::Mat image, bool &cross);

		
};