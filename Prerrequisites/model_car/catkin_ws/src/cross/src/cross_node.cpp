#include "ros/ros.h"
#include "CrossDetector.h"

cv::Mat Image;
bool image = false;
cv::Mat persp; 
cv::Mat transf; 

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try{
     Image = cv_bridge::toCvShare(msg, "bgr8")->image;

     image = true;
    }catch ( cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
} 

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cross_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport its(nh);
  image_transport::Subscriber sub = its.subscribe("app/camera/rgb/image_raw",1,imageCallback);

  image_transport::ImageTransport itp(nh);
  image_transport::Publisher pub = itp.advertise("crossImg",1);
  sensor_msgs::ImagePtr msg ;

  ros::Publisher cross_pub= nh.advertise<std_msgs::Bool>("cross",1000);

  ros::Rate loop_rate(5);
  CrossDetector cross = CrossDetector();
  std_msgs::Bool msg_cross;

  /*std::vector< cv::Point2f> srcPoints; 
  srcPoints.push_back( cv::Point2f( 132, 470) ); 
  srcPoints.push_back( cv::Point2f( 195, 390) ); 
  srcPoints.push_back( cv::Point2f( 486, 479) ); 
  srcPoints.push_back( cv::Point2f( 415, 386) ); 
  
  std::vector< cv::Point2f> dstPoints; 
  dstPoints.push_back( cv::Point2f( 132, 470) ); 
  dstPoints.push_back( cv::Point2f( 132, (float)390*0.8) ); 
  dstPoints.push_back( cv::Point2f( 486, 479) ); 
  dstPoints.push_back( cv::Point2f( 486, (float)386*0.8) ); 



  transf = getPerspectiveTransform(srcPoints, dstPoints ); 
*/
  while (ros::ok()) {
    
    if (image)
    {
      bool crss= false;
      cv::Mat img = cross.segmentationCross(Image, crss);
      //cv::waitKey(1);
      
      
      msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", img).toImageMsg();
      pub.publish(msg);
      msg_cross.data = crss;
      cross_pub.publish(msg_cross);
      
      
      image= false;

    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}