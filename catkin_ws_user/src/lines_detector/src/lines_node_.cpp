#include "ros/ros.h"
#include "LinesDetector.h"

cv::Mat Image;
bool image = false;
cv::Mat persp; 
cv::Mat transf; 

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try{
     Image = cv_bridge::toCvShare(msg, "bgr8")->image;
     /*cv::imshow("image", Image); 

      
     cv::warpPerspective( Image, persp, transf, Image.size() ); 
         
     cv::imshow( "persp", persp ); 

     if( cv::waitKey(30) == 's')
     {
        cv::imwrite("/home/haime/img.jpg",Image);
        std::cout<<"image saved...."<<std::endl;
     }*/
     image = true;
    }catch ( cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
} 

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport its(nh);
  image_transport::Subscriber sub = its.subscribe("app/camera/rgb/image_raw",1,imageCallback);

  image_transport::ImageTransport itp(nh);
  image_transport::Publisher pub = itp.advertise("lines",1);
  sensor_msgs::ImagePtr msg ;

  ros::Publisher path_pub= nh.advertise<nav_msgs::Path>("path",1000);

  ros::Rate loop_rate(5);
  LinesDetector lines = LinesDetector();
  nav_msgs::Path msg_path;

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
      std::vector<geometry_msgs::PoseStamped> poses;
      cv::Mat img = lines.segmentationLines(Image, poses);
     // cv::Mat img = lines.segmentationLines(Image, poses);
      //cv::Mat line = lines.linesToPoints(img);
      
      
      msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", img).toImageMsg();
      pub.publish(msg);
      if (poses.size() > 10)
      {
      	msg_path.poses = poses;
        path_pub.publish(msg_path);
      }
      else{
      	poses.clear();
      	msg_path.poses = poses;
      	path_pub.publish(msg_path);
	  }
      image= false;

    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}