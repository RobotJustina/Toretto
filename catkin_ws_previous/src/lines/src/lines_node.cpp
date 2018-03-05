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
  cv::Mat transfMatrix;
  cv::FileStorage fs("/root/catkin_ws_user/homoMat4.yaml", cv::FileStorage::READ);
  if (!fs.isOpened())
  {
   		std::cout<<"No homoMat4"<<std::endl;
   		return 0;
  }else
  {
  	std::cout<<"Si homoMat4************************************"<<std::endl;
  }
  fs ["Homography"] >> transfMatrix;
  fs.release();
  cv::Size transfSize = cv::Size(646, 540); 

  cv::Mat divMat = cv::Mat::zeros(3,3, CV_64FC1); 
  divMat.at<double>(0,0) = (double)0.5; 
  divMat.at<double>(1,1) = (double)0.5; 
  divMat.at<double>(2,2) = (double)1.0; 
  transfMatrix = divMat *  transfMatrix;


  while (ros::ok()) {
    
    if (image)
    {
      cv::Mat transformed;
      std::vector<geometry_msgs::PoseStamped> poses;
      cv::warpPerspective(Image, transformed, transfMatrix, transfSize, cv::INTER_LINEAR, cv::BORDER_REPLICATE, cv::Scalar(127, 127, 127) );
      cv::Mat img = lines.segmentationLines(transformed, poses);
     // cv::Mat img = lines.segmentationLines(Image, poses);
      //cv::Mat line = lines.linesToPoints(img);
      //cv::waitKey(1);
      
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