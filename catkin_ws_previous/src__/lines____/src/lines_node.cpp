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
  cv::setUseOptimized(true); 
  std::cout << "OPENCV OPTIMIZED = " <<  cv::useOptimized() << std::endl; 

  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport its(nh);
  image_transport::Subscriber sub = its.subscribe("app/camera/rgb/image_raw",1,imageCallback);

  //image_transport::ImageTransport itp(nh);
  //image_transport::Publisher pub = itp.advertise("lines",1);
  //sensor_msgs::ImagePtr msg ;

  ros::Publisher path_pub= nh.advertise<nav_msgs::Path>("center",1);
  ros::Publisher path_left= nh.advertise<nav_msgs::Path>("left",1);
  ros::Publisher path_right= nh.advertise<nav_msgs::Path>("right",1);

  ros::Rate loop_rate(100);
  
  LinesDetector lines = LinesDetector();
  nav_msgs::Path msg_center;
  nav_msgs::Path msg_left;
  nav_msgs::Path msg_right;

  bool DEBUG = false;
  lines.DebugMode = DEBUG; 

  cv::Mat matrixH;
  //cv::FileStorage fs("/root/catkin_ws_user/homoMat4.yaml", cv::FileStorage::READ);
  // cv::FileStorage fs("/home/robocup/homoMat4.yaml", cv::FileStorage::READ);
  // fs ["Homography"] >> matrixH;
  // fs.release(); 
  
  // cv::Mat divMat = cv::Mat::zeros(3,3, CV_64FC1); 
  // divMat.at<double>(0,0) = (double)0.5; 
  // divMat.at<double>(1,1) = (double)0.5; 
  // divMat.at<double>(2,2) = (double)1.0; 
    
  // matrixH = divMat * matrixH ; 

  // cv::Mat kernel = getStructuringElement(cv::MORPH_CROSS,cv::Size(5,5));
  std::cout<<"Start Lines"<<std::endl;

  while (ros::ok()) {
    
    if (image)
    {
      std::vector<geometry_msgs::PoseStamped> poses;
      std::vector<geometry_msgs::PoseStamped> lf;
      std::vector<geometry_msgs::PoseStamped> rg;
      //lines.imageSave(Image);
      //lines.homography(Image);
      
      if( DEBUG )
          cv::imshow("Image", Image );

      cv::Mat transf; 
      lines.TrasformImage(Image, transf);
      lines.GetLines(transf, poses, lf, rg);
       
      //cv::Mat img = lines.segmentationLines(lines.homography(Image, matrixH),kernel, poses, lf ,rg);
      //cv::Mat img = lines.segmentationLines(Image, poses);
      //cv::Mat line = lines.linesToPoints(img);
      
      
      //msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", img).toImageMsg();
      //pub.publish(msg);
      msg_center.poses = poses;
      msg_left.poses = lf;
      msg_right.poses = rg;
      path_pub.publish(msg_center);
      path_right.publish(msg_right);
      path_left.publish(msg_left);

   //    if (poses.size() > 10)
   //    {
   //    	msg_path.poses = poses;
   //      path_pub.publish(msg_path);
   //    }
   //    else{
   //    	poses.clear();
   //    	msg_path.poses = poses;
   //    	path_pub.publish(msg_path);
	  // }
      image= false;
	
      if( DEBUG )
          cv::waitKey(1);

    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}