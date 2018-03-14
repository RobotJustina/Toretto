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

int main(int argc, char** args)
{
  ros::init(argc, args, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport its(nh);
  image_transport::Subscriber sub = its.subscribe("app/camera/rgb/image_raw",1,imageCallback);

  image_transport::ImageTransport itp(nh);
  sensor_msgs::ImagePtr msg ;

  //ros::Publisher path_pub= nh.advertise<nav_msgs::Path>("path",1000);
  ros::Publisher rLine_pub= nh.advertise<std_msgs::Float32MultiArray>("rightLine",1000);
  ros::Publisher lLine_pub= nh.advertise<std_msgs::Float32MultiArray>("leftLine",1000);

  ros::Rate loop_rate(5);
  std::string argumento(args[1]);
  LinesDetector lines = LinesDetector(argumento == "debug");
  nav_msgs::Path msg_path;


  
  cv::Mat transfMatrix;
  cv::FileStorage fs("/root/catkin_ws_user/Matrix.yaml", cv::FileStorage::READ);
  //cv::FileStorage fs("/home/haime/Toretto/catkin_ws_user/Matrix.yaml", cv::FileStorage::READ);
  if (!fs.isOpened())
  {
   		std::cout<<"No Matrix"<<std::endl;
   		return 0;
  }else
  {
  	std::cout<<"Si Matrix************************************"<<std::endl;
  }
  fs["Homography"] >> transfMatrix;
 
  cv::Size transfSize;
  fs["tSize"] >> transfSize; 
  fs.release();
  
  while (ros::ok()) {
    
    if (image)
    {

      if (argumento == "transform")
      {
        lines.transformMatrix(Image);
        return 0;
      }
      cv::Mat transformed;
  
      std_msgs::Float32MultiArray right;
      std_msgs::Float32MultiArray left;

      

      cv::warpPerspective(Image, transformed, transfMatrix, transfSize, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(127, 127, 127) );
      //cv::imshow("transformed", transformed); 
      
      cv::Mat img = lines.segmentationLines(transformed, right,left);
      // cv::Mat img = lines.segmentationLines(Image, poses);
      //cv::Mat line = lines.linesToPoints(img);
      
      if (argumento=="debug")
      {
        cv::imshow("image", Image); 
        cv::waitKey(1);
      }     
      
     
      if (right.data.size()>1)
      {
        //std::cout<<"[right]: ["<<right.data[0]<<" X] ["<<right.data[1]<<" Y] ["<<right.data[2]<<" C]"<<std::endl;
        rLine_pub.publish(right);
      }
      if (left.data.size()>1)
      {
        //std::cout<<"[left]: ["<<left.data[0]<<" X] ["<<left.data[1]<<" Y] ["<<left.data[2]<<" C]"<<std::endl;
        lLine_pub.publish(left);
      }
      image= false;

    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}