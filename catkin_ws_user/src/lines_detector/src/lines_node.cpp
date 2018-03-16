#include "ros/ros.h"
#include "LinesDetector.h"


cv::Mat Image;
bool image = false;
cv::Mat persp; 
cv::Mat transf; 



void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    //try{
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
     //}catch ( cv_bridge::Exception& e)
     //{
     //ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
     //}
} 

int main(int argc, char** args)
{
  ros::init(argc, args, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport its(nh);
  image_transport::Subscriber sub = its.subscribe("app/camera/rgb/image_raw",1,imageCallback);

  image_transport::ImageTransport itp(nh);
  sensor_msgs::ImagePtr msg ;


  ros::Publisher rLine_pub= nh.advertise<std_msgs::Float32MultiArray>("rightLine",1000);
  ros::Publisher lLine_pub= nh.advertise<std_msgs::Float32MultiArray>("leftLine",1000);

  ros::Rate loop_rate(60);
  std::string argumento(args[1]);
  std::string path(args[2]);
  std::cout<<"[Matrix location]: "<<path<<std::endl; 
  LinesDetector lines = LinesDetector(argumento == "debug",path);


  cv::Mat transfMatrix;
  cv::Size transfSize;
  if(argumento != "transform"){

    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
     		std::cout<<"No Matrix"<<std::endl;
     		return 0;
    }else
    {
    	std::cout<<"Si Matrix************************************"<<std::endl;
    }
    fs["Homography"] >> transfMatrix;
    
    fs["tSize"] >> transfSize; 
    fs.release();
  }

  while (ros::ok()) {
      
    if (image)
    {
      cv::Rect rect(0,240,640,240);
      cv::Mat resizeImage= Image(rect);

      //cv::resize(Image, resizeImage, cv::Size(resizeImage.rows,resizeImage.cols), )
      if (argumento == "transform")
      {
        lines.transformMatrix(resizeImage);
        return 0;
      }
      cv::Mat transformed;
  
      std_msgs::Float32MultiArray right;
      std_msgs::Float32MultiArray left;

      

      cv::warpPerspective(resizeImage, transformed, transfMatrix, transfSize, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(127, 127, 127) );
      //cv::imshow("transformed", transformed); 
      
      cv::Mat img = lines.segmentationLines(transformed, right,left);
      //cv::Mat img = lines.segmentationLines(resizeImage, right,left);
      //std::cout << "Time testing version"<< std::endl;
      // cv::Mat img = lines.segmentationLines(Image, poses);
      //cv::Mat line = lines.linesToPoints(img);
      
      if (argumento=="debug")
      {
        cv::imshow("image", Image); 
        cv::imshow("Resize image", resizeImage); 
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
      
      /*
      if(image)
      {
	  image = false;
	  std_msgs::Float32MultiArray right;
	  std_msgs::Float32MultiArray left;
	  lLine_pub.publish(left);
	  rLine_pub.publish(right);
	  std::cout << "Simple version"<< std::endl;
      }*/
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
