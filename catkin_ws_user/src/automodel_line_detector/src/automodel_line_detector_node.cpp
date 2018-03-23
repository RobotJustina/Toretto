#include "ros/ros.h"
#include <iostream>
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>

#include "automodel_line_detector/AutomodelLineDetector.h"

using namespace automodel;

int main(int argc, char **argv)
{
  //initialize node
  ros::init(argc, argv, "automodel_line_detector");

  // node handler
  ros::NodeHandle n;

  std::string path(argv[2]);
  std::cout<<"[Matrix location]: "<<path<<std::endl;

  cv::Mat transfMatrix;
  cv::Size transfSize;
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

  AutomodelLineDetector lineDetector(n, transfMatrix, transfSize);


  return 0;
}
