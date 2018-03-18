cmake_minimum_required(VERSION 2.8.3)
project(cross)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  image_transport 
  cv_bridge
 
)


find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(cross_node src/cross_node.cpp src/CrossDetector.h src/CrossDetector.cpp)

target_link_libraries(cross_node ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

