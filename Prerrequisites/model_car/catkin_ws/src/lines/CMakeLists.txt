cmake_minimum_required(VERSION 2.8.3)
project(lines)


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

add_executable(lines_node src/lines_node.cpp src/LinesDetector.h src/LinesDetector.cpp)

target_link_libraries(lines_node ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

