cmake_minimum_required(VERSION 2.8.3)
project(lane_detection)


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

add_executable(lane_detection_node src/lane_detection.cpp src/lane_detection.h src/lane_detection_node.cpp)

target_link_libraries(lane_detection_node ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
