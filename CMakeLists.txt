cmake_minimum_required(VERSION 3.0.2)
project(Sensor_fusion)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
   roscpp
   pcl_conversions
   pcl_ros
   std_msgs
   message_generation
   rospy
   cv_bridge
   image_transport
   OpenCV
)

add_message_files(
  FILES
  obj_msg.msg
  LiDAR_BB.msg
  LiDAR_BB_arr.msg
  Camera_BB.msg
  Camera_BB_arr.msg
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_generation roscpp std_msgs rospy cv_bridge image_transport message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(Data_RCVD src/Data_Receive.cpp)
target_link_libraries(Data_RCVD ${catkin_LIBRARIES})
