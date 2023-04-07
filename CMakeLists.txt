cmake_minimum_required(VERSION 3.0.2)
project(cam_projection)
set(CMAKE_CXX_STANDARD 14)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslaunch
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
  sensor_msgs
  pcl_conversions
  pcl_ros
  cv_bridge
  tf
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  roslaunch
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
  sensor_msgs
  pcl_conversions
  pcl_ros
  cv_bridge
  tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(project
  src/project.cpp 
)

target_link_libraries(project
        ${catkin_LIBRARIES}
)


add_dependencies(project
        ${catkin_EXPORTED_TARGETS}
)

