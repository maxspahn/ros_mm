cmake_minimum_required(VERSION 2.8.3)
project(mobile_manipulator)

find_package(catkin REQUIRED
  roscpp
  rospy
  sensor_msgs
  nav_msgs
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_executable(odom_node src/odom_pub.cpp)
target_link_libraries(odom_node
  ${catkin_LIBRARIES}
)
