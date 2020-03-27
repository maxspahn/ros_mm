cmake_minimum_required(VERSION 2.8.3)
project(mobile_navigation)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
  actionlib
  actionlib_msgs
  move_base_msgs
)

find_package(Eigen3 PATHS /home/mspahn/develop/eigen-3.3.7/cmake)
find_package(ompl)

## Declare a catkin package
catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OMPL_INCLUDE_DIRS})

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})

add_executable(fibon src/fibonacci_client.cpp)
target_link_libraries(fibon ${catkin_LIBRARIES})

add_executable(goalClient src/sendGoalClient.cpp)
target_link_libraries(goalClient ${catkin_LIBRARIES} ${OMPL_LIBRARIES})
