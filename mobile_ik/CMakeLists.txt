cmake_minimum_required(VERSION 3.0.2)
project(mobile_ik)

find_package(catkin REQUIRED COMPONENTS
  mobile_manipulator
  trac_ik_lib
  tf
  mm_msgs
)

find_package(Boost REQUIRED COMPONENTS date_time)
find_package(orocos_kdl REQUIRED)


catkin_package(
  CATKIN_DEPENDS
    trac_ik_lib
    mm_msgs
  DEPENDS
    Boost
    orocos_kdl
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)

add_executable(ik_service src/ik_service.cpp)
target_link_libraries(ik_service
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)

