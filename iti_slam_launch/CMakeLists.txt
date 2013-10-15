cmake_minimum_required(VERSION 2.8.3)
project(iti_slam_launch)

IF(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo) 
endif()

set(THIS_PACKAGE_ROS_DEPS feature_extraction graph_slam graph_slam_msgs graph_slam_tools robot_pose_ekf)

find_package(catkin REQUIRED COMPONENTS)

catkin_package(
#    DEPENDS 
    CATKIN_DEPENDS ${THIS_PACKAGE_ROS_DEPS}
#    INCLUDE_DIRS # TODO include
#    LIBRARIES # TODO
)
