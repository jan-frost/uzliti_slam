# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(iti_slam_launch)

IF(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo) 
endif()

# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS )

catkin_package(
#    DEPENDS 
#    CATKIN_DEPENDS feature_extraction graph_slam graph_slam_tools
#    INCLUDE_DIRS # TODO include
#    LIBRARIES # TODO
)
