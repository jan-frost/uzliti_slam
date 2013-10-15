# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(graph_slam)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++0x -Wall -msse2 -msse3 -mssse3")

IF(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(THIS_PACKAGE_ROS_DEPS roslib roscpp visualization_msgs nav_msgs graph_slam_tools)

find_package(catkin REQUIRED COMPONENTS ${THIS_PACKAGE_ROS_DEPS} cmake_modules)

generate_dynamic_reconfigure_options(cfg/GraphSlam.cfg)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})

add_definitions(${EIGEN_DEFINITIONS})

catkin_package(
    DEPENDS eigen
    CATKIN_DEPENDS ${THIS_PACKAGE_ROS_DEPS}
    INCLUDE_DIRS
)

# Basic graph slam algorithm.
add_executable(graph_slam_node
    src/graph_slam_node.cpp
)
add_dependencies(graph_slam_node ${PROJECT_NAME}_gencfg)
target_link_libraries(graph_slam_node ${catkin_LIBRARIES})
