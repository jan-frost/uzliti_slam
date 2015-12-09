# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(graph_slam)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -msse2 -msse3 -mssse3 -fpermissive -O2")

IF(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(THIS_PACKAGE_ROS_DEPS graph_slam_common graph_optimization map_projection place_recognition transformation_estimation image_geometry tf message_filters)

find_package(catkin REQUIRED COMPONENTS ${THIS_PACKAGE_ROS_DEPS})

generate_dynamic_reconfigure_options(cfg/GraphSlam.cfg)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
    DEPENDS 
    CATKIN_DEPENDS ${THIS_PACKAGE_ROS_DEPS}
    INCLUDE_DIRS
)

# Basic graph slam algorithm.
add_executable(graph_slam_node
    src/graph_slam_node.cpp
)
add_dependencies(graph_slam_node ${PROJECT_NAME}_gencfg)
target_link_libraries(graph_slam_node ${catkin_LIBRARIES})
