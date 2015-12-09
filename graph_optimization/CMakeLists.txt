# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(graph_optimization)

IF(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
#  set(CMAKE_BUILD_TYPE Debug)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++0x -Wall -msse2 -msse3 -mssse3 -fpermissive -O3") # -Wextra

set(THIS_PACKAGE_ROS_DEPS graph_slam_common transformation_estimation dynamic_reconfigure)

find_package(catkin REQUIRED COMPONENTS ${THIS_PACKAGE_ROS_DEPS})

generate_dynamic_reconfigure_options(
    cfg/GraphOptimizer.cfg
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
    DEPENDS 
    CATKIN_DEPENDS ${THIS_PACKAGE_ROS_DEPS}
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)

add_library(${PROJECT_NAME}
    src/graph_optimizer.cpp
    src/g2o_optimizer.cpp
    src/sensor_transform_optimizer.cpp
    src/edge_se3_sensor_calib.cpp
    src/edge_se3_odom_differential_calib.cpp
    src/edge_rot_trans_sensor_calib.cpp
)
add_dependencies(${PROJECT_NAME} graph_optimization_gencfg)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
