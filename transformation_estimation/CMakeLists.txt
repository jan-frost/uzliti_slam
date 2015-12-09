# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(transformation_estimation)

IF(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++0x -Wall -msse2 -msse3 -mssse3 -fpermissive -O3") # -Wextra

set(THIS_PACKAGE_ROS_DEPS graph_slam_common dynamic_reconfigure csm)

find_package(catkin REQUIRED COMPONENTS ${THIS_PACKAGE_ROS_DEPS})

generate_dynamic_reconfigure_options(
    cfg/FeatureLinkEstimation.cfg
)

include_directories(
  include
  external
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
    DEPENDS 
    CATKIN_DEPENDS ${THIS_PACKAGE_ROS_DEPS}
    INCLUDE_DIRS include external
    LIBRARIES ${PROJECT_NAME} 
)

#common commands for building c++ executables and libraries
add_library(${PROJECT_NAME}
    src/transformation_filter.cpp
    src/transformation_estimator.cpp
    src/feature_transformation_estimator.cpp
    src/cloud_transformation_estimator.cpp
    src/laser_transformation_estimator.cpp

    external/gicp6d/gicp6d.cpp
)
add_dependencies(${PROJECT_NAME} transformation_estimation_gencfg)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
