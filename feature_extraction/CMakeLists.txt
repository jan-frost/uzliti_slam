# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(feature_extraction)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++0x -Wall -msse2 -msse3 -mssse3 -msse4.2") # -Wextra

IF(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo) 
endif()

set(THIS_PACKAGE_ROS_DEPS roslib roscpp std_msgs sensor_msgs cv_bridge image_transport pcl_ros tf dynamic_reconfigure image_geometry nav_msgs graph_slam_msgs nodelet graph_slam_tools)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS ${THIS_PACKAGE_ROS_DEPS} cmake_modules)

generate_dynamic_reconfigure_options(cfg/FeatureExtraction.cfg)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

catkin_package(
    DEPENDS eigen
    CATKIN_DEPENDS ${THIS_PACKAGE_ROS_DEPS}
    INCLUDE_DIRS
    LIBRARIES
)

add_library(feature_extraction_service
    src/feature_extraction_service_node.cpp
    src/feature_extraction_core.cpp
)
add_dependencies(feature_extraction_service ${PROJECT_NAME}_gencfg)
target_link_libraries(feature_extraction_service  ${catkin_LIBRARIES})

