# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(feature_extraction)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++0x -Wall -msse2 -msse3 -mssse3 -msse4.2") # -Wextra

IF(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo) 
endif()

set(THIS_PACKAGE_ROS_DEPS graph_slam_common map_projection image_geometry nodelet)

find_package(catkin REQUIRED COMPONENTS ${THIS_PACKAGE_ROS_DEPS} cmake_modules)

generate_dynamic_reconfigure_options(cfg/FeatureExtraction.cfg)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  external
)

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

include_directories(${PROJECT_SOURCE_DIR}/external/DepthMapRefinement)
set(WJBF_SOURCE_FILES
  ${PROJECT_SOURCE_DIR}/external/DepthMapRefinement/jointBilateralFilter.cpp
  ${PROJECT_SOURCE_DIR}/external/DepthMapRefinement/jointNearest.cpp
  ${PROJECT_SOURCE_DIR}/external/DepthMapRefinement/split.cpp)

catkin_package(
    DEPENDS eigen
    CATKIN_DEPENDS ${THIS_PACKAGE_ROS_DEPS}
    INCLUDE_DIRS include external/aorb
    LIBRARIES feature_extraction
)

add_library(feature_extraction
    src/feature_extraction_core.cpp
    ${WJBF_SOURCE_FILES}
    external/aorb/aorb.cpp
)

add_library(feature_extraction_service
    src/feature_extraction_service_node.cpp
)
add_dependencies(feature_extraction_service ${PROJECT_NAME}_gencfg)
target_link_libraries(feature_extraction_service ${catkin_LIBRARIES} feature_extraction)

add_executable(depth_image_to_laserscan
    src/depth_image_to_laserscan.cpp
)
add_dependencies(depth_image_to_laserscan ${PROJECT_NAME}_gencfg)
target_link_libraries(depth_image_to_laserscan ${catkin_LIBRARIES} feature_extraction)


