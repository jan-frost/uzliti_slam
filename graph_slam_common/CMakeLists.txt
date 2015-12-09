# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(graph_slam_common)

IF(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++0x -Wall -msse2 -msse3 -mssse3 -fpermissive -O3") # -Wextra

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake/Modules)

set(THIS_PACKAGE_ROS_DEPS roslib roscpp geometry_msgs graph_slam_msgs cv_bridge warehouse_ros rosbag pcl_ros)
#std_msgs geometry_msgs visualization_msgs image_geometry graph_slam_msgs tf pcl_ros cv_bridge gps_common image_transport dynamic_reconfigure occupancy_grid_utils canonical_scan_matcher warehouse_ros rosbag)

find_package(catkin REQUIRED COMPONENTS ${THIS_PACKAGE_ROS_DEPS} cmake_modules)

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

find_package(G2O REQUIRED)
include_directories(
  ${G2O_INCLUDE_DIRS}
)

find_package(SuiteSparse REQUIRED)
find_package(CSparse)
include_directories(
  ${CSPARSE_INCLUDE_DIR}
  ${CHOLMOD_INCLUDE_DIR}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
    DEPENDS eigen
    CATKIN_DEPENDS ${THIS_PACKAGE_ROS_DEPS}
    INCLUDE_DIRS include ${G2O_INCLUDE_DIRS} ${CSPARSE_INCLUDE_DIR} ${CHOLMOD_INCLUDE_DIR}
    LIBRARIES ${PROJECT_NAME}
)

add_library(${PROJECT_NAME}
    src/conversions.cpp

    src/slam_graph.cpp
    src/slam_node.cpp
    src/slam_edge.cpp
    src/sensor_data.cpp

    src/slam_graph_storage.cpp
    src/rosbag_storage.cpp
    src/mongodb_storage.cpp
)
target_link_libraries(${PROJECT_NAME} 
  ${catkin_LIBRARIES}
  ${CHOLMOD_LIBRARIES}
  ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY} ${G2O_SOLVER_COLMOD} ${G2O_SOLVER_CSPARSE} ${G2O_SOLVER_CSPARSE_EXTENSION} ${G2O_TYPES_SLAM3D} ${G2O_TYPES_SCLAM2D}
)
