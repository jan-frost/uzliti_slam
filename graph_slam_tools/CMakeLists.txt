# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(graph_slam_tools)

IF(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++0x -Wall -msse2 -msse3 -mssse3 -fpermissive") # -Wextra

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake/Modules "/usr/share/kde4/apps/cmake/modules/")

set(THIS_PACKAGE_ROS_DEPS roslib roscpp std_msgs geometry_msgs visualization_msgs image_geometry graph_slam_msgs tf pcl_ros cv_bridge dynamic_reconfigure occupancy_grid_utils)

find_package(catkin REQUIRED COMPONENTS ${THIS_PACKAGE_ROS_DEPS} cmake_modules)

generate_dynamic_reconfigure_options(
    cfg/FeatureLinkEstimation.cfg
    cfg/OccupancyGridProjector.cfg
    cfg/PlaceRecognizer.cfg
    cfg/GraphOptimizer.cfg
)

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
  thirdparty/include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
    DEPENDS eigen
    CATKIN_DEPENDS ${THIS_PACKAGE_ROS_DEPS}
    INCLUDE_DIRS include thirdparty/include ${G2O_INCLUDE_DIRS} ${CSPARSE_INCLUDE_DIR} ${CHOLMOD_INCLUDE_DIR}
    LIBRARIES ${PROJECT_NAME}
)

#common commands for building c++ executables and libraries
add_library(${PROJECT_NAME}
    src/conversions.cpp

    src/graph/sensor_data.cpp
    src/graph/slam_edge.cpp
    src/graph/slam_node.cpp
    src/graph/slam_graph.cpp

    src/optimization/g2o_optimizer.cpp

    src/pr/binary_gist_recognizer.cpp

    src/projection/graph_grid_mapper.cpp
    src/projection/occupancy_grid_projector.cpp

    src/transformation/feature_transformation_estimator.cpp

    thirdparty/src/lsh.cpp
    thirdparty/src/epnp.cpp
)
add_dependencies(${PROJECT_NAME} graph_slam_tools_gencfg)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES}
   ${CHOLMOD_LIBRARIES}
#   ${CSPARSE_LIBRARY}
   ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY} ${G2O_SOLVER_COLMOD} ${G2O_SOLVER_CSPARSE} ${G2O_SOLVER_CSPARSE_EXTENSION} ${G2O_TYPES_SLAM3D} ${G2O_TYPES_SCLAM2D}
)
