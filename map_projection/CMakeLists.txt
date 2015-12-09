# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(map_projection)

IF(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++0x -Wall -msse2 -msse3 -mssse3 -fpermissive -O3") # -Wextra

set(THIS_PACKAGE_ROS_DEPS graph_slam_common occupancy_grid_utils dynamic_reconfigure)

find_package(catkin REQUIRED COMPONENTS ${THIS_PACKAGE_ROS_DEPS})

generate_dynamic_reconfigure_options(
    cfg/OccupancyGridProjector.cfg
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
    src/map_projector.cpp
    src/occupancy_grid_projector.cpp
    src/graph_grid_mapper.cpp
)
add_dependencies(${PROJECT_NAME} map_projection_gencfg)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
