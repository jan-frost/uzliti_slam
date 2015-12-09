# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(graph_slam_msgs)

IF(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo) 
endif()

# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS message_generation geometry_msgs sensor_msgs)

#uncomment if you have defined messages
add_message_files(FILES
  DepthImage.msg
  Edge.msg
  Graph.msg
  GraphMeta.msg
  GraphReceived.msg
  Neighbor.msg
  NeighborArray.msg
  Node.msg
  NodePair.msg
  SensorTransform.msg
  Feature.msg
  Features.msg
  MapRequest.msg
  Scope.msg
  SensorData.msg
  SensorDataArray.msg
  SensorRequest.msg
)
#uncomment if you have defined services
add_service_files(FILES
  EmptyService.srv
  MapRequestService.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs geometry_msgs sensor_msgs)

catkin_package(
    CATKIN_DEPENDS message_runtime geometry_msgs sensor_msgs
)

