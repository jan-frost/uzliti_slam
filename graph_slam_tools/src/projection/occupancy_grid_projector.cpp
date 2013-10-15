#include <graph_slam_tools/projection/occupancy_grid_projector.h>
#include <nav_msgs/OccupancyGrid.h>

OccupancyGridProjector::OccupancyGridProjector(ros::NodeHandle nh) :
    MapProjector()
{
    map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("/map", 1);
}

OccupancyGridProjector::~OccupancyGridProjector()
{
}

void OccupancyGridProjector::setConfig(graph_slam_tools::OccupancyGridProjectorConfig config)
{
    this->config_ = config;
    mapper.setConfig(config);
}

void OccupancyGridProjector::projectImpl(SlamGraph &graph)
{
    ROS_DEBUG("project graph to occupancy grid");
    nav_msgs::OccupancyGrid map;
    if (config_.recompute_laserscans) {
        if (mapper.convertDepthImages2Map(graph, map, map_pub_)) {
            map.header.stamp = ros::Time::now();
            map_pub_.publish(map);
        }
    } else {
        if (mapper.convertLaserScans2Map(graph, map, map_pub_)) {
            map.header.stamp = ros::Time::now();
            map_pub_.publish(map);
        }
    }
}
