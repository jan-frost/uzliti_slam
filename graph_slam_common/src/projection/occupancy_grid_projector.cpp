#include <graph_slam_tools/projection/occupancy_grid_projector.h>
#include <nav_msgs/OccupancyGrid.h>
#include <fstream>
#include <chrono>
#include <graph_slam_tools/conversions.h>

using std::chrono::duration_cast;
using std::chrono::microseconds;
using std::chrono::milliseconds;
using std::chrono::minutes;
using std::chrono::system_clock;

OccupancyGridProjector::OccupancyGridProjector(ros::NodeHandle nh) :
    MapProjector(),
    tfl_(nh)
{
    map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("/map", 1);
    last_tf_map_odom_ = Eigen::Isometry3d::Identity();
    odom_frame_ = "/odom";
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
    if (map_pub_.getNumSubscribers() == 0) {
        return;
    }

    Eigen::Isometry3d tf_map_odom = Eigen::Isometry3d::Identity();
    Conversions::getTransform(tfl_, tf_map_odom, "/map", odom_frame_, ros::Time(0));
    Eigen::Isometry3d diff_transform = last_tf_map_odom_.inverse() * tf_map_odom;
    last_tf_map_odom_ = tf_map_odom;

    auto runtime_start = system_clock::now();
    ROS_DEBUG("project graph to occupancy grid");
    nav_msgs::OccupancyGrid map;
    if (config_.recompute_laserscans) {
        if (mapper.convertDepthImages2Map(graph, map, map_pub_)) {
            map.header.stamp = ros::Time::now();
            map_pub_.publish(map);
        }
    } else {
        if (mapper.convertLaserScans2Map(graph, map, map_pub_, diff_transform)) {
            map.header.stamp = ros::Time::now();
            map_pub_.publish(map);
        }
    }
    double runtime = (0.001 * duration_cast<microseconds>(system_clock::now() - runtime_start).count());
    std::ofstream of;
    of.open("/home/jan/Data/SLAM/runtimes/occupancy_grid_creation.txt", std::ios_base::out | std::ios_base::app);
    of << std::setprecision(16) << ros::Time::now() << " " << runtime << std::endl;
}
