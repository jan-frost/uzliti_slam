// Copyright (c) 2015, Institute of Computer Engineering (ITI), Universität zu Lübeck
// Jan Frost
// All rights reserved.
//
// This file is part of the uzliti_slam ROS package.
//
// uzliti_slam is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// uzliti_slam is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with uzliti_slam.  If not, see <http://www.gnu.org/licenses/>.

#include <map_projection/occupancy_grid_projector.h>

#include <nav_msgs/OccupancyGrid.h>
#include <fstream>
#include <chrono>
#include <graph_slam_common/conversions.h>

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

void OccupancyGridProjector::setConfig(map_projection::OccupancyGridProjectorConfig config)
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
}
