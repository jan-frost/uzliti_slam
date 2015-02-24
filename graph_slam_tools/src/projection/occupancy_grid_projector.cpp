// Copyright (c) 2014, Institute of Computer Engineering (ITI), Universität zu Lübeck
// Jan Frost, Jan Helge Klüssendorff
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice, this
//    list of conditions and the following disclaimer in the documentation and/or
//    other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its contributors may
//    be used to endorse or promote products derived from this software without
//    specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
// IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
// NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

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
