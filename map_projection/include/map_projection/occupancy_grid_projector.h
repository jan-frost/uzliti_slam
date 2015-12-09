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

#ifndef OCCUPANCY_GRID_PROJECTOR_H
#define OCCUPANCY_GRID_PROJECTOR_H

#include <map_projection/map_projector.h>

#include <dynamic_reconfigure/server.h>
#include <map_projection/OccupancyGridProjectorConfig.h>
#include <map_projection/graph_grid_mapper.h>
#include <ros/ros.h>

class OccupancyGridProjector : public MapProjector
{
public:
    OccupancyGridProjector(ros::NodeHandle nh);
    ~OccupancyGridProjector();
    void setConfig(map_projection::OccupancyGridProjectorConfig config);

    std::string odom_frame_;

protected:
    void projectImpl(SlamGraph &graph);

    ros::Publisher map_pub_;    /**< Publisher for the occupancy grid map. */
    GraphGridMapper mapper;     /**< An auxiliary object to handle the generation of an occupancy grid map */
    map_projection::OccupancyGridProjectorConfig config_;
    tf::TransformListener tfl_;
    Eigen::Isometry3d last_tf_map_odom_;
};

#endif
