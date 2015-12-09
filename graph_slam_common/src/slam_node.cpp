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

#include <graph_slam_common/slam_node.h>

SlamNode::SlamNode()
{
    id_ = "";
    pose_ = Eigen::Isometry3d::Identity();
    sub_pose_ = Eigen::Isometry3d::Identity();

    fixed_ = false;
    optimized_ = false;
    uncertainty_ = 0.;
    parent_ = -1;
    parent_transform_ = Eigen::Isometry3d::Identity();

    visited_ = false;
    distance_ = 0.;
    merge_count_ = 1;
}

SlamNode::SlamNode(ros::Time stamp, std::string id, Eigen::Isometry3d odom_pose, Eigen::Isometry3d map_pose)
{
    stamps_.push_back(stamp);
    this->id_ = id;
    this->pose_ = map_pose;
    this->sub_pose_ = odom_pose;

    fixed_ = false;
    optimized_ = false;
    uncertainty_ = 0.;
    parent_ = -1;
    parent_transform_ = Eigen::Isometry3d::Identity();

    visited_ = false;
    distance_ = 0.;
    merge_count_ = 1;
}

void SlamNode::addSensorData(const SensorDataPtr &data)
{
    this->sensor_data_.push_back(data);
}

void SlamNode::addSensorData(const std::vector<SensorDataPtr> &data)
{
    this->sensor_data_.insert(this->sensor_data_.end(), data.begin(), data.end());
}

bool SlamNode::deleteEdge(std::string edge_id)
{
    for (auto edge : edges_) {
        if (edge_id == edge) {
            edges_.erase(edge);
            return true;
        }
    }
    return false;
}
