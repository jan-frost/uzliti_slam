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

#include <graph_slam_tools/graph/slam_node.h>

SlamNode::SlamNode()
{
    stamp_ = ros::Time::now();
    id_ = "";
    pose_ = Eigen::Isometry3d::Identity();
    sub_pose_ = Eigen::Isometry3d::Identity();

    fixed_ = false;
    is_loop_ = true;
    is_border_ = false;
    is_in_scope_ = true;
    is_in_last_scope_ = false;
    visited_ = false;
    distance_ = 0.;
    parent_ = -1;
    parent_transform_ = Eigen::Isometry3d::Identity();
}

SlamNode::SlamNode(ros::Time stamp, std::string id, Eigen::Isometry3d odom_pose, Eigen::Isometry3d map_pose)
{
    this->stamp_ = stamp;
    this->id_ = id;
    this->pose_ = map_pose;
    this->sub_pose_ = odom_pose;

    fixed_ = false;
    is_loop_ = true;
    is_border_ = false;
    is_in_scope_ = true;
    is_in_last_scope_ = false;
    visited_ = false;
    distance_ = 0.;
    parent_ = -1;
    parent_transform_ = Eigen::Isometry3d::Identity();
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
