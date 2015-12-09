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

#ifndef SLAM_EDGE_H
#define SLAM_EDGE_H

#include <vector>
#include <Eigen/Dense>
#include <graph_slam_msgs/Edge.h>

class SlamEdgeWithPriority
{
public:
    SlamEdgeWithPriority(std::string id, double weight) {
        id_ = id;
        weight_ = weight;
    }

    bool operator<(const SlamEdgeWithPriority& other) const
    // such behavior is needed to use this type in std::priority queue
    // where top element is always max element and wee need to retrieve
    // edge with the lowest weight
    {
        return weight_ >= other.weight_;
    }

    std::string id_;
    double weight_;
};

class SlamEdge
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SlamEdge();

    SlamEdge(std::string id,
             std::string from_id,
             std::string to_id,
             Eigen::Isometry3d transform,
             Eigen::MatrixXd information,
             unsigned char type,
             std::string sensor_from,
             std::string sensor_to,
             ros::Duration diff_time = ros::Duration(1),
             double error = 0.,
             double age = 0.);

    void init(std::string id,
              std::string from_id,
              std::string to_id,
              Eigen::Isometry3d transform,
              Eigen::MatrixXd information,
              unsigned char type,
              std::string sensor_from,
              std::string sensor_to,
              ros::Duration diff_time = ros::Duration(1),
              double error = 0.,
              double age = 0.);

    std::string id_;
    std::string id_from_;
    std::string id_to_;
    Eigen::Isometry3d transform_;
    Eigen::Isometry3d displacement_from_;
    Eigen::Isometry3d displacement_to_;
    Eigen::MatrixXd information_;
    unsigned char type_;
    std::string sensor_from_;
    std::string sensor_to_;
    double age_;
    double error_;
    double matching_score_;
    bool valid_;
    ros::Duration diff_time_;
};

#endif // GRAPHLINK_H
