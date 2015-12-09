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

#ifndef SLAM_NODE_H
#define SLAM_NODE_H

#include <ros/ros.h>
#include <vector>
#include <map>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <graph_slam_common/slam_edge.h>
#include <graph_slam_common/sensor_data.h>

class SlamNodeWithPriority
{
public:
    SlamNodeWithPriority(std::string id, double weight) {
        id_ = id;
        weight_ = weight;
    }

    bool operator<(const SlamNodeWithPriority& other) const
    // such behavior is needed to use this type in std::priority queue
    // where top element is always max element and wee need to retrieve
    // edge with the lowest weight
    {
        return weight_ > other.weight_;
    }

    std::string id_;
    double weight_;
};

//* Class to hold all information of a SLAM node.
/**
* The SlamNode class stores all information of a node in the SLAM graph.
* This includes a identification number, the 3D pose, feature descriptors
* and positions, and the depth image as well as links to other nodes.
*/
class SlamNode {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /*!
    * \brief Default constructor.
    *
    * The default contructor initializes an empty node.
    */
    SlamNode();

    /*!
    * \brief Constructor to be used for node creation.
    *
    * The main constructor, which initializes the SLAM node with the
    * data given at the time of creation.
    * \param stamp              timestamp at the time of creation
    * \param id                 identification number of the node
    * \param pose               3D pose of the node
    */
    SlamNode(ros::Time stamp_, std::string id_, Eigen::Isometry3d odom_pose, Eigen::Isometry3d map_pose);

    /*!
    * \brief Adds sensor data to the node.
    *
    * \param data sensor data object
    */
    void addSensorData(const SensorDataPtr &data);
    void addSensorData(const std::vector<SensorDataPtr> &data);

    bool deleteEdge(std::string edge_id);

    std::string id_;                                /**< identification number of the node */
    std::vector<ros::Time> stamps_;                        /**< timestamp at the time of creation */
    Eigen::Isometry3d sub_pose_;
    Eigen::Isometry3d pose_;                   /**< 3D pose of the node */
    std::vector<SensorDataPtr> sensor_data_;
    double merge_count_;

    // Graph theory related variables.
    bool fixed_;
    bool optimized_;
    double uncertainty_;
    std::string parent_;
    Eigen::Isometry3d parent_transform_;

    std::set<std::string> edges_;

    // Search helpers.
    bool visited_;
    double distance_;

};

#endif
