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

#ifndef SLAM_NODE_H
#define SLAM_NODE_H

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <map>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <graph_slam_tools/graph/slam_edge.h>
#include <graph_slam_tools/graph/sensor_data.h>

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
    ros::Time stamp_;                        /**< timestamp at the time of creation */
    Eigen::Isometry3d sub_pose_;
    Eigen::Isometry3d pose_;                   /**< 3D pose of the node */
    std::vector<SensorDataPtr> sensor_data_;

    // Graph theory related variables.
    bool fixed_;
    bool is_loop_;
    bool is_border_;
    bool is_in_scope_;
    bool is_in_last_scope_;
    bool is_new_node_;
    bool visited_;
    double distance_;
    std::string parent_;
    Eigen::Isometry3d parent_transform_;

    std::set<std::string> edges_;

};

#endif
