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

#ifndef CONVERSIONS_H
#define CONVERSIONS_H

#include <graph_slam_msgs/Graph.h>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <graph_slam_tools/graph/slam_graph.h>
#include <graph_slam_tools/graph/sensor_data.h>
#include <g2o/types/slam3d/isometry3d_mappings.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/transform_listener.h>

class Conversions {
public:

    static graph_slam_msgs::Graph toMsg(SlamGraph &graph, bool add_sensor_data = true);

    /*!
    * \brief Conversion method for nodes poses
    *
    * This method converts a pose message with covariance to a 3D affine transformation.
    * \param poseMsg    ROS pose message with covariance
    * \return Eigen affine transformation
    */
    static void fromMsg(const geometry_msgs::PoseWithCovariance &poseMsg, Eigen::Isometry3d &pose, Eigen::MatrixXd &covariance);

    /*!
    * \brief Conversion method for nodes poses
    *
    * This method converts a pose message to a 3D affine transformation.
    * \param poseMsg    ROS pose message
    * \return Eigen affine transformation
    */
    static Eigen::Isometry3d fromMsg(const geometry_msgs::Pose &poseMsg);

    /*!
    * \brief Conversion method for SLAM nodes
    *
    * This method converts the features of a SLAM node to a feature message.
    * \param node   SLAM node object
    * \return feature message
    */
    static std::vector<graph_slam_msgs::SensorData> toMsg(const std::vector<SensorDataPtr> &sensor_data);
    static std::vector<graph_slam_msgs::SensorData> toMsg(const std::vector<SensorDataPtr> &sensor_data, int sensor_data_type);

    static void fromMsg(const std::vector<graph_slam_msgs::SensorData> &sensor_data, cv::Mat &features);
    static SensorDataPtr fromMsg(const graph_slam_msgs::SensorData &sensor_data_msg);
    static std::vector<SensorDataPtr> fromMsg(const graph_slam_msgs::SensorDataArray &sensor_data_msg);

    /*!
    * \brief Conversion method for node poses
    *
    * This method converts a node pose and its covariance to a ROS pose message.
    * \param pose   pose as Eigen affine transformation
    * \param Sigma  corresponding covariance matrix as Eigen matrix of size 6x6
    * \return ROS pose message with covariance
    */
	static geometry_msgs::PoseWithCovariance toMsg(const Eigen::Isometry3d &pose, const Eigen::MatrixXd &Sigma);

    /*!
    * \brief Conversion method for node poses
    *
    * This method converts a node pose to a ROS pose message.
    * \param pose   pose as Eigen affine transformation
    * \return ROS pose message
    */
	static geometry_msgs::Pose toMsg(const Eigen::Isometry3d &pose);

    /*!
    * \brief Conversion method for features
    *
    * This method reads the feature positions from a feature message.
    * \param featureMsg    feature message
    * \return 3D position matrix (3xN)
    */
    static Eigen::MatrixXd featureMsgToFeaturePositions(const graph_slam_msgs::Features &featureMsg);

    static SlamEdge fromMsg(const graph_slam_msgs::Edge &edge_msg);
    static graph_slam_msgs::Edge toMsg(const SlamEdge &edge);

    static SlamNode fromMsg(const graph_slam_msgs::Node &node_msg, bool copy_sensor_data = true);
    static graph_slam_msgs::Node toMsg(const SlamNode &node, bool copy_sensor_data = true);

    static Eigen::Vector3d fromMsg(const geometry_msgs::Point &point_msg);
    static geometry_msgs::Point toMsg(const Eigen::Vector3d &point);

    static graph_slam_msgs::SensorTransform toMsg(const std::string &name, const Eigen::Isometry3d transform);

    static pcl::PointCloud<pcl::PointXYZRGBA>::Ptr toPointCloudColor(const Eigen::Isometry3d &T, DepthImageDataPtr depth_data);

    static bool getTransform(tf::TransformListener &tfl, Eigen::Isometry3d &T, std::string from, std::string to, ros::Time stamp);
};

#endif
