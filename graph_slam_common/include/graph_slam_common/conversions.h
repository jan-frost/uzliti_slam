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

#ifndef CONVERSIONS_H
#define CONVERSIONS_H

#include <graph_slam_msgs/Graph.h>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <g2o/types/slam3d/isometry3d_mappings.h>
#include <graph_slam_common/slam_graph.h>
#include <graph_slam_common/sensor_data.h>
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
    static void toMsg(const g2o::Vector6d &point, boost::array<double, 6> &point_msg);
    static g2o::Vector6d fromMsg(const boost::array<double, 6> &point_msg);

    static graph_slam_msgs::SensorTransform toMsg(const std::string &name, const Eigen::Isometry3d transform);

    static pcl::PointCloud<pcl::PointXYZRGBA>::Ptr toPointCloudColor(const Eigen::Isometry3d &T, DepthImageDataPtr depth_data);
    static pcl::PointCloud<pcl::PointXYZ>::Ptr toPointCloud(const Eigen::Isometry3d &T, const image_geometry::PinholeCameraModel &cam, const cv::Mat &depth_img);

    static bool getTransform(tf::TransformListener &tfl, Eigen::Isometry3d &T, std::string from, std::string to, ros::Time stamp);

    /*!
     *  \author Philip G. Lee <rocketman768@gmail.com>
     *  Write \b mat into \b filename
     *  in uncompressed .mat format (Level 5 MATLAB) for Matlab.
     *  The variable name in matlab will be \b varName. If
     *  \b bgr2rgb is true and there are 3 channels, swaps 1st and 3rd
     *  channels in the output. This is needed because OpenCV matrices
     *  are bgr, while Matlab is rgb. This has been tested to work with
     *  3-channel single-precision floating point matrices, and I hope
     *  it works on other types/channels, but not exactly sure.
     *  Documentation at <http://www.mathworks.com/help/pdf_doc/matlab/matfile_format.pdf>
     */
    static void writeMat( cv::Mat const& mat, const char* filename, const char* varName = "A", bool bgr2rgb = true );

};

#endif
