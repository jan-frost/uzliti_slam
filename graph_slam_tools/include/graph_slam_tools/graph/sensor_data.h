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

#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <opencv2/opencv.hpp>
#include <graph_slam_msgs/SensorDataArray.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/LaserScan.h>

class SensorData {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SensorData();
    SensorData(int type,
               ros::Time stamp,
               std::string sensor_frame);
    virtual ~SensorData();
    virtual graph_slam_msgs::SensorData toMsg();
    virtual void fromMsg(graph_slam_msgs::SensorData msg);

    int type_;
    ros::Time stamp_;
    std::string sensor_frame_;
    Eigen::Isometry3d displacement_;
};

class FeatureData : public SensorData {
public:
    FeatureData();
    FeatureData(ros::Time stamp,
                std::string sensor_frame,
                int feature_type,
                cv::Mat features,
                Eigen::MatrixXd feature_positions_2d,
                Eigen::MatrixXd feature_positions,
                std::vector<bool> valid_3d,
                image_geometry::PinholeCameraModel camera_model);
    virtual graph_slam_msgs::SensorData toMsg();
    virtual void fromMsg(graph_slam_msgs::SensorData msg);

    int feature_type_;                  /**< type of the feature descriptors as defined by the FeatureType message */
    cv::Mat features_;                  /**< feature descriptors */
    Eigen::MatrixXd feature_positions_2d_; /**< 2D positions for the features */
    Eigen::MatrixXd feature_positions_; /**< 3D positions for the features */
    std::vector<bool> valid_3d_; /**< 3D positions for the features */
    image_geometry::PinholeCameraModel camera_model_;
};

class DepthImageData : public SensorData {
public:
    DepthImageData();
    DepthImageData(ros::Time stamp,
                   std::string sensor_frame,
                   sensor_msgs::Image depth_image,
                   image_geometry::PinholeCameraModel camera_model);
    virtual graph_slam_msgs::SensorData toMsg();
    virtual void fromMsg(graph_slam_msgs::SensorData msg);

    sensor_msgs::Image depth_image_;
    sensor_msgs::Image color_image_;
    image_geometry::PinholeCameraModel camera_model_;
};

class BinaryGistData : public SensorData {
public:
    BinaryGistData();
    BinaryGistData(ros::Time stamp,
                   std::string sensor_frame,
                   cv::Mat descriptor);
    virtual graph_slam_msgs::SensorData toMsg();
    virtual void fromMsg(graph_slam_msgs::SensorData msg);

    cv::Mat descriptor_;
};

class LaserscanData : public SensorData {
public:
    LaserscanData();
    LaserscanData(ros::Time stamp,
                  std::string sensor_frame,
                  sensor_msgs::LaserScan scan);

    virtual graph_slam_msgs::SensorData toMsg();
    virtual void fromMsg(graph_slam_msgs::SensorData msg);

    sensor_msgs::LaserScan scan_;
};

typedef boost::shared_ptr<SensorData> SensorDataPtr;
typedef boost::shared_ptr<FeatureData> FeatureDataPtr;
typedef boost::shared_ptr<DepthImageData> DepthImageDataPtr;
typedef boost::shared_ptr<BinaryGistData> BinaryGistDataPtr;
typedef boost::shared_ptr<LaserscanData> LaserscanDataPtr;

#endif
