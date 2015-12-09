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

#include <graph_slam_common/sensor_data.h>

#include <graph_slam_common/conversions.h>

SensorData::SensorData()
{
}

SensorData::SensorData(int type, ros::Time stamp, std::string sensor_frame) :
    type_(type),
    stamp_(stamp),
    sensor_frame_(sensor_frame),
    displacement_(Eigen::Isometry3d::Identity())
{
}

SensorData::~SensorData()
{
}

graph_slam_msgs::SensorData SensorData::toMsg()
{
    graph_slam_msgs::SensorData msg;
    msg.header.stamp = stamp_;
    msg.header.frame_id = sensor_frame_;
    msg.sensor_type = type_;
    msg.sensor_frame = sensor_frame_;
    msg.displacement = Conversions::toMsg(displacement_);
    return msg;
}

void SensorData::fromMsg(graph_slam_msgs::SensorData msg)
{
    type_ = msg.sensor_type;
    stamp_ = msg.header.stamp;
    sensor_frame_ = msg.header.frame_id;
    displacement_ = Conversions::fromMsg(msg.displacement);
}

//-------------------------------------------------------------------------------
// FeatureData

FeatureData::FeatureData()
{
}

FeatureData::FeatureData(ros::Time stamp, std::string sensor_frame, int feature_type, cv::Mat features, Eigen::MatrixXd feature_positions_2d, Eigen::MatrixXd feature_positions, std::vector<bool> valid_3d, image_geometry::PinholeCameraModel camera_model) :
    SensorData(graph_slam_msgs::SensorData::SENSOR_TYPE_FEATURE, stamp, sensor_frame),
    feature_type_(feature_type),
    features_(features),
    feature_positions_2d_(feature_positions_2d),
    feature_positions_(feature_positions),
    valid_3d_(valid_3d),
    camera_model_(camera_model)
{
}

graph_slam_msgs::SensorData FeatureData::toMsg()
{
    graph_slam_msgs::SensorData msg = SensorData::toMsg();

    msg.features.header.stamp = stamp_;
    msg.features.header.frame_id = sensor_frame_;
    msg.features.descriptor_type = feature_type_;
    msg.features.camera_model = camera_model_.cameraInfo();

    // Get descriptors and keypoints.
    graph_slam_msgs::Feature feature;
    for (int i = 0; i < features_.rows; i++) {
        feature.u = feature_positions_2d_(0,i);
        feature.v = feature_positions_2d_(1,i);
        feature.keypoint_strength = -1;
        feature.descriptor.clear();
        float val = 0.0f;
        for (int j = 0; j < features_.cols; j++) {
            switch (feature_type_) {
            case graph_slam_msgs::Features::BRIEF:
            case graph_slam_msgs::Features::ORB:
            case graph_slam_msgs::Features::BRISK:
            case graph_slam_msgs::Features::FREAK:
                val = features_.at<unsigned char>(i,j);
                break;
            case graph_slam_msgs::Features::SURF:
            case graph_slam_msgs::Features::SIFT:
                val = features_.at<float>(i,j);
                break;
            default:
                ROS_ERROR("unknown feature type: %d (conversion)", feature_type_);
                break;
            }
            feature.descriptor.push_back(val);
        }

        feature.keypoint_position.x = feature_positions_(0,i);
        feature.keypoint_position.y = feature_positions_(1,i);
        feature.keypoint_position.z = feature_positions_(2,i);
        feature.is_3d = valid_3d_[i];

        msg.features.features.push_back(feature);
    }

    return msg;
}

void FeatureData::fromMsg(graph_slam_msgs::SensorData msg)
{
    SensorData::fromMsg(msg);

    feature_type_ = msg.features.descriptor_type;
    camera_model_.fromCameraInfo(msg.features.camera_model);
    if (msg.features.features.size() > 0) {
        // Read all feature descriptors from the node message. We will need do distinguish between floating point and binary types.
        switch (feature_type_) {
        case graph_slam_msgs::Features::BRIEF:
        case graph_slam_msgs::Features::ORB:
        case graph_slam_msgs::Features::BRISK:
        case graph_slam_msgs::Features::FREAK:
            features_.create(msg.features.features.size(), msg.features.features[0].descriptor.size(), CV_8U);
            for (unsigned int i = 0; i < msg.features.features.size(); i++) {
                for (unsigned int j = 0; j < msg.features.features[0].descriptor.size(); j++) {
                    float val = msg.features.features[i].descriptor[j];
                    features_.at<unsigned char>(i,j) = (unsigned char)val;
                }
            }
            break;
        case graph_slam_msgs::Features::SURF:
        case graph_slam_msgs::Features::SIFT:
            features_.create(msg.features.features.size(), msg.features.features[0].descriptor.size(), CV_32F);
            for (unsigned int i = 0; i < msg.features.features.size(); i++) {
                for (unsigned int j = 0; j < msg.features.features[0].descriptor.size(); j++) {
                    features_.at<float>(i,j) = msg.features.features[i].descriptor[j];
                }
            }
            break;
        default:
            ROS_ERROR("unknown feature type: %d (conversion)", (int) feature_type_);
            break;
        }

        feature_positions_2d_.resize(2, msg.features.features.size());
        feature_positions_.resize(3, msg.features.features.size());
        for (unsigned int i = 0; i < msg.features.features.size(); i++) {
            geometry_msgs::Point pointCamera = msg.features.features[i].keypoint_position;
            feature_positions_(0,i) = pointCamera.x;
            feature_positions_(1,i) = pointCamera.y;
            feature_positions_(2,i) = pointCamera.z;
            feature_positions_2d_(0,i) = msg.features.features[i].u;
            feature_positions_2d_(1,i) = msg.features.features[i].v;
            valid_3d_.push_back(msg.features.features[i].is_3d);
        }
    }
}

void FeatureData::merge(const FeatureData &other)
{
    // combine descriptor matrices.
    // Match feature data.
    // keep the best K features, based on the distance to the second best match.
}

//-------------------------------------------------------------------------------
// DepthImageData

DepthImageData::DepthImageData()
{
}

DepthImageData::DepthImageData(ros::Time stamp, std::string sensor_frame, sensor_msgs::Image depth_image, image_geometry::PinholeCameraModel camera_model) :
    SensorData(graph_slam_msgs::SensorData::SENSOR_TYPE_DEPTH_IMAGE, stamp, sensor_frame),
    depth_image_(depth_image),
    camera_model_(camera_model)
{
}

graph_slam_msgs::SensorData DepthImageData::toMsg()
{
    graph_slam_msgs::SensorData msg = SensorData::toMsg();

    msg.depth_image.depth = depth_image_;
    msg.depth_image.color = color_image_;
    msg.features.camera_model = camera_model_.cameraInfo();

    return msg;
}

void DepthImageData::fromMsg(graph_slam_msgs::SensorData msg)
{
    SensorData::fromMsg(msg);

    depth_image_ = msg.depth_image.depth;
    color_image_ = msg.depth_image.color;
    camera_model_.fromCameraInfo(msg.features.camera_model);
}

//-------------------------------------------------------------------------------
// BinaryGistData

BinaryGistData::BinaryGistData()
{
}

BinaryGistData::BinaryGistData(ros::Time stamp, std::string sensor_frame, cv::Mat descriptor) :
    SensorData(graph_slam_msgs::SensorData::SENSOR_TYPE_BINARY_GIST, stamp, sensor_frame),
    descriptor_(descriptor)
{
}

graph_slam_msgs::SensorData BinaryGistData::toMsg()
{
    graph_slam_msgs::SensorData msg = SensorData::toMsg();

    for (int j = 0; j < descriptor_.cols; j++) {
        msg.gist_descriptor.push_back(descriptor_.at<unsigned char>(0,j));
    }

    return msg;
}

void BinaryGistData::fromMsg(graph_slam_msgs::SensorData msg)
{
    SensorData::fromMsg(msg);

    descriptor_.create(1, msg.gist_descriptor.size(), CV_8U);
    for (int i = 0; i < descriptor_.cols; i++) {
        descriptor_.at<unsigned char>(0, i) = (unsigned char)msg.gist_descriptor[i];
    }
}

//-------------------------------------------------------------------------------
// LaserscanData

LaserscanData::LaserscanData()
{
}

LaserscanData::LaserscanData(ros::Time stamp, std::string sensor_frame, sensor_msgs::LaserScan scan) :
    SensorData(graph_slam_msgs::SensorData::SENSOR_TYPE_LASERSCAN, stamp, sensor_frame),
    scan_(scan)
{
}

graph_slam_msgs::SensorData LaserscanData::toMsg()
{
    graph_slam_msgs::SensorData msg = SensorData::toMsg();
    msg.scan = scan_;
    msg.scan_center.x = scan_center_(0);
    msg.scan_center.y = scan_center_(1);
    msg.scan_center.z = scan_center_(2);
    return msg;
}

void LaserscanData::fromMsg(graph_slam_msgs::SensorData msg)
{
    SensorData::fromMsg(msg);

    scan_ = msg.scan;
    scan_center_(0) = msg.scan_center.x;
    scan_center_(1) = msg.scan_center.y;
    scan_center_(2) = msg.scan_center.z;
}
