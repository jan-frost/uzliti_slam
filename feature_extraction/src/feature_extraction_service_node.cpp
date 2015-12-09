// Copyright (c) 2014, Institute of Computer Engineering (ITI), Universität zu Lübeck
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

#include <feature_extraction/feature_extraction_service_node.h>

#include <graph_slam_msgs/SensorDataArray.h>
#include <graph_slam_common/conversions.h>
#include <pluginlib/class_list_macros.h>
#include "filter.h"
#include <fstream>
#include <chrono>

using std::chrono::duration_cast;
using std::chrono::microseconds;
using std::chrono::milliseconds;
using std::chrono::minutes;
using std::chrono::system_clock;

PLUGINLIB_EXPORT_CLASS(FeatureExtractionServiceNode, nodelet::Nodelet)

struct DataComparator {
    bool operator() (graph_slam_msgs::SensorData i, graph_slam_msgs::SensorData j) {
        return (i.header.stamp < j.header.stamp);
    }
} DataComparator;

FeatureExtractionServiceNode::FeatureExtractionServiceNode() :
    server_og(ros::NodeHandle("~/occupancy_grid_estimation")),
    tfl(ros::Duration(10.))
{

}

void FeatureExtractionServiceNode::onInit()
{
    request_sub_.subscribe(getPrivateNodeHandle(), "/sensor_request", 10);
    request_sub_filter_ = new tf::MessageFilter<graph_slam_msgs::SensorRequest>(request_sub_, tfl, "/base_footprint", 10);
    request_sub_filter_->registerCallback(boost::bind(&FeatureExtractionServiceNode::requestCallback, this, _1));

    sensor_data_publisher_ = getPrivateNodeHandle().advertise<graph_slam_msgs::SensorDataArray>("/sensor_data", 1);

    f = boost::bind(&FeatureExtractionServiceNode::configCallback, this, _1, _2);
    server.setCallback(f);
    f_og = boost::bind(&FeatureExtractionServiceNode::occupancyGridConfigCallback, this, _1, _2);
    server_og.setCallback(f_og);

    int descriptor_type = graph_slam_msgs::Features::BRISK;
    int number_of_features = 300;
    getPrivateNodeHandle().param<int>("descriptor_type", descriptor_type, graph_slam_msgs::Features::BRISK);
    getPrivateNodeHandle().param<int>("number_of_features", number_of_features, 300);
    extraction_core = FeatureExtractionCore(descriptor_type, number_of_features);

    odom_frame_ = "/odom";
    getPrivateNodeHandle().param<std::string>("odom_frame", odom_frame_, "/odom");

    getPrivateNodeHandle().param<bool>("use_feature_mask", use_feature_mask_, false);

    std::vector<std::string> cams;
    std::vector<std::string> cams_default;
    cams_default.push_back("camera");
    getPrivateNodeHandle().param<std::vector<std::string>>("cams", cams, cams_default);
    ROS_INFO("Using the following cameras:");
    for (auto cam : cams) {
        ROS_INFO("Added camera '%s'", cam.c_str());
        rgbd_listeners_.push_back(RGBDCameraListernerPtr(new RGBDCameraListerner(getPrivateNodeHandle(), "/" + cam)));
    }
}

void FeatureExtractionServiceNode::requestCallback(const graph_slam_msgs::SensorRequestConstPtr &sensor_request)
{
    graph_slam_msgs::SensorDataArray sensor_data_array;
    sensor_data_array.header.stamp = ros::Time(0);
    sensor_data_array.header.frame_id = "/base_footprint";
    std::vector<std::string> target_frames;
    bool first = true;
    if (sensor_request->sensor_names.size() == 0) {
        ros::Time base_time = ros::Time::now();
        Eigen::Isometry3d base_transform = Eigen::Isometry3d::Identity();
        sensor_msgs::LaserScan scan_combined;
        for (unsigned int i = 0; i < rgbd_listeners_.size(); i++) {
            sensor_msgs::Image color_img_msg;
            sensor_msgs::Image depth_img_msg;
            sensor_msgs::CameraInfo info_msg;
            // Only add data, if this is a new image.
            if (rgbd_listeners_[i]->getRgbdImage(color_img_msg, depth_img_msg, info_msg)) {
                if (depth_img_msg.header.stamp.toSec() == 0) {
                    continue;
                }

                depth_img_msg.header.stamp = depth_img_msg.header.stamp + ros::Duration(conf_.kinect_delay);
                target_frames.push_back(depth_img_msg.header.frame_id);

                // Get monochrome image.
                cv_bridge::CvImagePtr color_bridge;
                cv::Mat gray_img;
                try {
                    color_bridge = cv_bridge::toCvCopy(color_img_msg, sensor_msgs::image_encodings::MONO8);
                    gray_img = color_bridge->image;
                } catch (cv_bridge::Exception& e) {
                    ROS_ERROR("rgb cv_bridge exception (encoding %s): %s", color_img_msg.encoding.c_str(), e.what());
                    continue;
                }

                // Get depth image and filter image.
                cv_bridge::CvImagePtr depth_bridge;
                cv::Mat depth_img;
                try {
                    depth_bridge = cv_bridge::toCvCopy(depth_img_msg, sensor_msgs::image_encodings::TYPE_32FC1);
                    depth_img = depth_bridge->image;
                    if (depth_img_msg.encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
                        depth_img = 0.001 * depth_img;
                    }
                    if (conf_.depth_scale != 1.) {
                        depth_img *= conf_.depth_scale;
                    }

                    if (conf_.use_bilateral_filter) {
                        int r = 3;
                        int sigs = 30;
                        int sigc = 50;
                        int pr = 2;
                        double ss = sigs/10.0;
                        double sc = sigc/10.0;
                        int d = 2*r+1;
                        cv::Mat filteredDepthf = cv::Mat::ones(depth_img.size(), CV_32F);
                        Mat srcImagef; gray_img.convertTo(srcImagef,CV_32F);
                        jointBilateralFilter(depth_img, srcImagef, filteredDepthf, Size(d,d), sc, ss, BILATERAL_SEPARABLE);
                        jointNearestFilter(filteredDepthf, depth_img, Size(2*pr+1,2*pr+1), filteredDepthf);
                        depth_img = filteredDepthf;

                        depth_bridge->image = depth_img;
                        depth_bridge->toImageMsg(depth_img_msg);
                    }
                } catch (cv_bridge::Exception& e) {
                    ROS_ERROR("depth cv_bridge exception (encoding %s):\n %s", depth_img_msg.encoding.c_str(), e.what());
                    continue;
                }

                // Publish all sensor data.
                if (depth_img.rows == 0 || depth_img.cols == 0) {
                    ROS_WARN("No image received yet.");
                    continue;
                }

                // Add depth image.
                if (conf_.store_depth_images || conf_.store_color_images) {
                    ROS_INFO("add depth data");
                    graph_slam_msgs::SensorData depth_data;
                    depth_data.header = depth_img_msg.header;
                    depth_data.sensor_type = graph_slam_msgs::SensorData::SENSOR_TYPE_DEPTH_IMAGE;
                    depth_data.sensor_frame = depth_img_msg.header.frame_id;
                    if (conf_.store_depth_images) depth_data.depth_image.depth = depth_img_msg;
                    if (conf_.store_color_images) depth_data.depth_image.color = color_img_msg;
                    depth_data.features.camera_model = info_msg;
                    sensor_data_array.data.push_back(depth_data);
                }

                image_geometry::PinholeCameraModel camera_model;
                camera_model.fromCameraInfo(info_msg);

                // Get camera info for 3D point estimation.
                graph_slam_msgs::Features feature_msg;
                feature_msg.header = depth_img_msg.header;
                feature_msg.camera_model = info_msg;
                feature_msg.descriptor_type = extraction_core.getExtractorType();

                std::vector<graph_slam_msgs::Feature> features;
                if (use_feature_mask_) {
                    features = extraction_core.extract2dFeatures(gray_img, depth_img, conf_.number_of_features, conf_.feature_max_depth);
                } else {
                    features = extraction_core.extract2dFeatures(gray_img, depth_img, conf_.number_of_features, 0);
                }
                feature_msg.features = extraction_core.extract3dFeatures(features, depth_img, camera_model, conf_.feature_max_depth);

                Eigen::Isometry3d odom_transform = Eigen::Isometry3d::Identity();
                Eigen::Isometry3d rot_transform = Eigen::Isometry3d::Identity();
                if (tfl.waitForTransform("/base_footprint", odom_frame_, depth_img_msg.header.stamp, ros::Duration(0.5))) {
                    if (Conversions::getTransform(tfl, odom_transform, odom_frame_, "/base_footprint", depth_img_msg.header.stamp)) {
                        Eigen::Vector3d rpy;

                        rpy = g2o::internal::toEuler(odom_transform.linear());
                        rpy(2) = 0;
                        rot_transform.linear() = g2o::internal::fromEuler(rpy);

                        rpy = g2o::internal::toEuler(odom_transform.linear());
                        rpy(0) = 0;
                        rpy(1) = 0;
                        odom_transform.linear() = g2o::internal::fromEuler(rpy);
                        if (first) {
                            base_transform = odom_transform;
                            base_time = depth_img_msg.header.stamp;
                        }
                    }
                } else {
                    ROS_ERROR("wait for transform failed");
                }
                Eigen::Isometry3d displacement = ((base_transform.inverse() * odom_transform) * rot_transform);

                if (displacement.translation().norm() > 0.1) {
                    displacement = Eigen::Isometry3d::Identity();
                }

                graph_slam_msgs::SensorData feature_data;
                feature_data.header = feature_msg.header;
                feature_data.header.stamp = base_time;
                feature_data.sensor_type = graph_slam_msgs::SensorData::SENSOR_TYPE_FEATURE;
                feature_data.sensor_frame = feature_data.header.frame_id;
                feature_data.features = feature_msg;
                feature_data.displacement = Conversions::toMsg(displacement);
                sensor_data_array.data.push_back(feature_data);

                // Get camera transform from TF.
                Eigen::Isometry3d camera_transform;
                tfl.waitForTransform("/base_footprint", depth_img_msg.header.frame_id, depth_img_msg.header.stamp, ros::Duration(0.5));
                if (!Conversions::getTransform(tfl, camera_transform, "/base_footprint", depth_img_msg.header.frame_id, depth_img_msg.header.stamp)) {
                    ROS_WARN("feature_extaction: could not get camera transform");
                    continue;
                }                
                camera_transform = displacement * camera_transform;

                // Add gist descriptor.
                graph_slam_msgs::SensorData gist_data;
                gist_data.header = feature_data.header;
                gist_data.sensor_type = graph_slam_msgs::SensorData::SENSOR_TYPE_BINARY_GIST;
                gist_data.sensor_frame = feature_data.header.frame_id;
                gist_data.gist_descriptor = extraction_core.extractBinaryGist(gray_img, camera_model, camera_transform);
                sensor_data_array.data.push_back(gist_data);

                if (sensor_data_array.header.stamp < feature_data.header.stamp) {
                    sensor_data_array.header.stamp = feature_data.header.stamp;
                }

                // Add laser scan.
                if (conf_.store_laser_scans) {
                    sensor_msgs::LaserScan scan;
                    scan.header = feature_data.header;
                    mapper.extractImageLaserLine(depth_img, camera_transform, camera_model, scan);

                    if (first) {
                        scan_combined = scan;
                        first = false;
                    } else {
                        scan_combined = mapper.mergeLaserScans(scan_combined, scan, Eigen::Isometry3d::Identity());
                    }
                }
            } else {
                ROS_DEBUG("tried to acquire previously captured image");
            }
        }

        if (conf_.store_laser_scans && scan_combined.ranges.size() > 0) {
            // Compute laser center.
            auto scan_mean = GraphGridMapper::scanMean(scan_combined);

            graph_slam_msgs::SensorData laser_data;
            laser_data.displacement = Conversions::toMsg(Eigen::Isometry3d::Identity());
            laser_data.header = scan_combined.header;
            laser_data.sensor_type = graph_slam_msgs::SensorData::SENSOR_TYPE_LASERSCAN;
            laser_data.sensor_frame = "/base_footprint";
            laser_data.scan = scan_combined;
            laser_data.scan_center.x = scan_mean(0);
            laser_data.scan_center.y = scan_mean(1);
            laser_data.scan_center.z = scan_mean(2);
            sensor_data_array.data.push_back(laser_data);
        }

        // Publish sensor data (sorted by timestamp).
        std::sort(sensor_data_array.data.begin(), sensor_data_array.data.end(), DataComparator);
        sensor_data_publisher_.publish(sensor_data_array);
    }
}

void FeatureExtractionServiceNode::configCallback(feature_extraction::FeatureExtractionConfig &config, uint32_t level)
{ //save config to use
    this->conf_ = config;
}

void FeatureExtractionServiceNode::occupancyGridConfigCallback(map_projection::OccupancyGridProjectorConfig &config, uint32_t level)
{
    mapper.setConfig(config);
}
