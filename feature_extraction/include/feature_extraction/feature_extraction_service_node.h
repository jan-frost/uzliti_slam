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

#ifndef FEATURE_EXTRACTION_SERVICE_NODE_H
#define FEATURE_EXTRACTION_SERVICE_NODE_H

#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <feature_extraction/feature_extraction_core.h>
#include <feature_extraction/FeatureExtractionConfig.h>
#include <graph_slam_msgs/Features.h>
#include <graph_slam_msgs/SensorRequest.h>
#include <map_projection/graph_grid_mapper.h>
#include <image_geometry/pinhole_camera_model.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <nodelet/nodelet.h>
#include <opencv/cv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <tf/message_filter.h>
#include <tf/transform_listener.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::CameraInfo> RGBDSyncPolicy;

class RGBDCameraListerner {
public:
    RGBDCameraListerner(ros::NodeHandle nh, std::string prefix) :
        rgb_img_sub_(nh, prefix + "/rgb/image_raw", 10),
        rgb_info_sub_(nh, prefix + "/rgb/camera_info", 10),
        depth_img_sub_(nh, prefix + "/depth_registered/image_raw", 10),
        depth_info_sub_(nh, prefix + "/depth_registered/camera_info", 10)
    {
        sync = new message_filters::Synchronizer<RGBDSyncPolicy>(RGBDSyncPolicy(10), rgb_img_sub_, rgb_info_sub_, depth_img_sub_, depth_info_sub_);
        sync->registerCallback(boost::bind(&RGBDCameraListerner::imageCallback, this, _1, _2, _3, _4));
        tick_ = 0;
        last_get_ = -1;
    }

    ~RGBDCameraListerner() {
        delete(sync);
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& rgb_img_msg,
                       const sensor_msgs::CameraInfoConstPtr &rgb_info_msg,
                       const sensor_msgs::ImageConstPtr &depth_img_msg,
                       const sensor_msgs::CameraInfoConstPtr &depth_info_msg) {

        this->color_image_msg_ = *rgb_img_msg;
        this->depth_image_msg_ = *depth_img_msg;
        this->depth_info_msg_ = *depth_info_msg;
        tick_++;
    }

    bool getRgbdImage(sensor_msgs::Image &gray_img, sensor_msgs::Image &depth_img, sensor_msgs::CameraInfo &info) {
        if (last_get_ == tick_) {   // Check if this is realy a new image.
            return false;
        }

        gray_img = color_image_msg_;
        depth_img = depth_image_msg_;
        info = depth_info_msg_;

        last_get_ = tick_;
        return true;
    }

    bool getRgbdImage(cv::Mat &gray_img, cv::Mat &depth_img, image_geometry::PinholeCameraModel &camera_model) {
        if (last_get_ == tick_) {   // Check if this is realy a new image.
            return false;
        }

        cv_bridge::CvImagePtr rgb_bridge;
        cv_bridge::CvImagePtr depth_bridge;
        try {
            rgb_bridge = cv_bridge::toCvCopy(color_image_msg_, sensor_msgs::image_encodings::MONO8);
            gray_img = rgb_bridge->image;
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("rgb cv_bridge exception (encoding %s): %s", color_image_msg_.encoding.c_str(), e.what());
            return false;
        }

        try {
            depth_bridge = cv_bridge::toCvCopy(depth_image_msg_, sensor_msgs::image_encodings::TYPE_32FC1);
            depth_img = depth_bridge->image;
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("depth cv_bridge exception (encoding %s):\n %s", depth_image_msg_.encoding.c_str(), e.what());
            return false;
        }

        // Get camera info for 3D point estimation.
        camera_model.fromCameraInfo(depth_info_msg_);

        last_get_ = tick_;
        return true;
    }

    sensor_msgs::Image getDepthImage() {
        return depth_image_msg_;
    }

    sensor_msgs::Image getColorImage() {
        return color_image_msg_;
    }

    graph_slam_msgs::Features getMessageStub() {
        graph_slam_msgs::Features features;
        features.header = depth_image_msg_.header;
        features.camera_model = depth_info_msg_;
        return features;
    }

private:
    message_filters::Subscriber<sensor_msgs::Image> rgb_img_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> rgb_info_sub_;
    message_filters::Subscriber<sensor_msgs::Image> depth_img_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> depth_info_sub_;
    message_filters::Synchronizer<RGBDSyncPolicy> *sync;

    sensor_msgs::Image color_image_msg_;
    sensor_msgs::Image depth_image_msg_;
    sensor_msgs::CameraInfo depth_info_msg_;

    int tick_;
    int last_get_;
};
typedef boost::shared_ptr<RGBDCameraListerner> RGBDCameraListernerPtr;


class FeatureExtractionServiceNode : public nodelet::Nodelet {
public:
    FeatureExtractionServiceNode();

    virtual void onInit();

    void requestCallback(const graph_slam_msgs::SensorRequestConstPtr &sensor_request);

    void configCallback(feature_extraction::FeatureExtractionConfig &config, uint32_t level);

    void occupancyGridConfigCallback(map_projection::OccupancyGridProjectorConfig &config, uint32_t level);

private:
    message_filters::Subscriber<graph_slam_msgs::SensorRequest> request_sub_;
    tf::MessageFilter<graph_slam_msgs::SensorRequest> *request_sub_filter_ ;

    std::vector<RGBDCameraListernerPtr> rgbd_listeners_;
    ros::Publisher sensor_data_publisher_;

    dynamic_reconfigure::Server<feature_extraction::FeatureExtractionConfig> server;
    dynamic_reconfigure::Server<feature_extraction::FeatureExtractionConfig>::CallbackType f;

    dynamic_reconfigure::Server<map_projection::OccupancyGridProjectorConfig> server_og;
    dynamic_reconfigure::Server<map_projection::OccupancyGridProjectorConfig>::CallbackType f_og;

    FeatureExtractionCore extraction_core;
    GraphGridMapper mapper;

    feature_extraction::FeatureExtractionConfig conf_;

    tf::TransformListener tfl; /** @brief to lookup transform between camera_link and base_link */
    std::string odom_frame_;
    bool use_feature_mask_;
};

#endif
