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

#include <feature_extraction/feature_extraction_service_node.h>

#include <graph_slam_msgs/SensorDataArray.h>
#include <graph_slam_tools/conversions.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(feature_extraction, FeatureExtractionServiceNode, feature_extraction::FeatureExtractionServiceNode, nodelet::Nodelet)

namespace feature_extraction
{

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

//FeatureExtractionServiceNode::~FeatureExtractionServiceNode(){}

void FeatureExtractionServiceNode::onInit()
{
    request_subscriber_ = getPrivateNodeHandle().subscribe<graph_slam_msgs::SensorRequest>("/sensor_request", 1, boost::bind(&feature_extraction::FeatureExtractionServiceNode::requestCallback, this, _1));

    sensor_data_publisher_ = getPrivateNodeHandle().advertise<graph_slam_msgs::SensorDataArray>("/sensor_data", 1);

    f = boost::bind(&feature_extraction::FeatureExtractionServiceNode::configCallback, this, _1, _2);
    server.setCallback(f);
    f_og = boost::bind(&feature_extraction::FeatureExtractionServiceNode::occupancyGridConfigCallback, this, _1, _2);
    server_og.setCallback(f_og);

    int descriptor_type = graph_slam_msgs::Features::BRISK;
    int number_of_features = 300;
    getPrivateNodeHandle().param<int>("descriptor_type", descriptor_type, graph_slam_msgs::Features::BRISK);
    getPrivateNodeHandle().param<int>("number_of_features", number_of_features, 300);
    extraction_core = FeatureExtractionCore(descriptor_type, number_of_features);

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
    sensor_data_array.header.frame_id = "/base_link";
    if (sensor_request->sensor_names.size() == 0) {
        for (unsigned int i = 0; i < rgbd_listeners_.size(); i++) {
            sensor_msgs::Image color_img_msg;
            sensor_msgs::Image depth_img_msg;
            sensor_msgs::CameraInfo info_msg;
            // Only add data, if this is a new image.
            if (rgbd_listeners_[i]->getRgbdImage(color_img_msg, depth_img_msg, info_msg)) {
                cv_bridge::CvImagePtr color_bridge;
                cv::Mat gray_img;
                try {
                    color_bridge = cv_bridge::toCvCopy(color_img_msg, sensor_msgs::image_encodings::MONO8);
                    gray_img = color_bridge->image;
                } catch (cv_bridge::Exception& e) {
                    ROS_ERROR("rgb cv_bridge exception (encoding %s): %s", color_img_msg.encoding.c_str(), e.what());
                    continue;
                }

                cv_bridge::CvImagePtr depth_bridge;
                cv::Mat depth_img;
                try {
                    depth_bridge = cv_bridge::toCvCopy(depth_img_msg, sensor_msgs::image_encodings::TYPE_32FC1);
                    depth_img = depth_bridge->image;
                } catch (cv_bridge::Exception& e) {
                    ROS_ERROR("depth cv_bridge exception (encoding %s):\n %s", depth_img_msg.encoding.c_str(), e.what());
                    continue;
                }

                // Publish all sensor data.
                if (depth_img.rows == 0 || depth_img.cols == 0) {
                    ROS_WARN("No image received yet.");
                    continue;
                }

                image_geometry::PinholeCameraModel camera_model;
                camera_model.fromCameraInfo(info_msg);

                // Get camera info for 3D point estimation.
                graph_slam_msgs::Features feature_msg;
                feature_msg.header = depth_img_msg.header;
                feature_msg.camera_model = info_msg;
                feature_msg.descriptor_type = extraction_core.getExtractorType();

                std::vector<graph_slam_msgs::Feature> features = extraction_core.extract2dFeatures(gray_img, depth_img, conf_.number_of_features);
                feature_msg.features = extraction_core.extract3dFeatures(features, depth_img, camera_model);

                graph_slam_msgs::SensorData feature_data;
                feature_data.header = feature_msg.header;
                feature_data.sensor_type = graph_slam_msgs::SensorData::SENSOR_TYPE_FEATURE;
                feature_data.sensor_frame = feature_msg.header.frame_id;
                feature_data.features = feature_msg;
                sensor_data_array.data.push_back(feature_data);

                // Add gist descriptor.
                graph_slam_msgs::SensorData gist_data;
                gist_data.header = feature_msg.header;
                gist_data.sensor_type = graph_slam_msgs::SensorData::SENSOR_TYPE_BINARY_GIST;
                gist_data.sensor_frame = feature_msg.header.frame_id;
                gist_data.gist_descriptor = extraction_core.extractBinaryGist(gray_img);
                sensor_data_array.data.push_back(gist_data);

                if (sensor_data_array.header.stamp < feature_data.header.stamp) {
                    sensor_data_array.header.stamp = feature_data.header.stamp;
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
                    depth_data.features.camera_model = feature_msg.camera_model;
                    sensor_data_array.data.push_back(depth_data);
                }

                // Add laser scan.
                if (conf_.store_laser_scans) {
                    Eigen::Isometry3d camera_transform;
                    Conversions::getTransform(tfl, camera_transform, "/base_link", depth_img_msg.header.frame_id, ros::Time(0));

                    std::vector<cv::Point3d> poi;
                    mapper.extractImageLaserLine(depth_img, camera_transform, camera_model, poi);

                    sensor_msgs::LaserScan scan;
                    mapper.toLaserMsg(poi, scan);

                    graph_slam_msgs::SensorData laser_data;
                    laser_data.header = depth_img_msg.header;
                    laser_data.header.frame_id = "/base_link";
                    laser_data.sensor_type = graph_slam_msgs::SensorData::SENSOR_TYPE_LASERSCAN;
                    laser_data.sensor_frame = depth_img_msg.header.frame_id;
                    laser_data.scan = scan;
                    sensor_data_array.data.push_back(laser_data);
                }
            }
        }
    }

    // Publish sensor data (sorted by timestamp).
    std::sort(sensor_data_array.data.begin(), sensor_data_array.data.end(), DataComparator);
    sensor_data_publisher_.publish(sensor_data_array);
}

void FeatureExtractionServiceNode::configCallback(feature_extraction::FeatureExtractionConfig &config, uint32_t level)
{ //save config to use
    this->conf_ = config;
}

void FeatureExtractionServiceNode::occupancyGridConfigCallback(graph_slam_tools::OccupancyGridProjectorConfig &config, uint32_t level)
{
    mapper.setConfig(config);
}

}
