// Copyright (c) 2014, Institute of Computer Engineering (ITI), Universität zu Lübeck
// Jan Frost, Jan Helge Klüssendorff
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

#include <feature_extraction/feature_extraction_core.h>

#include <aorb/aorb.h>
#include <chrono>
#include <g2o/types/slam3d/isometry3d_mappings.h>
#include <opencv2/nonfree/features2d.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/filters/median_filter.h>
#include <ros/ros.h>

using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::system_clock;

bool CustomKeyPointSort(const cv::KeyPoint &a, const cv::KeyPoint &b)
{
    if( a.response > b.response ) {
        return true;
    }
    else {
        return false;
    }
}

FeatureExtractionCore::FeatureExtractionCore()
{
    FeatureExtractionCore(graph_slam_msgs::Features::BRISK, 200);
}

FeatureExtractionCore::FeatureExtractionCore(int descriptor_type, int number_of_features)
{
    number_of_features_ = number_of_features;
    descriptor_type_ = descriptor_type;

    // Create descriptor extractor.
    switch (descriptor_type) {
    case graph_slam_msgs::Features::BRIEF:
        detector_ = new cv::FastFeatureDetector(5);
        extractor_ = new cv::ORB();
        ROS_INFO("BRIEF descriptor");
        break;
    case graph_slam_msgs::Features::ORB:
//        detector_ = new cv::ORB(number_of_features, 1.2f, 2, 31, 0, 2, cv::ORB::FAST_SCORE);
        detector_ = new cv::AORB(2 * number_of_features, 1.2f, 2, 31, 0, 2, cv::ORB::FAST_SCORE, 31, 5);
        extractor_ = new cv::AORB();
        ROS_INFO("ORB descriptor");
        break;
    case graph_slam_msgs::Features::BRISK:
        detector_ = new cv::BRISK(10);
        extractor_ = new cv::BRISK();
        ROS_INFO("BRISK descriptor");
        break;
    case graph_slam_msgs::Features::FREAK:
        detector_ = new cv::BRISK(10);
        extractor_ = new cv::FREAK();
        ROS_INFO("FREAK descriptor");
        break;
    default:
        ROS_ERROR("no valid descriptor extractor: %d", (int) descriptor_type);
        break;
    }

    this->adapted_detector_ = cv::GridAdaptedFeatureDetector(detector_, number_of_features_, 2, 2);
}

FeatureExtractionCore::~FeatureExtractionCore()
{
    detector_.release();
    extractor_.release();
}

void FeatureExtractionCore::extractFeatures(const cv::Mat &image, const cv::Mat &mask, vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors, int desFeatures)
{
    adapted_detector_.detect(image, keypoints, mask);
    ROS_DEBUG("# of keypoints: %d", (int)keypoints.size());

    // get the strongest, cause opencv doesnt do it
    sort(keypoints.begin(), keypoints.end(), CustomKeyPointSort);

    extractor_->compute(image,keypoints,descriptors);
}

int FeatureExtractionCore::getExtractorType()
{
    return descriptor_type_;
}

void FeatureExtractionCore::setNumberOfFeatures(int number_of_features)
{
    this->number_of_features_ = number_of_features;
}

int FeatureExtractionCore::getNumberOfFeatures()
{
    return this->number_of_features_;
}

std::vector<float> FeatureExtractionCore::extractBinaryGist(const cv::Mat &image, image_geometry::PinholeCameraModel camera_model, Eigen::Isometry3d camera_transform)
{
    cv::Mat rect_image;
    cv::Mat map1, map2;
    if (camera_model.distortionCoeffs().at<double>(0,0) != 0) {
        cv::initUndistortRectifyMap(camera_model.fullIntrinsicMatrix(),
                                    camera_model.distortionCoeffs(),
                                    cv::Mat(),
                                    camera_model.fullIntrinsicMatrix(),
                                    camera_model.fullResolution(),
                                    CV_16SC2,
                                    map1, map2);
        cv::remap(image, rect_image, map1, map2, CV_INTER_LINEAR);
    } else {
        rect_image = image;
    }

    // Get camera roll, pitch and yaw.
    Eigen::Vector3d rpy = g2o::internal::toEuler(camera_transform.linear());

    // Resize image to patch size.
    cv::OrbDescriptorExtractor binary_extractor;
    int patch_size = 63;
    int max_patch_size = 63;
    int center = (patch_size - 1) / 2;
    int max_center = (max_patch_size - 1) / 2;
    cv::Mat img_gist;
    cv::resize(rect_image, img_gist, cv::Size(max_patch_size, max_patch_size));

    // Define image center with roll orientation.
    cv::KeyPoint point = cv::KeyPoint(max_center, max_center, center, 180. * rpy(0) / M_PI);

    std::vector<cv::KeyPoint> keypoints;
    keypoints.push_back(point);
    cv::Mat desc_mat;
    binary_extractor.compute(img_gist, keypoints, desc_mat);

    std::vector<float> descriptor;
    for (int j = 0; j < desc_mat.cols; j++) {
        descriptor.push_back(desc_mat.at<unsigned char>(0,j));
    }

    return descriptor;
}

graph_slam_msgs::SensorDataArray FeatureExtractionCore::extractSensorData(const cv::Mat &grey, cv::Mat &depth, image_geometry::PinholeCameraModel camera_model, bool extract_gist, bool extract_features, double max_depth)
{
    graph_slam_msgs::SensorDataArray data_array;

    if (extract_features) {
        graph_slam_msgs::Features feature_msg;
        feature_msg.descriptor_type = getExtractorType();
        feature_msg.features = extract2dFeatures(grey, depth, number_of_features_, 0);
        if (depth.cols == grey.cols && depth.rows == grey.rows) {
            feature_msg.features = extract3dFeatures(feature_msg.features, depth, camera_model, max_depth);
        }

        graph_slam_msgs::SensorData feature_data;
        feature_data.header = feature_msg.header;
        feature_data.sensor_type = graph_slam_msgs::SensorData::SENSOR_TYPE_FEATURE;
        feature_data.sensor_frame = feature_msg.header.frame_id;
        feature_data.features = feature_msg;
        data_array.data.push_back(feature_data);
    }

    if (extract_gist) {
        graph_slam_msgs::SensorData gist_data;
        gist_data.sensor_type = graph_slam_msgs::SensorData::SENSOR_TYPE_BINARY_GIST;
        gist_data.gist_descriptor = extractBinaryGist(grey, camera_model);
        data_array.data.push_back(gist_data);
    }

    return data_array;
}

std::vector<graph_slam_msgs::Feature> FeatureExtractionCore::extract2dFeatures(const cv::Mat &color_image, const cv::Mat &depth_image, int number_of_features, double max_depth)
{
    std::vector<graph_slam_msgs::Feature> features;

    std::vector<cv::KeyPoint> keypoints;
    cv::Mat desc;

    if (max_depth > 0.) {
        //convert depth image to a mask used for feature extraction in the rgb image
        cv::Mat depthMask(depth_image.rows, depth_image.cols, CV_8U);
        for (int i = 0; i < depth_image.rows; i++) {
            for (int j = 0; j < depth_image.cols; j++) {
                double depth = ((double) depth_image.at<float>(i,j));
                if (depth != 0 && !std::isnan(depth) && depth <= max_depth) {
                    depthMask.at<unsigned char>(i,j) = 255;
                } else {
                    depthMask.at<unsigned char>(i,j) = 0;
                }
            }
        }
        this->extractFeatures(color_image, depthMask, keypoints, desc, number_of_features);
    } else {
        this->extractFeatures(color_image, cv::Mat(), keypoints, desc, number_of_features);
    }

    //put extracted keypoints into ros msg
    graph_slam_msgs::Feature feature;
    for (unsigned int i = 0; i < keypoints.size(); i++)
    {
        feature.u = keypoints.at(i).pt.x;
        feature.v = keypoints.at(i).pt.y;
        feature.keypoint_strength = keypoints.at(i).response;
        feature.descriptor.clear();

        float val = 0.0f;
        for (int j = 0; j < desc.cols; j++) {
            switch (descriptor_type_) {
            case graph_slam_msgs::Features::BRIEF:
            case graph_slam_msgs::Features::ORB:
            case graph_slam_msgs::Features::BRISK:
            case graph_slam_msgs::Features::FREAK:
                val = desc.at<unsigned char>(i,j);
                break;
            case graph_slam_msgs::Features::SURF:
            case graph_slam_msgs::Features::SIFT:
                val = desc.at<float>(i,j);
                break;
            default:
                ROS_ERROR_NAMED("feature_extraction_service", "unknown feature type: %d (FE)", descriptor_type_);
                break;
            }
            feature.descriptor.push_back(val);
        }

        features.push_back(feature);
    }

    return features;
}

std::vector<graph_slam_msgs::Feature> FeatureExtractionCore::extract3dFeatures(const std::vector<graph_slam_msgs::Feature> &features_2d, const cv::Mat &depth_image, image_geometry::PinholeCameraModel camera_model, double max_depth)
{
    vector<graph_slam_msgs::Feature> features;

    int width = depth_image.cols;
    int height = depth_image.rows;

    //map keypoints to image coordinates
    for (int i = features_2d.size() - 1; i >= 0; i--) {
        graph_slam_msgs::Feature feature = features_2d[i];
        int u = round(feature.u);
        if(u < 0) {
            u = 0;
        } else if (u >= width) {
            u = width -1;
        }
        int v = round(feature.v);
        if(v < 0) {
            v = 0;
        } else if (v >= height) {
            v = height - 1;
        }


        double depth = ((double) depth_image.at<float>(v,u))/* * 0.94*/;        // will need to fix this.
        //transform 2d points to 3d
        if (depth != 0 && !std::isnan(depth) && (max_depth == 0. || depth <= max_depth)) {
            feature.keypoint_position.z = depth;
            feature.keypoint_position.x = (u - camera_model.cx()) * depth / camera_model.fx();
            feature.keypoint_position.y = (v - camera_model.cy()) * depth / camera_model.fy();
            feature.is_3d = true;
        } else {
            feature.keypoint_position.z = -1;
            feature.keypoint_position.x = 0;
            feature.keypoint_position.y = 0;
            feature.is_3d = false;
        }
        features.push_back(feature);
    }
    ROS_DEBUG_NAMED("feature extraction" , "found %d keypoints with 3d information", (int)features.size());
    return features;
}

vector<graph_slam_msgs::Feature> FeatureExtractionCore::extract3dFeaturesFiltered(const vector<graph_slam_msgs::Feature> &features_2d, const cv::Mat &depth_image, image_geometry::PinholeCameraModel camera_model)
{
    auto start = system_clock::now();
    vector<graph_slam_msgs::Feature> features;

    int width = depth_image.cols;
    int height = depth_image.rows;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    // Fill in the cloud data
    cloud->width  = width;
    cloud->height = height;
    cloud->points.resize (cloud->width * cloud->height);
    filtered_cloud->width  = width;
    filtered_cloud->height = height;
    filtered_cloud->points.resize (filtered_cloud->width * filtered_cloud->height);

    for (int i = 0; i < width; i++) {
        for (int j = 0; j < height; j++) {
            double depth = ((double) depth_image.at<float>(j,i))/* * 0.94*/;        // will need to fix this.
            //transform 2d points to 3d
            if (depth != 0 && !std::isnan(depth)) {
                cloud->at(i,j).x = (i - camera_model.cx()) * depth / camera_model.fx();
                cloud->at(i,j).y = (j - camera_model.cy()) * depth / camera_model.fy();
                cloud->at(i,j).z = depth;
            } else {
                depth = 0.0;
                cloud->at(i,j).x = (i - camera_model.cx()) * depth / camera_model.fx();
                cloud->at(i,j).y = (j - camera_model.cy()) * depth / camera_model.fy();
                cloud->at(i,j).z = depth;
            }
        }
    }

    for (int i = features_2d.size() - 1; i >= 0; i--) {
        graph_slam_msgs::Feature feature = features_2d[i];
        int u = round(feature.u);
        if(u < 0) {
            u = 0;
        } else if (u >= width) {
            u = width -1;
        }
        int v = round(feature.v);
        if(v < 0) {
            v = 0;
        } else if (v >= height) {
            v = height - 1;
        }

        feature.keypoint_position.x = cloud->at(u,v).x;
        feature.keypoint_position.y = cloud->at(u,v).y;
        feature.keypoint_position.z = cloud->at(u,v).z;
        feature.is_3d = (feature.keypoint_position.z > 0);
        features.push_back(feature);
    }
    ROS_WARN("pcl filter: %d ms", (int)(duration_cast<milliseconds>(system_clock::now() - start).count()));

    return features;
}
