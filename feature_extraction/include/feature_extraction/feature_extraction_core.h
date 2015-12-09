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

#ifndef FEATURE_EXTRACTION_CORE_H_
#define FEATURE_EXTRACTION_CORE_H_

#include <graph_slam_msgs/Features.h>
#include <graph_slam_common/sensor_data.h>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

using namespace std;

class FeatureExtractionCore {

public:
    FeatureExtractionCore();
    FeatureExtractionCore(int descriptor_type, int number_of_features_);
    ~FeatureExtractionCore();

    /**
      * @brief Extract the number of desired features from the image using a mask and stores them into keypoints and descriptors
      * @param image - the image extracting the features from
      * @param mask - a mask for ignoring some regions in the image
      * @param keypoints - a reference to store the extracted keypoints in
      * @param descriptors - a reference to store the extracted descriptors in
      * @param desFeatures - the number of features to find
      */
    void extractFeatures(const cv::Mat &image, const cv::Mat &mask, vector<cv::KeyPoint > &keypoints, cv::Mat &descriptors, int desFeatures );

    /**
      * @brief returns current feature descriptor used based on the defined detector types
      * @return current used feature detector
      */
    int getExtractorType();

    /**
      * @brief set the number of features to extract
      * @param numFeat - the number of features to extract
      */
    void setNumberOfFeatures(int number_of_features);

    /**
      * @brief get the number of features to extract
      * @return the current number of features to extract
      */
    int getNumberOfFeatures();

    std::vector<float> extractBinaryGist(const cv::Mat &image, image_geometry::PinholeCameraModel camera_model, Eigen::Isometry3d camera_transform = Eigen::Isometry3d::Identity());

    graph_slam_msgs::SensorDataArray extractSensorData(const cv::Mat &grey, cv::Mat &depth, image_geometry::PinholeCameraModel camera_model, bool extract_gist, bool extract_features, double max_depth);

    std::vector<graph_slam_msgs::Feature> extract2dFeatures(const cv::Mat &image, const cv::Mat &depth_image, int number_of_features_, double max_depth = 0.0);
    std::vector<graph_slam_msgs::Feature> extract3dFeatures(const std::vector<graph_slam_msgs::Feature> &features_2d, const cv::Mat &depth_image, image_geometry::PinholeCameraModel camera_model, double max_depth = 0.0);
    std::vector<graph_slam_msgs::Feature> extract3dFeaturesFiltered(const std::vector<graph_slam_msgs::Feature> &features_2d, const cv::Mat &depth_image, image_geometry::PinholeCameraModel camera_model);

private:
    int number_of_features_; /** @param number of features to extract */

    //detectors and extractors
    cv::GridAdaptedFeatureDetector adapted_detector_;
    cv::Ptr<cv::FeatureDetector> detector_;
    cv::Ptr<cv::DescriptorExtractor> extractor_;

    int descriptor_type_;
};

#endif //FEATURE_EXTRACTION_H_
