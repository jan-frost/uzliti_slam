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

#ifndef FEATURE_EXTRACTION_CORE_H_
#define FEATURE_EXTRACTION_CORE_H_

#include <graph_slam_msgs/Features.h>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/opencv.hpp>

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

    std::vector<float> extractBinaryGist(const cv::Mat &image);


    std::vector<graph_slam_msgs::Feature> extract2dFeatures(const cv::Mat &image, const cv::Mat &depth_image, int number_of_features_, bool use_mask = true);
    std::vector<graph_slam_msgs::Feature> extract3dFeatures(const std::vector<graph_slam_msgs::Feature> &features_2d, const cv::Mat &depth_image, image_geometry::PinholeCameraModel camera_model);

private:
    int number_of_features_; /** @param number of features to extract */

    //detectors and extractors
    cv::GridAdaptedFeatureDetector adapted_detector_;
    cv::Ptr<cv::FeatureDetector> detector_;
    cv::Ptr<cv::DescriptorExtractor> extractor_;

    int descriptor_type_;
};

#endif //FEATURE_EXTRACTION_H_
