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

#include <feature_extraction/feature_extraction_core.h>

#include <chrono>
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
        detector_ = new cv::ORB(number_of_features);
        extractor_ = new cv::ORB();
        ROS_INFO("ORB descriptor");
        break;
    case graph_slam_msgs::Features::BRISK:
        detector_ = new cv::BRISK(5);
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

    this->adapted_detector_ = cv::GridAdaptedFeatureDetector(detector_, number_of_features_, 1, 1);
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

std::vector<float> FeatureExtractionCore::extractBinaryGist(const cv::Mat &image)
{
    cv::OrbDescriptorExtractor binary_extractor;
    int patch_size = 63;
    int center = (patch_size - 1) / 2;

    cv::Mat img_gist;
    cv::resize(image, img_gist, cv::Size(patch_size, patch_size));

    std::vector<cv::KeyPoint> keypoints;
    keypoints.push_back(cv::KeyPoint(center, center, center));
    cv::Mat desc_mat;
    binary_extractor.compute(img_gist, keypoints, desc_mat);

    std::vector<float> descriptor;
    for (int j = 0; j < desc_mat.cols; j++) {
        descriptor.push_back(desc_mat.at<unsigned char>(0,j));
    }

    return descriptor;
}

std::vector<graph_slam_msgs::Feature> FeatureExtractionCore::extract2dFeatures(const cv::Mat &color_image, const cv::Mat &depth_image, int number_of_features, bool use_mask)
{
    std::vector<graph_slam_msgs::Feature> features;

    std::vector<cv::KeyPoint> keypoints;
    cv::Mat desc;

    if (use_mask) {
        //convert depth image to a mask used for feature extraction in the rgb image
        cv::Mat depthMask(depth_image.rows, depth_image.cols, CV_8U);
        for (int i = 0; i < depth_image.rows; i++) {
            for (int j = 0; j < depth_image.cols; j++) {
                double depth = ((double) depth_image.at<float>(i,j));
                if (depth != 0 && !std::isnan(depth)) {
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
            default:
                ROS_ERROR("unknown feature type: %d", descriptor_type_);
                break;
            }
            feature.descriptor.push_back(val);
        }

        features.push_back(feature);
    }

    return features;
}

std::vector<graph_slam_msgs::Feature> FeatureExtractionCore::extract3dFeatures(const std::vector<graph_slam_msgs::Feature> &features_2d, const cv::Mat &depth_image, image_geometry::PinholeCameraModel camera_model)
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
        if (depth != 0 && !std::isnan(depth)/* && depth <= 5.0*/)
        {
            feature.keypoint_position.z = depth;
            feature.keypoint_position.x = (u - camera_model.cx()) * depth / camera_model.fx();
            feature.keypoint_position.y = (v - camera_model.cy()) * depth / camera_model.fy();
            feature.is_3d = true;

            features.push_back(feature);
        }
    }
    ROS_DEBUG_NAMED("feature extraction" , "found %d keypoints with 3d information", (int)features.size());
    return features;
}
