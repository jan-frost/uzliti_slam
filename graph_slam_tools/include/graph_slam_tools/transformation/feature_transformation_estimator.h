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

#ifndef FEATURE_TRANSFORMATION_ESTIMATION_H
#define FEATURE_TRANSFORMATION_ESTIMATION_H

#include <graph_slam_tools/transformation/transformation_estimator.h>

#include <eigen3/Eigen/Dense>
#include <boost/function.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <graph_slam_tools/FeatureLinkEstimationConfig.h>

typedef boost::function<void(Eigen::MatrixXd, Eigen::MatrixXd, Eigen::Isometry3d&)> PoseEstimationFunction;
typedef boost::function<int(Eigen::MatrixXd, Eigen::MatrixXd, Eigen::Isometry3d&, Eigen::Array<bool,1,Eigen::Dynamic>&)> ConsensusFunction;

class FeatureTransformationEstimator : public TransformationEstimator
{
public:
    FeatureTransformationEstimator(boost::function<void (SlamEdge)> callback);

    void setConfig(graph_slam_tools::FeatureLinkEstimationConfig config) {
        config_ = config;
    }

protected:
    bool estimateEdgeImpl(SlamNode &from, SlamNode &to, SlamEdge &edge);

    void initCameraModel(sensor_msgs::CameraInfo info);
    int estimateSVD(Eigen::MatrixXd P, Eigen::MatrixXd Q, Eigen::Isometry3d &T, double maxError, int iterations, double breakPercentage);

    int prosac(Eigen::MatrixXd P, Eigen::MatrixXd Q, Eigen::Isometry3d &T, double maxError, int iterations, double breakPercentage, int minCorrespondenceCount, PoseEstimationFunction pose, ConsensusFunction consensus);
    
    void estimatePoseSVD(Eigen::MatrixXd P, Eigen::MatrixXd Q, Eigen::Isometry3d &T);
    int consensusReprojection(Eigen::MatrixXd P, Eigen::MatrixXd Q, Eigen::Isometry3d T, double thresh, Eigen::Array<bool,1,Eigen::Dynamic> &consensusSet);
    int consensus3D(Eigen::MatrixXd P, Eigen::MatrixXd Q, Eigen::Isometry3d T, double thresh, Eigen::Array<bool,1,Eigen::Dynamic> &consensusSet);

    image_geometry::PinholeCameraModel camera_;
    graph_slam_tools::FeatureLinkEstimationConfig config_;
};

#endif
