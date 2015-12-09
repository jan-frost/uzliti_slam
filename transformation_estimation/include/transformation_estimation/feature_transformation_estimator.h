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

#ifndef FEATURE_TRANSFORMATION_ESTIMATION_H
#define FEATURE_TRANSFORMATION_ESTIMATION_H

#include <transformation_estimation/transformation_estimator.h>

#include <eigen3/Eigen/Dense>
#include <boost/function.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <transformation_estimation/FeatureLinkEstimationConfig.h>

typedef boost::function<void(Eigen::MatrixXd, Eigen::MatrixXd, Eigen::Isometry3d&)> PoseEstimationFunction;
typedef boost::function<int(Eigen::MatrixXd, Eigen::MatrixXd, Eigen::Isometry3d&, Eigen::Array<bool,1,Eigen::Dynamic>&)> ConsensusFunction;

class FeatureTransformationEstimator : public TransformationEstimator
{
public:
    FeatureTransformationEstimator(boost::function<void (SlamEdge)> callback);

    bool estimateEdgeImpl(SlamNode &from, SlamNode &to, SlamEdge &edge);

    void setConfig(transformation_estimation::FeatureLinkEstimationConfig config);

    bool estimateEdgeDirect(std::vector<SensorDataPtr> from, std::vector<SensorDataPtr> to, SlamEdge &edge);

    void initCameraModel(sensor_msgs::CameraInfo info);
    void estimateSVD(Eigen::MatrixXd P, Eigen::MatrixXd Q, Eigen::Isometry3d &T, int &consensus, double &mse, double maxError, int iterations, double breakPercentage, bool do_prosac = true);

    void prosac(Eigen::MatrixXd P, Eigen::MatrixXd Q, Eigen::Isometry3d &T, int &consensus,
               double &mse, int iterations, double breakPercentage, int minCorrespondenceCount,
               PoseEstimationFunction pose_function, ConsensusFunction consensus_function, bool do_prosac = true);

    void estimatePoseSVD(Eigen::MatrixXd P, Eigen::MatrixXd Q, Eigen::Isometry3d &T);
    int consensusReprojection(Eigen::MatrixXd P, Eigen::MatrixXd Q, Eigen::Isometry3d T, double thresh, Eigen::Array<bool,1,Eigen::Dynamic> &consensusSet);
    int consensus3D(Eigen::MatrixXd P, Eigen::MatrixXd Q, Eigen::Isometry3d T, double thresh, Eigen::Array<bool,1,Eigen::Dynamic> &consensusSet);

protected:
    image_geometry::PinholeCameraModel camera_;
    transformation_estimation::FeatureLinkEstimationConfig config_;
};

#endif
