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

#include <graph_slam_tools/transformation/feature_transformation_estimator.h>

#include <boost/bind.hpp>
#include <graph_slam_msgs/Edge.h>
#include <graph_slam_msgs/Features.h>
#include <pcl/common/transformation_from_correspondences.h>

FeatureTransformationEstimator::FeatureTransformationEstimator(boost::function<void (SlamEdge)> callback) :
    TransformationEstimator(callback)
{
}

bool FeatureTransformationEstimator::estimateEdgeImpl(SlamNode &from, SlamNode &to, SlamEdge &edge)
{
    // Get features from different frames.
    std::vector<SensorDataPtr> sensor_data_from = from.sensor_data_;
    std::vector<SensorDataPtr> sensor_data_to = to.sensor_data_;

    // Match pairwise.
    double best_matching_score = std::numeric_limits<double>::max();
    std::pair<FeatureDataPtr,FeatureDataPtr> best_match;
    std::vector<cv::DMatch> potentialNeighborMatches;
    cv::BFMatcher binaryMatcher(cv::NORM_HAMMING);

    for (auto data_from : sensor_data_from) {
        if (data_from->type_ == graph_slam_msgs::SensorData::SENSOR_TYPE_FEATURE) {
            for (auto data_to : sensor_data_to) {
                if (data_to->type_ == graph_slam_msgs::SensorData::SENSOR_TYPE_FEATURE) {
                    FeatureDataPtr feature_data_from = boost::dynamic_pointer_cast<FeatureData>(data_from);
                    FeatureDataPtr feature_data_to = boost::dynamic_pointer_cast<FeatureData>(data_to);

                    if (feature_data_from->feature_positions_.cols() >= 7 && feature_data_to->feature_positions_.cols() >= 7 &&
                            feature_data_from->feature_type_ == feature_data_to->feature_type_) {
                        // Match features for each node.
                        std::vector<cv::DMatch> matches;
                        switch (feature_data_from->feature_type_) {
                        case graph_slam_msgs::Features::BRIEF:
                        case graph_slam_msgs::Features::ORB:
                        case graph_slam_msgs::Features::BRISK:
                        case graph_slam_msgs::Features::FREAK:
                            binaryMatcher.match(feature_data_to->features_, feature_data_from->features_, matches);
                            break;
                        default:
                            ROS_ERROR("unknown feature type: %d", feature_data_from->feature_type_);
                            break;
                        }

                        // Sum up score.
                        double score = 0;
                        for (auto match : matches) {
                            score += match.distance;
                        }

                        if (score < best_matching_score) {
                            best_matching_score = score;
                            best_match = std::pair<FeatureDataPtr,FeatureDataPtr>(feature_data_from, feature_data_to);
                            potentialNeighborMatches = matches;
                            initCameraModel(feature_data_from->camera_model_.cameraInfo());
                        }
                    }
                }
            }
        }
    }

    // Estimate transformation for best match.
    if (best_matching_score < std::numeric_limits<double>::max() && potentialNeighborMatches.size() >= 7) {

        Eigen::MatrixXd Xd = Eigen::MatrixXd::Zero(3,potentialNeighborMatches.size());
        Eigen::MatrixXd Pd = Eigen::MatrixXd::Zero(3,potentialNeighborMatches.size());
        for (unsigned int i = 0; i < potentialNeighborMatches.size(); i++) {
            Xd.col(i) = best_match.first->feature_positions_.col(potentialNeighborMatches[i].trainIdx);
            Pd.col(i) = best_match.second->feature_positions_.col(potentialNeighborMatches[i].queryIdx);
        }

        // Estimate pose with either SVD or EPnP.
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        int consensus = estimateSVD(Pd, Xd, T, config_.ransac_threshold, config_.ransac_iteration, config_.ransac_break_percentage);
        ROS_DEBUG("CONSENSUS: %d", consensus);

        Eigen::MatrixXd link_covariance = Eigen::MatrixXd::Zero(6,6);
        link_covariance(0, 0) = config_.link_covariance * config_.link_covariance;
        link_covariance(1, 1) = link_covariance(0, 0);
        link_covariance(2, 2) = link_covariance(0, 0);
        link_covariance(3, 3) = std::sin(std::atan2(config_.link_covariance, 2.0));
        link_covariance(3, 3) *= link_covariance(3, 3);
        link_covariance(4, 4) = link_covariance(3, 3);
        link_covariance(5, 5) = link_covariance(3, 3);

        // Initialize to edge.
        edge.id_from_ = from.id_;
        edge.id_to_ = to.id_;
        edge.transform_ = T;
        edge.information_ = link_covariance.inverse();
        edge.type_ = graph_slam_msgs::Edge::TYPE_3D_FULL;
        edge.sensor_from_ = best_match.first->sensor_frame_;
        edge.sensor_to_ = best_match.second->sensor_frame_;
        edge.matching_score_ = ((double)consensus) / potentialNeighborMatches.size();
        return true;
    }
    return false;
}

void FeatureTransformationEstimator::initCameraModel(sensor_msgs::CameraInfo info)
{
    this->camera_.fromCameraInfo(info);
}

int FeatureTransformationEstimator::estimateSVD(Eigen::MatrixXd P, Eigen::MatrixXd Q, Eigen::Isometry3d &T, double maxError, int iterations, double breakPercentage)
{
    return prosac(P, Q, T, maxError, iterations, breakPercentage, 3,
                  boost::bind(&FeatureTransformationEstimator::estimatePoseSVD, this, _1, _2, _3),
                  boost::bind(&FeatureTransformationEstimator::consensus3D, this, _1, _2, _3, maxError, _4));
}

int FeatureTransformationEstimator::prosac(Eigen::MatrixXd P, Eigen::MatrixXd Q, Eigen::Isometry3d &T, double maxError, int iterations, double breakPercentage, int minCorrespondenceCount, PoseEstimationFunction poseFunction, ConsensusFunction consensusFunction)
{
    std::vector<int> idx;
    for (int i = 0; i < P.cols(); i++) {
        idx.push_back(i);
    }

    // Consensus variables for PROSAC.
    int maxConsensus = 0;
    int consensus = 0;
    Eigen::Array<bool,1,Eigen::Dynamic> consensusSet;
    Eigen::Array<bool,1,Eigen::Dynamic> maxConsensusSet;

    Eigen::Isometry3d Ttemp;
    Eigen::MatrixXd Ptemp(3, minCorrespondenceCount);
    Eigen::MatrixXd Qtemp(3, minCorrespondenceCount);

    // Get maximum consensus transform.
    for (int i = 0; i < iterations; i++) {
        // Shuffle part of the set of correspondences.
        std::random_shuffle(idx.begin(), std::min(idx.begin() + std::ceil(((double)i / iterations) * P.cols()), idx.end()));

        for (int j = 0; j < minCorrespondenceCount; j++) {
            Ptemp.col(j) = P.col(idx[j]);
            Qtemp.col(j) = Q.col(idx[j]);
        }

        poseFunction(Ptemp, Qtemp, Ttemp);

        // Estimate consensus set.
        consensus = consensusFunction(P, Q, Ttemp, consensusSet);

        // Check if this is the new maximum consensus set.
        if (consensus > maxConsensus) {
            maxConsensus = consensus;
            maxConsensusSet = consensusSet;
            T = Ttemp;

            // Break if the consensus set is large enough.
            if (maxConsensus > breakPercentage * P.cols()) {
                break;
            }
        }
    }

    if (maxConsensus > minCorrespondenceCount) {
        int lastConsensus = maxConsensus;
        Eigen::Isometry3d Tlast = T;
        do {
            if (maxConsensus <= minCorrespondenceCount || lastConsensus > maxConsensus) {
                break;
            }

            T = Tlast;

            Eigen::MatrixXd Pfinal(3, maxConsensus);
            Eigen::MatrixXd Qfinal(3, maxConsensus);
            lastConsensus = maxConsensus;
            int k = 0;
            for (int i = 0; i < P.cols(); i++) {
                if (maxConsensusSet[i]) {
                    Pfinal.col(k) = P.col(i);
                    Qfinal.col(k) = Q.col(i);
                    k++;
                }
            }
            poseFunction(Pfinal, Qfinal, Tlast);
            maxConsensus = consensusFunction(P, Q, Tlast, maxConsensusSet);
        } while (lastConsensus != maxConsensus);
    } else {
        T = Eigen::Isometry3d::Identity();
    }

    return maxConsensus;
}

void FeatureTransformationEstimator::estimatePoseSVD(Eigen::MatrixXd P, Eigen::MatrixXd Q, Eigen::Isometry3d &T)
{
    pcl::TransformationFromCorrespondences tfc;
    for (int i = 0; i < P.cols(); i++) {
        tfc.add(P.col(i).cast<float>(), Q.col(i).cast<float>());
    }

    T = tfc.getTransformation().cast<double>().matrix();
}

int FeatureTransformationEstimator::consensusReprojection(Eigen::MatrixXd P, Eigen::MatrixXd Q, Eigen::Isometry3d T, double thresh, Eigen::Array<bool, 1, Eigen::Dynamic> &consensusSet)
{
    Eigen::MatrixXd Pproj(2, P.cols());
    Eigen::MatrixXd Qproj(2, P.cols());
    for (int i = 0; i < P.cols(); i++) {
        Eigen::Vector3d PT = T * P.col(i).homogeneous();
        Pproj(0,i) = camera_.cx() + camera_.fx() * PT(0) / PT(2);
        Pproj(1,i) = camera_.cy() + camera_.fy() * PT(1) / PT(2);

        Qproj(0,i) = camera_.cx() + camera_.fx() * Q(0,i) / Q(2,i);
        Qproj(1,i) = camera_.cy() + camera_.fy() * Q(1,i) / Q(2,i);
    }

    int consensus = 0;
    Eigen::MatrixXd norms = (Pproj - Qproj).colwise().norm();
    consensusSet = norms.array() < thresh;
    consensus = consensusSet.count();

    return consensus;
}

int FeatureTransformationEstimator::consensus3D(Eigen::MatrixXd P, Eigen::MatrixXd Q, Eigen::Isometry3d T, double thresh, Eigen::Array<bool, 1, Eigen::Dynamic> &consensusSet)
{
    int consensus = 0;

    P = T * P.colwise().homogeneous();
    Eigen::MatrixXd norms = (P - Q).colwise().norm();
    consensusSet = norms.array() < thresh;
    consensus = consensusSet.count();

    return consensus;
}
