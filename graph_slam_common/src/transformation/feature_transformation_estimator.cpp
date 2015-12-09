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
    pnp.set_internal_parameters(239.5, 319.5, 525.0, 525.0);
    pnp.set_maximum_number_of_correspondences(1000);
}

bool FeatureTransformationEstimator::estimateEdgeDirect(std::vector<SensorDataPtr> sensor_data_from, std::vector<SensorDataPtr> sensor_data_to, SlamEdge &edge)
{
    // Match pairwise.
    double best_matching_score = -1; //std::numeric_limits<double>::max();
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
                            feature_data_from->feature_type_ == feature_data_to->feature_type_ &&
                            feature_data_from->sensor_frame_ == feature_data_to->sensor_frame_) {
                        // Match features for each node.
                        std::vector<cv::DMatch> matches;
                        std::vector<std::vector<cv::DMatch> > knn_matches;
                        switch (feature_data_from->feature_type_) {
                        case graph_slam_msgs::Features::BRIEF:
                        case graph_slam_msgs::Features::ORB:
                        case graph_slam_msgs::Features::BRISK:
                        case graph_slam_msgs::Features::FREAK:
                            binaryMatcher.knnMatch(feature_data_to->features_, feature_data_from->features_, knn_matches, 2);
                            break;
                        default:
                            ROS_ERROR("unknown feature type: %d", feature_data_from->feature_type_);
                            break;
                        }

                        for (auto match_pair : knn_matches) {
                            if (match_pair.size() == 2) {
                                if (match_pair[0].distance < 0.99 * match_pair[1].distance) {
                                    matches.push_back(match_pair[0]);
                                }
                            }
                        }

                        // Sum up score.
                        double score = 0;
                        for (auto match : matches) {
                            score += match.distance;
                        }
                        score = (double) matches.size();

                        ROS_DEBUG_NAMED("feature_transformation", "match score: %f", score);
                        if (score > best_matching_score) {
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

    if (best_matching_score == -1) {
        return false;
    }

    ROS_DEBUG_NAMED("feature_transformation", "total keypoints: (%d,%d)", best_match.first->feature_positions_.cols(), best_match.second->feature_positions_.cols());
    ROS_DEBUG_NAMED("feature_transformation", "best match score: %f between frames %s and %s", best_matching_score, best_match.first->sensor_frame_.c_str(), best_match.second->sensor_frame_.c_str());
    ROS_DEBUG_NAMED("feature_transformation", "number of matches: %d", potentialNeighborMatches.size());

    // Filter matches based on distance.
    std::vector<cv::DMatch> final_matches;
    for (auto match : potentialNeighborMatches) {
        if (best_match.first->valid_3d_[match.trainIdx] && best_match.second->valid_3d_[match.queryIdx]) {
//            Eigen::Vector3d p = best_match.first->feature_positions_.col(match.trainIdx);
//            Eigen::Vector3d q = best_match.second->feature_positions_.col(match.queryIdx);
//            double angle = 180. * acos(p.dot(q) / (p.norm()*q.norm())) / M_PI;
//            if (fabs(p.norm() - q.norm()) < 3.0 && fabs(angle) < 60.) {
                final_matches.push_back(match);
//            }
        }
    }

    std::sort(final_matches.begin(), final_matches.end());
    ROS_DEBUG_NAMED("feature_transformation", "filtered matches: %d", final_matches.size());

    // Estimate transformation for best match.
    if (final_matches.size() >= 3) {
        Eigen::MatrixXd Xd = Eigen::MatrixXd::Zero(3,final_matches.size());
        Eigen::MatrixXd Pd = Eigen::MatrixXd::Zero(3,final_matches.size());
        for (unsigned int i = 0; i < final_matches.size(); i++) {
            Xd.col(i) = best_match.first->feature_positions_.col(final_matches[i].trainIdx);
            Pd.col(i) = best_match.second->feature_positions_.col(final_matches[i].queryIdx);
        }

        // Estimate pose with either SVD or EPnP.
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        int consensus = 0;
        double mse = 0;
        if (config_.use_epnp) {
            estimateEPNP(Pd, Xd, T, consensus, mse, config_.ransac_threshold, config_.ransac_iteration, config_.ransac_break_percentage);
        } else {
            estimateSVD(Pd, Xd, T, consensus, mse, config_.ransac_threshold, config_.ransac_iteration, config_.ransac_break_percentage);
        }
        ROS_DEBUG_NAMED("feature_transformation", "consensus: %d", consensus);

        Eigen::MatrixXd link_information = Eigen::MatrixXd::Identity(6,6);
        if (consensus > 0 && mse > 0) {
            link_information *= 0.1 * consensus / mse;
            link_information.block<3,3>(3,3) *= 100.;
        }
//        link_covariance(0, 0) = config_.link_covariance * config_.link_covariance;
//        link_covariance(1, 1) = link_covariance(0, 0);
//        link_covariance(2, 2) = link_covariance(0, 0);
//        link_covariance(3, 3) = std::sin(std::atan2(config_.link_covariance, 2.0));
//        link_covariance(3, 3) *= link_covariance(3, 3);
//        link_covariance(4, 4) = link_covariance(3, 3);
//        link_covariance(5, 5) = link_covariance(3, 3);

        // Initialize to edge.
        edge.transform_ = T;
        edge.information_ = link_information;
        edge.type_ = graph_slam_msgs::Edge::TYPE_3D_FULL;
        edge.sensor_from_ = best_match.first->sensor_frame_;
        edge.sensor_to_ = best_match.second->sensor_frame_;
        edge.displacement_from_ = best_match.first->displacement_;
        edge.displacement_to_ = best_match.second->displacement_;
//        edge.matching_score_ = ((double)consensus) / std::min(best_match.first->feature_positions_.cols(), best_match.second->feature_positions_.cols());
        edge.matching_score_ = consensus;
        return true;
    }
    return false;
}

bool FeatureTransformationEstimator::estimateEdgeImpl(SlamNode &from, SlamNode &to, SlamEdge &edge)
{
    // Get features from different frames.
    std::vector<SensorDataPtr> sensor_data_from = from.sensor_data_;
    std::vector<SensorDataPtr> sensor_data_to = to.sensor_data_;

    bool success = estimateEdgeDirect(sensor_data_from, sensor_data_to, edge);
    edge.id_from_ = from.id_;
    edge.id_to_ = to.id_;
    return success;
}

void FeatureTransformationEstimator::initCameraModel(sensor_msgs::CameraInfo info)
{
    this->camera_.fromCameraInfo(info);
    pnp.set_internal_parameters(camera_.cy(), camera_.cx(), camera_.fy(), camera_.fx());
}

void FeatureTransformationEstimator::estimateEPNP(Eigen::MatrixXd P, Eigen::MatrixXd Q, Eigen::Isometry3d &T, int &consensus, double &mse, double maxError, int iterations, double breakPercentage)
{
    prosac(P, Q, T, consensus, mse, iterations, breakPercentage, 7,
           boost::bind(&FeatureTransformationEstimator::estimatePoseEPNP, this, _1, _2, _3),
           boost::bind(&FeatureTransformationEstimator::consensusReprojection, this, _1, _2, _3, maxError, _4));
}

void FeatureTransformationEstimator::estimateSVD(Eigen::MatrixXd P, Eigen::MatrixXd Q, Eigen::Isometry3d &T, int &consensus, double &mse, double maxError, int iterations, double breakPercentage, bool do_prosac)
{
    prosac(P, Q, T, consensus, mse, iterations, breakPercentage, 3,
           boost::bind(&FeatureTransformationEstimator::estimatePoseSVD, this, _1, _2, _3),
           boost::bind(&FeatureTransformationEstimator::consensus3D, this, _1, _2, _3, maxError, _4),
           do_prosac);
}

void FeatureTransformationEstimator::prosac(Eigen::MatrixXd P,
                                            Eigen::MatrixXd Q,
                                            Eigen::Isometry3d &T,
                                            int &consensus,
                                            double &mse,
                                            int iterations,
                                            double breakPercentage,
                                            int minCorrespondenceCount,
                                            PoseEstimationFunction pose_function,
                                            ConsensusFunction consensus_function,
                                            bool do_prosac)
{
    std::vector<int> idx;
    for (int i = 0; i < P.cols(); i++) {
        idx.push_back(i);
    }

    // Consensus variables for PROSAC.
    int maxConsensus = 0;
    consensus = 0;
    Eigen::Array<bool,1,Eigen::Dynamic> consensusSet;
    Eigen::Array<bool,1,Eigen::Dynamic> maxConsensusSet;

    Eigen::Isometry3d Ttemp;
    Eigen::MatrixXd Ptemp(3, minCorrespondenceCount);
    Eigen::MatrixXd Qtemp(3, minCorrespondenceCount);

    // Get maximum consensus transform.
    for (int i = 0; i < iterations; i++) {
        // Shuffle part of the set of correspondences.
        if (do_prosac) {
            std::random_shuffle(idx.begin(), idx.begin() + std::min((int)std::ceil(((i + 3.) / iterations) * P.cols()), (int)P.cols()));
        } else {
            std::random_shuffle(idx.begin(), idx.end());
        }

        for (int j = 0; j < minCorrespondenceCount; j++) {
            Ptemp.col(j) = P.col(idx[j]);
            Qtemp.col(j) = Q.col(idx[j]);
        }

        pose_function(Ptemp, Qtemp, Ttemp);

        // Estimate consensus set.
        consensus = consensus_function(P, Q, Ttemp, consensusSet);

        // Check if this is the new maximum consensus set.
        if (consensus > maxConsensus) {
            maxConsensus = consensus;
            maxConsensusSet = consensusSet;
            T = Ttemp;

            // Break if the consensus set is large enough.
            if (maxConsensus >= minCorrespondenceCount && maxConsensus > breakPercentage * P.cols()) {
                break;
            }
        }
    }

    mse = 0.;
    if (maxConsensus >= minCorrespondenceCount) {
        Eigen::MatrixXd Pfinal(3, maxConsensus);
        Eigen::MatrixXd Qfinal(3, maxConsensus);
        int k = 0;
        for (int i = 0; i < P.cols(); i++) {
            if (maxConsensusSet[i]) {
                Pfinal.col(k) = P.col(i);
                Qfinal.col(k) = Q.col(i);
                k++;
            }
        }
        pose_function(Pfinal, Qfinal, T);
        maxConsensus = consensus_function(P, Q, T, maxConsensusSet);

//        int lastConsensus = maxConsensus;
//        Eigen::Isometry3d Tlast = T;
//        do {
//            ROS_INFO("  cons %d", maxConsensus);
//            if (maxConsensus <= minCorrespondenceCount || lastConsensus > maxConsensus) {
//                break;
//            }

//            T = Tlast;

//            Eigen::MatrixXd Pfinal(3, maxConsensus);
//            Eigen::MatrixXd Qfinal(3, maxConsensus);
//            lastConsensus = maxConsensus;
//            int k = 0;
//            for (int i = 0; i < P.cols(); i++) {
//                if (maxConsensusSet[i]) {
//                    Pfinal.col(k) = P.col(i);
//                    Qfinal.col(k) = Q.col(i);
//                    k++;
//                }
//            }
//            pose_function(Pfinal, Qfinal, Tlast);
//            maxConsensus = consensus_function(P, Q, Tlast, maxConsensusSet);
//        } while (lastConsensus != maxConsensus);

        for (int i = 0; i < P.cols(); i++) {
            if (maxConsensusSet[i]) {
                mse += (T * P.col(i).homogeneous() - Q.col(i)).norm();
            }
        }
        mse /= maxConsensus;
    } else {
        maxConsensus = 0;
        T = Eigen::Isometry3d::Identity();
    }

    consensus = maxConsensus;
}

void FeatureTransformationEstimator::estimatePoseEPNP(Eigen::MatrixXd P, Eigen::MatrixXd Q, Eigen::Isometry3d &T)
{
    Eigen::MatrixXd q = Eigen::MatrixXd::Zero(2, Q.cols());
    for (int i = 0; i < q.cols(); i++) {
        q(0,i) = pnp.uc + pnp.fu * Q(0, i) / Q(2, i);
        q(1,i) = pnp.vc + pnp.fv * Q(1, i) / Q(2, i);
    }

    pnp.reset_correspondences();
    for (int j = 0; j < P.cols(); j++) {
        pnp.add_correspondence(P(0,j), P(1,j), P(2,j), q(0,j), q(1,j));
    }

    pnp.compute_pose(T);
}

void FeatureTransformationEstimator::estimatePoseEPNP2D(Eigen::MatrixXd P, Eigen::MatrixXd q, Eigen::Isometry3d &T)
{
    pnp.reset_correspondences();
    for (int j = 0; j < P.cols(); j++) {
        pnp.add_correspondence(P(0,j), P(1,j), P(2,j), q(0,j), q(1,j));
    }

    pnp.compute_pose(T);
}

void FeatureTransformationEstimator::estimatePoseSVD(Eigen::MatrixXd P, Eigen::MatrixXd Q, Eigen::Isometry3d &T)
{
    pcl::TransformationFromCorrespondences tfc;
    for (int i = 0; i < P.cols(); i++) {
        Eigen::Vector3f p_i = P.col(i).cast<float>();
        Eigen::Vector3f q_i = Q.col(i).cast<float>();
        float inverse_weight = p_i(2)*p_i(2) + q_i(2)*q_i(2);
        float weight = 1;
        if (inverse_weight > 0) {
            weight = 1. / weight;
        }
        tfc.add(p_i, q_i, weight);
    }
    T.matrix() = tfc.getTransformation().matrix().cast<double>();
//    T.matrix() = Eigen::umeyama(P, Q, false);
}

int FeatureTransformationEstimator::consensusReprojection(Eigen::MatrixXd P, Eigen::MatrixXd Q, Eigen::Isometry3d T, double thresh, Eigen::Array<bool, 1, Eigen::Dynamic> &consensusSet)
{
    Eigen::MatrixXd Pproj(2, P.cols());
    Eigen::MatrixXd Qproj(2, P.cols());
    for (int i = 0; i < P.cols(); i++) {
        Eigen::Vector3d PT = T * P.col(i).homogeneous();
        Pproj(0,i) = pnp.uc + pnp.fu * PT(0) / PT(2);
        Pproj(1,i) = pnp.vc + pnp.fv * PT(1) / PT(2);

        Qproj(0,i) = pnp.uc + pnp.fu * Q(0,i) / Q(2,i);
        Qproj(1,i) = pnp.vc + pnp.fv * Q(1,i) / Q(2,i);
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
