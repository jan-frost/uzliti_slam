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

#include <graph_slam_tools/transformation/laser_transformation_estimator.h>

#include <chrono>
#include <graph_slam_tools/conversions.h>
#include <fstream>

using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::microseconds;
using std::chrono::system_clock;

LaserTransformationEstimator::LaserTransformationEstimator(boost::function<void (SlamEdge)> callback) :
    TransformationEstimator(callback)
{
    // Maximum angular displacement between scans
      input_.max_angular_correction_deg =  1.5 * 30.0;

      // Maximum translation between scans (m)
      input_.max_linear_correction = 1.5 * 1.0;

      // Maximum ICP cycle iterations
      input_.max_iterations = 10;

      // A threshold for stopping (m)
      input_.epsilon_xy = 0.01;

      // A threshold for stopping (rad)
      input_.epsilon_theta = 0.02;

      // Maximum distance for a correspondence to be valid
      input_.max_correspondence_dist = 0.3;

      // Noise in the scan (m)
      input_.sigma = 0.050;

      // Use smart tricks for finding correspondences.
      input_.use_corr_tricks = 0;

      // Restart: Restart if error is over threshold
      input_.restart = 0;

      // Restart: Threshold for restarting
      input_.restart_threshold_mean_error = 0.01;

      // Restart: displacement for restarting. (m)
      input_.restart_dt = 1.0;

      // Restart: displacement for restarting. (rad)
      input_.restart_dtheta = 0.1;

      // Max distance for staying in the same clustering
      input_.clustering_threshold = 0.25;

      // Number of neighbour rays used to estimate the orientation
      input_.orientation_neighbourhood = 5;

      // If 0, it's vanilla ICP
      input_.use_point_to_line_distance = 1;

      // Discard correspondences based on the angles
      input_.do_alpha_test = 0;

      // Discard correspondences based on the angles - threshold angle, in degrees
      input_.do_alpha_test_thresholdDeg = 45.0;

      // Percentage of correspondences to consider: if 0.9,
      // always discard the top 10% of correspondences with more error
      input_.outliers_maxPerc = 0.80;

      // Parameters describing a simple adaptive algorithm for discarding.
      //  1) Order the errors.
      //  2) Choose the percentile according to outliers_adaptive_order.
      //     (if it is 0.7, get the 70% percentile)
      //  3) Define an adaptive threshold multiplying outliers_adaptive_mult
      //     with the value of the error at the chosen percentile.
      //  4) Discard correspondences over the threshold.
      //  This is useful to be conservative; yet remove the biggest errors.
      input_.outliers_adaptive_order = 0.7;
      input_.outliers_adaptive_mult = 2.0;

      // If you already have a guess of the solution, you can compute the polar angle
      // of the points of one scan in the new position. If the polar angle is not a monotone
      // function of the readings index, it means that the surface is not visible in the
      // next position. If it is not visible, then we don't use it for matching.
      input_.do_visibility_test = 0;

      // no two points in laser_sens can have the same corr.
      input_.outliers_remove_doubles = 1;

      // If 1, computes the covariance of ICP using the method http://purl.org/censi/2006/icpcov
      input_.do_compute_covariance = 1;

      // Checks that find_correspondences_tricks gives the right answer
      input_.debug_verify_tricks = 0;

      // If 1, the field 'true_alpha' (or 'alpha') in the first scan is used to compute the
      // incidence beta, and the factor (1/cos^2(beta)) used to weight the correspondence.");
      input_.use_ml_weights = 0;

      // If 1, the field 'readings_sigma' in the second scan is used to weight the
      // correspondence by 1/sigma^2
      input_.use_sigma_weights = 1;

      input_.min_reading = 0;
      input_.max_reading = 150.;

      sm_debug_write(false);

      ros::NodeHandle nh;
      from_pub_ = nh.advertise<visualization_msgs::Marker>("from_marker", 10);
      to_pub_ = nh.advertise<visualization_msgs::Marker>("to_marker", 10);
      do_near_ = false;
}

bool LaserTransformationEstimator::estimateEdgeImpl(SlamNode &from, SlamNode &to, SlamEdge &edge)
{
    auto start = system_clock::now();
    std::vector<SensorDataPtr> sensor_data_from = from.sensor_data_;
    std::vector<SensorDataPtr> sensor_data_to = to.sensor_data_;

    for (auto data_from : sensor_data_from) {
        if (data_from->type_ == graph_slam_msgs::SensorData::SENSOR_TYPE_LASERSCAN) {
            for (auto data_to : sensor_data_to) {
                if (data_to->type_ == graph_slam_msgs::SensorData::SENSOR_TYPE_LASERSCAN) {
                    LaserscanDataPtr laser_data_from = boost::dynamic_pointer_cast<LaserscanData>(data_from);
                    LaserscanDataPtr laser_data_to = boost::dynamic_pointer_cast<LaserscanData>(data_to);

                    // Construct point cloud.
                    Eigen::Affine3d T_diff = sensor_transforms_[laser_data_from->sensor_frame_].inverse() *
                            laser_data_from->displacement_.inverse() *
                            from.pose_.inverse() * to.pose_ *
                            laser_data_to->displacement_ *
                            sensor_transforms_[laser_data_to->sensor_frame_];

                    Eigen::Affine3d T = T_diff;
                    Eigen::MatrixXd link_information = Eigen::MatrixXd::Identity(6,6);

                    // Estimate CSM tranform. Watch out: CSM dows not seem to be thread safe!
                    estimation_mutex_.lock();
                    auto runtime_start = system_clock::now();
                    double match_score = estimateTransform(laser_data_from->scan_, laser_data_to->scan_, T, link_information);
                    double runtime = (0.001 * duration_cast<microseconds>(system_clock::now() - runtime_start).count());
                    std::ofstream of;
                    of.open("/home/jan/Data/SLAM/runtimes/scan_edge_estimation.txt", std::ios_base::out | std::ios_base::app);
                    of << std::setprecision(16) << ros::Time::now() << " " << runtime << std::endl;
                    estimation_mutex_.unlock();

                    Eigen::Affine3d diff = T_diff.inverse() * T;
                    Eigen::AngleAxisd diff_angle_axis(diff.linear());
                    double diff_rotation = fabs(diff_angle_axis.angle()) * 180 / M_PI;
                    if (match_score == 0 || 1.5 * diff.translation().norm() > input_.max_linear_correction || 1.5 * diff_rotation > input_.max_angular_correction_deg) {
                        ROS_DEBUG_NAMED("laser_transformation", "laser transformation difference too large %f / %f", diff.translation().norm(), diff_rotation);
                        return false;
                    }


                    auto deg = laser_data_from->scan_.angle_min;
                    float num_matches = 0;
                    std::set<float> matched;
                    auto data_from = laser_data_from->scan_.ranges;
                    auto data_to = laser_data_to->scan_.ranges;
                    if (!do_near_) {
                        data_from = laser_data_from->scan_.intensities;
                        data_to = laser_data_to->scan_.intensities;
                    }
                    for (auto r : data_from) {
                        if (!isnan(r) && r > laser_data_from->scan_.range_min && r < laser_data_from->scan_.range_max) {
                            Eigen::Vector3f point = (from.pose_ * laser_data_from->displacement_).cast<float>() *
                                    Eigen::Vector3f(std::cos(deg) * r, std::sin(deg) * r, 0).homogeneous();
                            auto deg2 = laser_data_to->scan_.angle_min;
                            auto min_deg = deg2;
                            float min_match = input_.max_correspondence_dist*input_.max_correspondence_dist;
                            bool found = false;
                            for (auto r2 : data_to) {
                                if (!isnan(r2) && r2 > laser_data_to->scan_.range_min && r2 < laser_data_to->scan_.range_max && matched.find(deg2) == matched.end()) {
                                    Eigen::Vector3f point2 = ((from.pose_ * laser_data_from->displacement_) * T ).cast<float>()*
                                            Eigen::Vector3f(std::cos(deg2) * r2, std::sin(deg2) * r2, 0).homogeneous();
                                    if ((point - point2).squaredNorm() < min_match) {
                                        min_match = (point - point2).squaredNorm();
                                        min_deg = deg2;
                                        found = true;
                                    }
                                }
                                deg2 +=laser_data_to->scan_.angle_increment;
                            }
                            if (found) {
                                num_matches++;
                                matched.insert(min_deg);
                            }
                        }
                        deg +=laser_data_from->scan_.angle_increment;
                    }
                    ROS_DEBUG_NAMED("laser_transformation", "match scores: %f / %f", match_score, num_matches);

                    // Initialize to edge.
                    edge.id_from_ = from.id_;
                    edge.id_to_ = to.id_;
                    edge.transform_.matrix() = T.matrix();
                    edge.information_ = link_information;
                    edge.type_ = graph_slam_msgs::Edge::TYPE_2D_LASER;
                    edge.sensor_from_ = laser_data_from->sensor_frame_;
                    edge.sensor_to_ = laser_data_to->sensor_frame_;
                    edge.displacement_from_ = laser_data_from->displacement_;
                    edge.displacement_to_ = laser_data_to->displacement_;
                    edge.matching_score_ = match_score;

                    // Plot scans.
                    if (edge.matching_score_ >= 0) {
                        if (from_pub_.getNumSubscribers() > 0) {
                            visualization_msgs::Marker marker_from;
                            marker_from.header.frame_id = "/map";
                            marker_from.id = 0;
                            marker_from.type = visualization_msgs::Marker::SPHERE_LIST;
                            marker_from.action = visualization_msgs::Marker::ADD;
                            marker_from.scale.x = 0.1;
                            marker_from.scale.y = 0.1;
                            marker_from.scale.z = 0.1;
                            marker_from.color.a = 0.8;
                            marker_from.color.r = 0.0;
                            marker_from.color.g = 1.0;
                            marker_from.color.b = 0.0;
                            auto deg = laser_data_from->scan_.angle_min;
                            for (auto r : data_from) {
                                if (!isnan(r) && laser_data_from->scan_.range_max > 0 && r < laser_data_from->scan_.range_max) {
                                    Eigen::Vector3d point = (from.pose_ * laser_data_from->displacement_) *
                                            Eigen::Vector3d(std::cos(deg) * r, std::sin(deg) * r, 0).homogeneous();

                                    geometry_msgs::Point p;
                                    p.x = point(0);
                                    p.y = point(1);
                                    p.z = point(2);
                                    marker_from.points.push_back(p);
                                }
                                deg +=laser_data_from->scan_.angle_increment;
                            }
                            from_pub_.publish(marker_from);
                        }

                        if (to_pub_.getNumSubscribers()) {
                            visualization_msgs::Marker marker_to;
                            marker_to.header.frame_id = "/map";
                            marker_to.id = 0;
                            marker_to.type = visualization_msgs::Marker::SPHERE_LIST;
                            marker_to.action = visualization_msgs::Marker::ADD;
                            marker_to.scale.x = 0.1;
                            marker_to.scale.y = 0.1;
                            marker_to.scale.z = 0.1;
                            marker_to.color.a = 0.8;
                            marker_to.color.r = 1.0;
                            marker_to.color.g = 0.0;
                            marker_to.color.b = 0.0;
                            auto deg = laser_data_to->scan_.angle_min;
                            for (auto r : data_to) {
                                if (!isnan(r) && laser_data_to->scan_.range_max > 0 && r < laser_data_to->scan_.range_max) {
                                    Eigen::Vector3d point = (from.pose_ * laser_data_from->displacement_) * edge.transform_ *
                                            Eigen::Vector3d(std::cos(deg) * r, std::sin(deg) * r, 0).homogeneous();

                                    geometry_msgs::Point p;
                                    p.x = point(0);
                                    p.y = point(1);
                                    p.z = point(2);
                                    marker_to.points.push_back(p);
                                }
                                deg +=laser_data_to->scan_.angle_increment;
                            }
                            to_pub_.publish(marker_to);
                        }
                    }

                    ROS_DEBUG_NAMED("laser_transformation", "time: %f ms", 0.001 * duration_cast<microseconds>(system_clock::now() - start).count());
                    return true;
                }
            }
        }
    }

    return false;
}

double LaserTransformationEstimator::estimateTransform(const sensor_msgs::LaserScan &from, const sensor_msgs::LaserScan &to, Eigen::Affine3d &T, Eigen::MatrixXd &link_information)
{
    sm_result output;

    LDP from_ldp_scan;
    laserScanToLDP(from, from_ldp_scan);
    LDP to_ldp_scan;
    laserScanToLDP(to, to_ldp_scan);

    // CSM is used in the following way:
    // The scans are always in the laser frame
    // The reference scan (prevLDPcan_) has a pose of [0, 0, 0]
    // The new scan (currLDPScan) has a pose equal to the movement
    // of the laser in the laser frame since the last scan
    // The computed correction is then propagated using the tf machinery

    from_ldp_scan->odometry[0] = 0.0;
    from_ldp_scan->odometry[1] = 0.0;
    from_ldp_scan->odometry[2] = 0.0;

    from_ldp_scan->estimate[0] = 0.0;
    from_ldp_scan->estimate[1] = 0.0;
    from_ldp_scan->estimate[2] = 0.0;

    from_ldp_scan->true_pose[0] = 0.0;
    from_ldp_scan->true_pose[1] = 0.0;
    from_ldp_scan->true_pose[2] = 0.0;

    input_.laser_ref  = from_ldp_scan;
    input_.laser_sens = to_ldp_scan;

    input_.first_guess[0] = T.translation()(0);
    input_.first_guess[1] = T.translation()(1);
    input_.first_guess[2] = g2o::internal::toEuler(T.linear())(2);

    // *** scan match - using point to line icp from CSM

    sm_icp(&input_, &output);

    double res = 0.;
    if (output.valid) {
        ROS_DEBUG_NAMED("laser_transformation", "valid transform: %d matches", output.nvalid);
        int last_corr = -1;
        int deg_count = 0;
        int scan_valid = 0;
        for (int i = 0; i < input_.laser_sens->nrays; i++) {
            if (input_.laser_sens->valid[i]) {
                scan_valid++;
            }

            correspondence c = input_.laser_sens->corr[i];
            if (c.valid) {
                if (c.j1 > last_corr) {
                    deg_count++;
                } else if (c.j1 < last_corr) {
                    deg_count--;
                }
                last_corr = c.j1;
            }
        }
        ROS_DEBUG_NAMED("laser_transformation", "viewpoint score: %d", deg_count);

        if (deg_count > 0) {
            Eigen::Matrix3d cov;
            for (int i = 0; i < output.cov_x_m->rows(); i++) {
                for (int j = 0; j < output.cov_x_m->cols(); j++) {
                    cov(i,j) = gsl_matrix_get(output.cov_x_m, i, j);
                }
            }
            Eigen::Matrix3d inf3 = cov.inverse();
            double goal_trace = 10000.;
            double scale_factor = goal_trace / inf3.trace();
            inf3 *= scale_factor;
//            if (inf3(0,0) < 0.01 * goal_trace) {
//                inf3(0,0) = 0;
//            }

            link_information = 100. * Eigen::MatrixXd::Identity(6,6);
            link_information(0,0) = inf3(0,0);
            link_information(0,1) = inf3(0,1);
            link_information(1,1) = inf3(1,1);
            link_information(1,0) = inf3(1,0);
            link_information(5,5) = inf3(2,2);

            Eigen::Translation3d trans(output.x[0], output.x[1], 0.);
            Eigen::AngleAxisd rot(output.x[2], Eigen::Vector3d::UnitZ());
            T = trans * rot;
            res = output.nvalid;

            if (0.25 * scan_valid > output.nvalid) {
                ROS_DEBUG_NAMED("laser_transformation", "percentage of matches too low");
                res = 0;
            }

            gsl_matrix_free(output.cov_x_m);
            gsl_matrix_free(output.dx_dy1_m);
            gsl_matrix_free(output.dx_dy2_m);
        }
    }

    csm_free_unused_memory();
    ld_free(from_ldp_scan);
    ld_free(to_ldp_scan);
    return res;
}

void LaserTransformationEstimator::laserScanToLDP(const sensor_msgs::LaserScan &scan_msg, LDP& ldp)
{
    unsigned int n = scan_msg.intensities.size();
    ldp = ld_alloc_new(n);

    int count = 0;
    for (unsigned int i = 0; i < n; i++) {
        // calculate position in laser frame
        double r = 0.0;
        if (do_near_) {
            r = scan_msg.ranges[i];
        } else {
            r = scan_msg.intensities[i];
        }

        if (r >= scan_msg.range_min && r <= scan_msg.range_max) {
            // fill in laser scan data

            ldp->valid[i] = 1;
            ldp->readings[i] = r;
            ldp->readings_sigma[i] = r;
            count++;
        } else {
            ldp->valid[i] = 0;
            ldp->readings[i] = -1;  // for invalid range
            ldp->readings_sigma[i] = r;
        }

        ldp->theta[i]    = scan_msg.angle_min + i * scan_msg.angle_increment;

        ldp->cluster[i]  = -1;
    }

    ldp->min_theta = ldp->theta[0];
    ldp->max_theta = ldp->theta[n-1];

    ldp->odometry[0] = 0.0;
    ldp->odometry[1] = 0.0;
    ldp->odometry[2] = 0.0;

    ldp->true_pose[0] = 0.0;
    ldp->true_pose[1] = 0.0;
    ldp->true_pose[2] = 0.0;
}


