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

#ifndef LASER_TRANSFORMATION_ESTIMATION_H
#define LASER_TRANSFORMATION_ESTIMATION_H

#include <transformation_estimation/transformation_estimator.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>

#include <csm/csm.h>

class LaserTransformationEstimator : public TransformationEstimator
{
public:
    LaserTransformationEstimator(boost::function<void (SlamEdge)> callback);

    bool estimateEdgeImpl(SlamNode &from, SlamNode &to, SlamEdge &edge);

    bool do_near_;

protected:
    double estimateTransform(const sensor_msgs::LaserScan &from, const sensor_msgs::LaserScan &to, Eigen::Affine3d &T, Eigen::MatrixXd &link_information);
    void laserScanToLDP(const sensor_msgs::LaserScan &scan_msg, LDP& ldp);
    sm_params input_;
    ros::Publisher from_pub_;
    ros::Publisher to_pub_;
};

#endif
