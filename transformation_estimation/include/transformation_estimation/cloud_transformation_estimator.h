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

#ifndef CLOUD_TRANSFORMATION_ESTIMATION_H
#define CLOUD_TRANSFORMATION_ESTIMATION_H

#include <transformation_estimation/transformation_estimator.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

class CloudTransformationEstimator : public TransformationEstimator
{
public:
    CloudTransformationEstimator(boost::function<void (SlamEdge)> callback);

    bool estimateEdgeImpl(SlamNode &from, SlamNode &to, SlamEdge &edge);

    std::map<std::string, Eigen::Isometry3d, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string, Eigen::Isometry3d> > > sensor_transforms_;

protected:
    void transformPointCloudInPlace(Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> Tr, pcl::PointCloud<pcl::PointXYZRGBA> &pc);
    double estimateTransform(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &from, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &to, Eigen::Affine3d &T);

    pcl::visualization::CloudViewer viewer;
};

#endif
