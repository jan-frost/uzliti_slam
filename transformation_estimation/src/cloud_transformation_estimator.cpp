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

#include <transformation_estimation/cloud_transformation_estimator.h>

#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <sensor_msgs/image_encodings.h>
#include <gicp6d/gicp6d.h>
#include <graph_slam_common/conversions.h>

using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::system_clock;

CloudTransformationEstimator::CloudTransformationEstimator(boost::function<void (SlamEdge)> callback) :
    TransformationEstimator(callback),
    viewer("after")
{
}

bool CloudTransformationEstimator::estimateEdgeImpl(SlamNode &from, SlamNode &to, SlamEdge &edge)
{
    auto start = system_clock::now();
    std::vector<SensorDataPtr> sensor_data_from = from.sensor_data_;
    std::vector<SensorDataPtr> sensor_data_to = to.sensor_data_;

    for (auto data_from : sensor_data_from) {
        if (data_from->type_ == graph_slam_msgs::SensorData::SENSOR_TYPE_DEPTH_IMAGE) {
            for (auto data_to : sensor_data_to) {
                if (data_to->type_ == graph_slam_msgs::SensorData::SENSOR_TYPE_DEPTH_IMAGE) {
                    DepthImageDataPtr depth_data_from = boost::dynamic_pointer_cast<DepthImageData>(data_from);
                    DepthImageDataPtr depth_data_to = boost::dynamic_pointer_cast<DepthImageData>(data_to);

                    // Construct point cloud.
                    Eigen::Affine3d T_diff = sensor_transforms_[depth_data_from->sensor_frame_].inverse() *
                            depth_data_from->displacement_.inverse() *
                            from.pose_.inverse() * to.pose_ *
                            depth_data_to->displacement_ *
                            sensor_transforms_[depth_data_to->sensor_frame_];

                    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_from_colored = Conversions::toPointCloudColor(Eigen::Isometry3d::Identity(), depth_data_from);
                    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_to_colored = Conversions::toPointCloudColor(Eigen::Isometry3d::Identity(), depth_data_to);

                    Eigen::Affine3d T = T_diff;
                    double match_score = estimateTransform(cloud_from_colored, cloud_to_colored, T);

                    if (match_score > 0.3) {
                        Eigen::Affine3d T_change = T_diff * T.inverse();
                        Eigen::AngleAxisd diffAngleAxis(T_change.linear());
                        double diffRotation = fabs(diffAngleAxis.angle()) * 180 / M_PI;
                        if (T_change.translation().norm() <= 1.0 && diffRotation <= 30.) {
                            Eigen::MatrixXd link_covariance = Eigen::MatrixXd::Zero(6,6);
                            link_covariance(0, 0) = 0.01 * 0.01;
                            link_covariance(1, 1) = link_covariance(0, 0);
                            link_covariance(2, 2) = link_covariance(0, 0);
                            link_covariance(3, 3) = 0.001;
                            link_covariance(3, 3) *= link_covariance(3, 3);
                            link_covariance(4, 4) = link_covariance(3, 3);
                            link_covariance(5, 5) = link_covariance(3, 3);

                            // Initialize to edge.
                            edge.id_from_ = from.id_;
                            edge.id_to_ = to.id_;
                            edge.transform_.matrix() = T.matrix();
                            edge.information_ = link_covariance.inverse();
                            edge.type_ = graph_slam_msgs::Edge::TYPE_3D_FULL;
                            edge.sensor_from_ = depth_data_from->sensor_frame_;
                            edge.sensor_to_ = depth_data_to->sensor_frame_;
                            edge.displacement_from_ = depth_data_from->displacement_;
                            edge.displacement_to_ = depth_data_to->displacement_;
                            edge.matching_score_ = 1.0;

                            ROS_INFO("CLOUD TRANSFORMATION: %d ms", (int)duration_cast<milliseconds>(system_clock::now() - start).count());
                            return true;
                        }
                    } else {
                        ROS_DEBUG("cloud transformation estimation failed (%f match score)", match_score);
                    }
                }
            }
        }
    }

    return false;
}

void CloudTransformationEstimator::transformPointCloudInPlace(Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> Tr, pcl::PointCloud<pcl::PointXYZRGBA> &pc) {
    Eigen::Transform<float,3,Eigen::Affine,Eigen::ColMajor> T = Tr.cast<float>();
    for(unsigned int pit=0; pit<pc.points.size(); ++pit) {
    Eigen::Map<Eigen::Vector3f> pt((float*)&pc.points[pit],3);
    pt = T*pt;
    }
}

double CloudTransformationEstimator::estimateTransform(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &from, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &to, Eigen::Affine3d &T_init)
{

    // Apply Voxelgrid filter.
    pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
    sor.setLeafSize(0.05f, 0.05f, 0.05f);
    sor.setFilterFieldName("z");
    sor.setFilterLimits(0.0, 5.0);

    sor.setInputCloud(from);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr from_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>());
    sor.filter(*from_filtered);

    sor.setInputCloud(to);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr to_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>());
    sor.filter(*to_filtered);

//    // Apply passthrough filter.
//    pcl::PassThrough<pcl::PointXYZRGBA> pass;
//    pass.setFilterFieldName("y");
//    pass.setFilterLimits(-1.0, 0.5);

//    pass.setInputCloud(from_filtered);
//    pass.filter(*from_filtered);
//    pass.setInputCloud(to_filtered);
//    pass.filter(*to_filtered);

    double corr_count = 0.;
    transformPointCloudInPlace(T_init, *to_filtered);

    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    pcl::GeneralizedIterativeClosestPoint6D gicp6d(0.024);
    gicp6d.setInputSource(from_filtered);
    gicp6d.setInputTarget(to_filtered);
    gicp6d.setMaxCorrespondenceDistance(0.2);
    gicp6d.setMaximumIterations(20);
    pcl::PointCloud<pcl::PointXYZRGBA> Final_6d;
    gicp6d.align(Final_6d);
    T = gicp6d.getFinalTransformation();
    corr_count = ((double)gicp6d.getNumberOfCorrespondences() / std::max((double)to_filtered->points.size(), (double)from_filtered->points.size()));

    T_init = Eigen::Affine3d(T.cast<double>()).inverse() * T_init;

    transformPointCloudInPlace(Eigen::Affine3d(T.cast<double>()).inverse(), *to_filtered);
    viewer.showCloud(from_filtered, "from");
    viewer.showCloud(to_filtered, "to_registered");
    return corr_count;
}
