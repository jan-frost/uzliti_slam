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

#include "graph_slam_tools/conversions.h"
#include <boost/cast.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

Eigen::MatrixXd Conversions::featureMsgToFeaturePositions(const graph_slam_msgs::Features &featureMsg)
{
    if (featureMsg.features.size() < 1) {
        return Eigen::MatrixXd();
    }

    // Read all feature positions from the feature message.
    Eigen::MatrixXd positions(3, featureMsg.features.size());
    for (unsigned int i = 0; i < featureMsg.features.size(); i++) {
        geometry_msgs::Point point = featureMsg.features[i].keypoint_position;
        positions(0,i) = point.x;
        positions(1,i) = point.y;
        positions(2,i) = point.z;
    }
    return positions;
}

geometry_msgs::PoseWithCovariance Conversions::toMsg(const Eigen::Isometry3d &pose, const Eigen::MatrixXd &Sigma)
{
    // Store the covariance in a ROS pose message.
    geometry_msgs::PoseWithCovariance pwc;
    pwc.pose = toMsg(pose);
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            pwc.covariance[6*i + j] = Sigma(i, j);
        }
    }
    return pwc;
}

geometry_msgs::Pose Conversions::toMsg(const Eigen::Isometry3d &pose)
{
    // Store the pose in a ROS pose message.
    Eigen::Vector3d position = pose.translation();
    Eigen::Quaterniond orientation(pose.linear());
    geometry_msgs::Pose poseMsg;
    poseMsg.position.x = position(0);
    poseMsg.position.y = position(1);
    poseMsg.position.z = position(2);
    poseMsg.orientation.w = orientation.w();
    poseMsg.orientation.x = orientation.x();
    poseMsg.orientation.y = orientation.y();
    poseMsg.orientation.z = orientation.z();
    return poseMsg;
}

std::vector<graph_slam_msgs::SensorData> Conversions::toMsg(const std::vector<SensorDataPtr> &sensor_data)
{
    std::vector<graph_slam_msgs::SensorData> sensor_data_msg;
    for (auto data : sensor_data) {
        sensor_data_msg.push_back(data->toMsg());
    }
    return sensor_data_msg;
}

std::vector<graph_slam_msgs::SensorData> Conversions::toMsg(const std::vector<SensorDataPtr> &sensor_data, int sensor_data_type)
{
    std::vector<graph_slam_msgs::SensorData> sensor_data_msg;
    for (auto data : sensor_data) {
        if (data->type_ == sensor_data_type) {
            sensor_data_msg.push_back(data->toMsg());
        }
    }
    return sensor_data_msg;
}

void Conversions::fromMsg(const std::vector<graph_slam_msgs::SensorData> &sensor_data, cv::Mat &features)
{
    // Count feature data points. Assumption: all have the same feature type.
    int feature_type = 0;
    int feature_count = 0;
    int descriptor_size = 0;
    for (auto data : sensor_data) {
        if (data.sensor_type == graph_slam_msgs::SensorData::SENSOR_TYPE_FEATURE) {
            feature_type = data.features.descriptor_type;
            if (data.features.features.size() > 0) {
                descriptor_size = data.features.features[0].descriptor.size();
                feature_count += data.features.features.size();
            }
        }
    }

    if (feature_count > 0) {
        int k = 0;
        switch (feature_type) {
        case graph_slam_msgs::Features::BRIEF:
        case graph_slam_msgs::Features::ORB:
        case graph_slam_msgs::Features::BRISK:
        case graph_slam_msgs::Features::FREAK:
            features.create(feature_count, descriptor_size, CV_8U);
            for (auto data : sensor_data) {
                if (data.sensor_type == graph_slam_msgs::SensorData::SENSOR_TYPE_FEATURE) {
                    for (unsigned int i = 0; i < data.features.features.size(); i++) {
                        for (int j = 0; j < descriptor_size; j++) {
                            float val = data.features.features[i].descriptor[j];
                            features.at<unsigned char>(k,j) = (unsigned char)val;
                        }
                        k++;
                    }
                }
            }
            break;
        default:
            ROS_ERROR_NAMED("feature_link_estimation", "unknown feature type: %d (LE)", feature_type);
            break;
        }
    }
}

std::vector<SensorDataPtr> Conversions::fromMsg(const graph_slam_msgs::SensorDataArray &sensor_data_msg)
{
    std::vector<SensorDataPtr> sensor_data_array;
    for (auto data : sensor_data_msg.data) {
        sensor_data_array.push_back(fromMsg(data));
    }
    return sensor_data_array;
}

SensorDataPtr Conversions::fromMsg(const graph_slam_msgs::SensorData &sensor_data_msg)
{
    if (sensor_data_msg.sensor_type == graph_slam_msgs::SensorData::SENSOR_TYPE_FEATURE) {
        FeatureDataPtr feature_data(new FeatureData());
        feature_data->fromMsg(sensor_data_msg);
        return feature_data;
    } else if (sensor_data_msg.sensor_type == graph_slam_msgs::SensorData::SENSOR_TYPE_DEPTH_IMAGE) {
        DepthImageDataPtr depth_data(new DepthImageData());
        depth_data->fromMsg(sensor_data_msg);
        return depth_data;
    } else if (sensor_data_msg.sensor_type == graph_slam_msgs::SensorData::SENSOR_TYPE_BINARY_GIST) {
        BinaryGistDataPtr gist_data(new BinaryGistData());
        gist_data->fromMsg(sensor_data_msg);
        return gist_data;
    } else if (sensor_data_msg.sensor_type == graph_slam_msgs::SensorData::SENSOR_TYPE_LASERSCAN) {
        LaserscanDataPtr laser_data(new LaserscanData());
        laser_data->fromMsg(sensor_data_msg);
        return laser_data;
    } else {
        SensorDataPtr unknown_data(new SensorData());
        unknown_data->fromMsg(sensor_data_msg);
        return unknown_data;
    }
}

graph_slam_msgs::Graph Conversions::toMsg(SlamGraph &graph, bool add_sensor_data)
{
    graph_slam_msgs::Graph graph_msg;
    graph_msg.header.stamp = ros::Time::now();
    graph_msg.header.frame_id = graph.frame();

    // Add nodes.
    for (auto node_it = graph.nodeIterator(); node_it.first != node_it.second; ++node_it.first) {
        graph_msg.nodes.push_back(Conversions::toMsg(node_it.first->second, add_sensor_data));
    }

    // Add links.
    for (auto edge_it = graph.edgeIterator(); edge_it.first != edge_it.second; ++edge_it.first) {
        graph_msg.edges.push_back(Conversions::toMsg(edge_it.first->second));
    }

    // Add odometry parameters.
    for (int i = 0; i < 6; i++) {
        graph_msg.odometry_parameters[i] = graph.odom()(i);
    }

    // Add sensor transformations.
    for (auto sensor_it = graph.sensorIterator(); sensor_it.first != sensor_it.second; ++sensor_it.first) {
        graph_slam_msgs::SensorTransform sensor_tranform_msg;
        sensor_tranform_msg.sensor_name = sensor_it.first->first;
        sensor_tranform_msg.transform = Conversions::toMsg(sensor_it.first->second);
        graph_msg.sensor_tranforms.push_back(sensor_tranform_msg);
    }

    return graph_msg;
}

void Conversions::fromMsg(const geometry_msgs::PoseWithCovariance &poseMsg, Eigen::Isometry3d &pose, Eigen::MatrixXd &covariance)
{
    pose = fromMsg(poseMsg.pose);

    covariance.resize(6,6);
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            covariance(i,j) = poseMsg.covariance[6*i + j];
        }
    }
}

Eigen::Isometry3d Conversions::fromMsg(const geometry_msgs::Pose &poseMsg)
{
    g2o::Vector7d v;
    v << poseMsg.position.x,
            poseMsg.position.y,
            poseMsg.position.z,
            poseMsg.orientation.x,
            poseMsg.orientation.y,
            poseMsg.orientation.z,
            poseMsg.orientation.w;
    Eigen::Isometry3d T = g2o::internal::fromVectorQT(v);
    return T;
}

SlamEdge Conversions::fromMsg(const graph_slam_msgs::Edge &edge_msg)
{
    Eigen::Isometry3d T;
    Eigen::MatrixXd I;
    Conversions::fromMsg(edge_msg.transformation, T, I);

    SlamEdge edge = SlamEdge(edge_msg.id, edge_msg.id_from, edge_msg.id_to, T, I, edge_msg.type, edge_msg.sensor_from, edge_msg.sensor_to);
    edge.error_ = edge_msg.error;
    edge.age_ = edge_msg.age;
    edge.matching_score_ = edge_msg.matching_score;
    return edge;
}

graph_slam_msgs::Edge Conversions::toMsg(const SlamEdge &edge)
{
    graph_slam_msgs::Edge edge_msg;
    edge_msg.id = edge.id_;
    edge_msg.type = edge.type_;
    edge_msg.id_from = edge.id_from_;
    edge_msg.id_to = edge.id_to_;
    edge_msg.transformation = toMsg(edge.transform_, edge.information_);
    edge_msg.error = edge.error_;
    edge_msg.matching_score = edge.matching_score_;
    edge_msg.sensor_from = edge.sensor_from_;
    edge_msg.sensor_to = edge.sensor_to_;
    edge_msg.error = edge.error_;
    edge_msg.age = edge.age_;
    return edge_msg;
}

SlamNode Conversions::fromMsg(const graph_slam_msgs::Node &node_msg, bool copy_sensor_data)
{
    SlamNode node;
    node.stamp_ = node_msg.header.stamp;
    node.id_ = node_msg.id;
    node.pose_ = Conversions::fromMsg(node_msg.pose);
    node.sub_pose_ = Conversions::fromMsg(node_msg.odom_pose);
    node.fixed_ = node_msg.fixed;
    node.is_loop_ = node_msg.is_loop;
    node.is_border_ = node_msg.is_border;
    node.is_in_scope_ = node_msg.is_in_scope;
    for (auto edge : node_msg.edge_ids) {
        node.edges_.insert(edge);
    }

    if (copy_sensor_data) {
        node.sensor_data_ = Conversions::fromMsg(node_msg.sensor_data);
    }

    return node;
}

graph_slam_msgs::Node Conversions::toMsg(const SlamNode &node, bool copy_sensor_data)
{
    graph_slam_msgs::Node node_msg;
    node_msg.header.stamp = node.stamp_;
    node_msg.id = node.id_;
    node_msg.pose = Conversions::toMsg(node.pose_);
    node_msg.odom_pose = Conversions::toMsg(node.sub_pose_);
    node_msg.fixed = node.fixed_;
    node_msg.is_loop = node.is_loop_;
    node_msg.is_border = node.is_border_;
    node_msg.is_in_scope = node.is_in_scope_;
    for (auto edge : node.edges_) {
        node_msg.edge_ids.push_back(edge);
    }

    if (copy_sensor_data) {
        node_msg.sensor_data.data = Conversions::toMsg(node.sensor_data_);
    }

    return node_msg;
}

Eigen::Vector3d Conversions::fromMsg(const geometry_msgs::Point &point_msg)
{
    return Eigen::Vector3d(point_msg.x, point_msg.y, point_msg.z);
}

geometry_msgs::Point Conversions::toMsg(const Eigen::Vector3d &point)
{
    geometry_msgs::Point point_msg;
    point_msg.x = point(0);
    point_msg.y = point(1);
    point_msg.z = point(2);
    return point_msg;
}

graph_slam_msgs::SensorTransform Conversions::toMsg(const std::string &name, const Eigen::Isometry3d transform)
{
    graph_slam_msgs::SensorTransform sensor_transform_msg;
    sensor_transform_msg.sensor_name = name;
    sensor_transform_msg.transform = Conversions::toMsg(transform);
    return sensor_transform_msg;
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Conversions::toPointCloudColor(const Eigen::Isometry3d &T, DepthImageDataPtr depth_data)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
    cv::Mat depth_img;
    cv::Mat color_img;

    cv_bridge::CvImagePtr depth_bridge;
    cv_bridge::CvImagePtr rgb_bridge;
    try {
        depth_bridge = cv_bridge::toCvCopy(depth_data->depth_image_, sensor_msgs::image_encodings::TYPE_32FC1);
        depth_img = depth_bridge->image;
        rgb_bridge = cv_bridge::toCvCopy(depth_data->color_image_, sensor_msgs::image_encodings::BGR8);
        color_img = rgb_bridge->image;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s",e.what());
        return cloud;
    }

    // Get camera info for 3D point estimation.
    image_geometry::PinholeCameraModel cam = depth_data->camera_model_;

    cloud->header.frame_id = depth_data->sensor_frame_;
    cloud->is_dense = false;
    cloud->height = depth_img.rows;
    cloud->width = depth_img.cols;
    cloud->sensor_origin_ = Eigen::Vector4f( T.translation()(0), T.translation()(1), T.translation()(2), 1.f );
    cloud->sensor_orientation_ = Eigen::Quaternionf(T.rotation().cast<float>());
    cloud->points.resize( cloud->height * cloud->width );

    size_t idx = 0;
    const float* depthdata = reinterpret_cast<float*>( &depth_img.data[0] );
    const unsigned char* colordata = &color_img.data[0];
    for(int v = 0; v < depth_img.rows; ++v) {
        for(int u = 0; u < depth_img.cols; ++u) {
            pcl::PointXYZRGBA& p = cloud->points[ idx ]; ++idx;

            float d = (*depthdata++);

            if (d > 0 && !isnan(d)/* && p.z <= 7.5*/) {
                p.z = d;
                p.x = (u - cam.cx()) * d / cam.fx();
                p.y = (v - cam.cy()) * d / cam.fy();

                cv::Point3_<uchar>* rgb = color_img.ptr<cv::Point3_<uchar> >(v,u);
                p.b = rgb->x;
                p.g = rgb->y;
                p.r = rgb->z;
                p.a = 255;
            } else {
                p.x = std::numeric_limits<float>::quiet_NaN();;
                p.y = std::numeric_limits<float>::quiet_NaN();;
                p.z = std::numeric_limits<float>::quiet_NaN();;
                p.rgb = 0;
                colordata += 3;
            }
        }
    }

    return cloud;
}

bool Conversions::getTransform(tf::TransformListener &tfl, Eigen::Isometry3d &T, std::string from, std::string to, ros::Time stamp)
{
    geometry_msgs::Pose transform_msg;
    try {
        tf::StampedTransform t;
        tfl.lookupTransform(from, to, stamp, t);
        transform_msg.orientation.w = t.getRotation().w();
        transform_msg.orientation.x = t.getRotation().x();
        transform_msg.orientation.y = t.getRotation().y();
        transform_msg.orientation.z = t.getRotation().z();
        transform_msg.position.x = t.getOrigin().x();
        transform_msg.position.y = t.getOrigin().y();
        transform_msg.position.z = t.getOrigin().z();
    } catch (tf::LookupException le) {
        ROS_ERROR_ONCE("Couldn't find transformation between %s and %s", from.c_str(), to.c_str());
        return false;
    } catch (tf::ExtrapolationException ee) {
        ROS_ERROR_ONCE("Transformation lookup time is in the future");
        return false;
    }
    T = Conversions::fromMsg(transform_msg);
    return true;
}
