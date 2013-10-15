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

#include <graph_slam/graph_slam_node.h>

#include <chrono>
#include <geometry_msgs/PolygonStamped.h>
#include <graph_slam_tools/conversions.h>
#include <graph_slam_tools/optimization/sensor_transform_optimizer.h>
#include <graph_slam_msgs/SensorRequest.h>
#include <graph_slam_msgs/NodePair.h>

#include <g2o/types/sclam2d/odometry_measurement.h>

//! Please note: we cannot include this any earlier, because flann seems to have an ambiguity issue towards g2o.
#include <graph_slam_tools/transformation/cloud_transformation_estimator.h>
#include <graph_slam_tools/transformation/laser_transformation_estimator.h>

namespace mr=mongo_ros;

typedef mr::MessageWithMetadata<graph_slam_msgs::Node> NodeWithMetadata;
typedef NodeWithMetadata::ConstPtr NodeMetaPtr;
typedef mr::MessageWithMetadata<graph_slam_msgs::Edge> EdgeWithMetadata;
typedef EdgeWithMetadata::ConstPtr EdgeMetaPtr;
typedef mr::MessageWithMetadata<graph_slam_msgs::SensorTransform> SensorWithMetadata;
typedef SensorWithMetadata::ConstPtr SensorMetaPtr;

mr::Metadata makeMetadata(const graph_slam_msgs::Node& n)
{
    return mr::Metadata("id", n.id, "x", n.pose.position.x, "y", n.pose.position.y, "z", n.pose.position.z);
}

mr::Metadata makeMetadata(const graph_slam_msgs::Edge& e)
{
    return mr::Metadata("id", e.id, "id_from", e.id_from, "id_to", e.id_to);
}

mr::Metadata makeMetadata(const graph_slam_msgs::SensorTransform& s)
{
    return mr::Metadata("sensor_name", s.sensor_name);
}

using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::system_clock;

GraphSlamNode::GraphSlamNode(ros::NodeHandle nh) :
    _nh(nh),
    graph("/map"),
    place_recognizer_(new BinaryGistRecognizer()),
    optimizer_(new G2oOptimizer()),
    sensor_optimizer_(new SensorTransformOptimizer()),
    transformation_estimator_(new FeatureTransformationEstimator(boost::bind(&GraphSlamNode::newEdgeCallback, this, _1))),
    projector_(new OccupancyGridProjector(nh)),
    fusion_projector_(new FastFusionProjector(nh)),
    tfl(ros::Duration(10.0))
{
    // Read odometry frames.
    nh.param<std::string>("odom_frame", odom_frame_, "odom");
    ROS_INFO("odom_frame: %s", odom_frame_.c_str());
    nh.param<std::string>("odom_childframe", odom_childframe_, "base_link");
    ROS_INFO("odom_childframe: %s", odom_childframe_.c_str());

    current_node_ = "";
    last_node_ = "";
    most_recent_loop_closure_ = "";
    current_odometry_pose_ = Eigen::Isometry3d::Identity();
    last_odometry_pose_ = Eigen::Isometry3d::Identity();
    odom_covariance_ = Eigen::MatrixXd::Zero(6,6);
    odom_information_ = Eigen::MatrixXd::Zero(6,6);
    map_transform_ = Eigen::Isometry3d::Identity();
    odom_transform_.frame_id_ = "/map";

    // Initialize publishers
    pose_pub_ = _nh.advertise<geometry_msgs::PoseStamped>("graph_pose", 1);
    graph_display_pub_ = _nh.advertise<graph_slam_msgs::Graph>("/display_graph", 1);
    sensor_request_pub_ = _nh.advertise<graph_slam_msgs::SensorRequest>("/sensor_request", 1);
    surfelPolyPub = _nh.advertise<geometry_msgs::PolygonStamped>("poly_surfel_first",1);
    surfelPolyPub2 = _nh.advertise<geometry_msgs::PolygonStamped>("poly_surfel_second",1);

    // Initialize synchronized sensor data subscriber.
    sensor_sub_.subscribe(nh, "/sensor_data", 10);
    sensor_sub_filter_ = new tf::MessageFilter<graph_slam_msgs::SensorDataArray>(sensor_sub_, tfl, "/odom", 10);
    sensor_sub_filter_->registerCallback(boost::bind(&GraphSlamNode::sensorDataCallback, this, _1));

    // Initialize synchronized odometry subscriber.
    odom_sub_.subscribe(nh, "/odom", 10);
    odom_sub_filter_ = new tf::MessageFilter<nav_msgs::Odometry>(odom_sub_, tfl, "/base_link", 10);
    odom_sub_filter_->registerCallback(boost::bind(&GraphSlamNode::odomCallback, this, _1));

    // Start display and optimization thread.
//    graph_thread_ = std::thread(&GraphSlamNode::processThread, this);

    // Initialize timer callbacks.
    odom_timer_ = nh.createTimer(ros::Duration(.1), &GraphSlamNode::odomTimerCallback, this);
    optimization_timer_ = nh.createTimer(ros::Duration(5.), &GraphSlamNode::optimizationTimerCallback, this);

    conn_ = mr::makeDbConnection(nh, "localhost", 27019, 60.0);

    graph.addSensor("/base_link", Eigen::Isometry3d::Identity());
}

void GraphSlamNode::odomTimerCallback(const ros::TimerEvent& e)
{
    ros::Time current_time = ros::Time::now();

    // Get last odometry reading from TF.
    if (!Conversions::getTransform(tfl, current_odometry_pose_, odom_frame_, odom_childframe_, ros::Time(0))) {
        ROS_ERROR_ONCE("failed to lookup TF %s -> %s", odom_frame_.c_str(), odom_childframe_.c_str());
        return;
    }

    // Calculate the difference in translation and rotation as compared to the last node pose.
    double diffTranslation = (last_odometry_pose_.translation() - current_odometry_pose_.translation()).norm();
    Eigen::AngleAxisd diffAngleAxis(current_odometry_pose_.linear().transpose() * last_odometry_pose_.linear());
    double diffRotation = fabs(diffAngleAxis.angle()) * 180 / M_PI;

    // Update covariance matrix. Choose the maximum covariance.
    if (current_odom_covariance.diagonal().prod() > odom_covariance_.diagonal().prod()) {
        odom_covariance_ = current_odom_covariance;
    }

    // Add a new node if we have moved too much.
    if (diffTranslation >= config_.new_node_distance_T || diffRotation >= config_.new_node_distance_R) {
        ROS_DEBUG("request sensor data");
        graph_slam_msgs::SensorRequest request;
        request.header.stamp = current_time;
        sensor_request_pub_.publish(request);
        last_odometry_pose_ = current_odometry_pose_;
    }

    // Publish the SLAM-to-odometry pose offset.
    g2o::Vector7d odom_map_transform = g2o::internal::toVectorQT(map_transform_);
    odom_transform_.stamp_ = current_time;
    odom_transform_.setOrigin(tf::Vector3(odom_map_transform(0), odom_map_transform(1), odom_map_transform(2)));
    odom_transform_.setRotation(tf::Quaternion(odom_map_transform(3), odom_map_transform(4), odom_map_transform(5), odom_map_transform(6)));
    broadcaster.sendTransform(odom_transform_);

    // Publish the global pose separately.
    geometry_msgs::PoseStamped currentPose;
    currentPose.header = odom->header;
    currentPose.header.frame_id = "/map";
    currentPose.header.stamp = current_time;
    Eigen::Isometry3d tempPose = map_transform_ * current_odometry_pose_;
    currentPose.pose = Conversions::toMsg(tempPose);
    pose_pub_.publish(currentPose);
}

void GraphSlamNode::sensorDataCallback(const graph_slam_msgs::SensorDataArrayConstPtr &sensor_data_array)
{
    auto start = system_clock::now();
    graph_mutex_.lock();

    bool first_run = true;
    if (sensor_data_array->data.size() > 0) {
        // Add one node per data piece.
        std::vector<SensorDataPtr> sensor_data = Conversions::fromMsg(*sensor_data_array);
        for (auto data : sensor_data) {
            std::string camera_frame = data->sensor_frame_;
            if (!graph.existsSensor(camera_frame)) {
                Eigen::Isometry3d sensor_transform = Eigen::Isometry3d::Identity();
                if (Conversions::getTransform(tfl, sensor_transform, "/base_link", camera_frame, ros::Time(0))) {
                    graph.addSensor(camera_frame, sensor_transform);
                } else {
                    ROS_ERROR("tried to add sensor data with unknown frame_id");
                    continue;
                }
            }

            // Check if this sensor data belongs to a new node.
            if (!first_run && graph.size() > 0 && fabs((data->stamp_ - graph.node(current_node_).stamp_).toSec()) < 0.001) {
                graph.node(current_node_).addSensorData(data);
            } else {                
                // Get odometry and map positions.
                Eigen::Isometry3d odometry_pose = Eigen::Isometry3d::Identity();
                if (!Conversions::getTransform(tfl, odometry_pose, odom_transform_.child_frame_id_, "/base_link", data->stamp_)) {
                    ROS_ERROR("tried to add node without TF");
                    continue;
                }

                if (graph.existsNode(current_node_)) {
                    // Perform place recognition for the node that is to be replaced as the current node.
                    place_recognizer_->searchAndAddPlace(graph.node(current_node_));

                    // Get new connections from place recognizer and estimate transforms.
                    if (place_recognizer_->hasRecognizedPlaces()) {
                        std::vector<std::pair<std::string, std::string> > neighbors = place_recognizer_->recognizedPlaces();
                        for (auto node_ids : neighbors) {
                            if (config_.new_edge_time < 0 || fabs((graph.node(node_ids.first).stamp_ - graph.node(node_ids.second).stamp_).toSec()) >= config_.new_edge_time) {
                                ROS_DEBUG("potential link: %s -> %s", node_ids.first.c_str(), node_ids.second.c_str());
                                transformation_estimator_->estimateEdge(graph.node(node_ids.first), graph.node(node_ids.second));
                            }
                        }
                    }
                }

                // Keep last node id;
                last_node_ = current_node_;
                current_node_ = boost::lexical_cast<std::string>(data->stamp_.toSec()) + boost::uuids::to_string(uuid_generator());

                // Construct a new node at the current position.
                SlamNode newNode(data->stamp_, current_node_, odometry_pose, map_transform_ * odometry_pose);
                newNode.addSensorData(data);

                // Add new node to the graph.
                graph.addNode(newNode);
                ROS_DEBUG("NODE ID: %s", newNode.id_.c_str());

                // Add links.
                if (graph.existsNode(last_node_)) {
                    // Add odometry link from previous Node to new Node (if this is not the first node).
                    double diff_time = std::max(0.1, fabs((graph.node(current_node_).stamp_ - graph.node(last_node_).stamp_).toSec()));
                    Eigen::Isometry3d odom_movement = graph.node(last_node_).sub_pose_.inverse() * graph.node(current_node_).sub_pose_;

                    SlamEdge odom_edge(boost::lexical_cast<std::string>(data->stamp_.toSec()) + boost::uuids::to_string(uuid_generator()),
                                       last_node_, current_node_,
                                       odom_movement,
                                       (1. / (diff_time * diff_time)) * odom_information_,
                                       graph_slam_msgs::Edge::TYPE_2D_WHEEL_ODOMETRY,
                                       "odom", "odom");
                    graph.addEdge(odom_edge);
                }
            }
            first_run = false;
        }

        ROS_DEBUG("SENSOR DATA CALLBACK: %d ms", (int)duration_cast<milliseconds>(system_clock::now() - start).count());
    }
    graph_mutex_.unlock();
}

bool GraphSlamNode::mapRequestCallback(graph_slam_msgs::MapRequestService::Request &req, graph_slam_msgs::MapRequestService::Response &res)
{
    // Generate occupancy grid.
    if (req.info.type == graph_slam_msgs::MapRequest::OCCUPANCY_GRID) {
        graph_mutex_.lock();
        projector_->project(graph);
        graph_mutex_.unlock();
    } else {
        graph_mutex_.lock();
        fusion_projector_->project(graph);
        graph_mutex_.unlock();
    }
    return true;
}

bool GraphSlamNode::calibrateRequestCallback(graph_slam_msgs::EmptyService::Request &req, graph_slam_msgs::EmptyService::Response &res)
{
    graph_mutex_.lock();
    sensor_optimizer_->optimize(graph, boost::bind(&GraphSlamNode::finishedSensorOptimization, this));
    graph_mutex_.unlock();
    return true;
}

bool GraphSlamNode::clearCallback(graph_slam_msgs::EmptyService::Request &req, graph_slam_msgs::EmptyService::Response &res)
{
    graph_mutex_.lock();
    clear();
    graph_mutex_.unlock();
    return true;
}

bool GraphSlamNode::loadCallback(graph_slam_msgs::EmptyService::Request &req, graph_slam_msgs::EmptyService::Response &res)
{
    graph_mutex_.lock();
    load();
    graph_mutex_.unlock();
    return true;
}

bool GraphSlamNode::storeCallback(graph_slam_msgs::EmptyService::Request &req, graph_slam_msgs::EmptyService::Response &res)
{
    graph_mutex_.lock();
    store();
    graph_mutex_.unlock();
    return true;
}

void GraphSlamNode::newEdgeCallback(SlamEdge &edge)
{
    auto start = system_clock::now();
    graph_mutex_.lock();
    ROS_DEBUG("edge estimate: %f (%s -> %s)", edge.matching_score_, edge.id_from_.c_str(), edge.id_to_.c_str());

    // Only make link, if the nodes match well enough.
    if (edge.matching_score_ >= config_.min_matching_score) {

        // Do not add a new edge if we have moved too much.
        Eigen::AngleAxisd diffAngleAxis(edge.transform_.linear());
        double diffRotation = fabs(diffAngleAxis.angle()) * 180 / M_PI;
        if (edge.transform_.translation().norm() <= config_.max_edge_distance_T && diffRotation <= config_.max_edge_distance_R) {
            edge.id_ = boost::lexical_cast<std::string>(ros::Time::now().toSec()) + boost::uuids::to_string(uuid_generator());
            graph.addEdge(edge);

            if (edge.id_from_ > most_recent_loop_closure_) {
                most_recent_loop_closure_ = edge.id_from_;
            }
            if (edge.id_to_ > most_recent_loop_closure_) {
                most_recent_loop_closure_ = edge.id_to_;
            }
        }
    }
    ROS_DEBUG("NEW EDGE CALLBACK: %d ms", (int)duration_cast<milliseconds>(system_clock::now() - start).count());
    graph_mutex_.unlock();
}

void GraphSlamNode::graphSlamConfigCallback(graph_slam::GraphSlamConfig &config, uint32_t level)
{
    graph_mutex_.lock();
    config_ = config;
    odom_information_ = Eigen::MatrixXd::Identity(6,6);
    odom_information_(0,0) = 1. / (config.odom_covariance_T * config.odom_covariance_T);
    odom_information_(1,1) = odom_information_(0,0);
    odom_information_(2,2) = odom_information_(0,0);
    odom_information_(3,3) = 1. / (std::sin(config.odom_covariance_R * M_PI / 180.) * std::sin(config.odom_covariance_R * M_PI / 180.));
    odom_information_(4,4) = odom_information_(3,3);
    odom_information_(5,5) = odom_information_(3,3);
    graph_mutex_.unlock();
}

void GraphSlamNode::store()
{
    mr::dropDatabase("graph_slam", "localhost", 27019, 60.0);

    // Add nodes that are new.
    mr::MessageCollection<graph_slam_msgs::Node> nodes_collection("graph_slam", "nodes", "localhost", 27019, 60.0);
    for (auto node_it = graph.nodeIterator(); node_it.first != node_it.second; ++node_it.first) {
        graph_slam_msgs::Node node_msg = Conversions::toMsg(node_it.first->second, true);
        nodes_collection.insert(node_msg, makeMetadata(node_msg));
    }

    // Add edges that are new.
    mr::MessageCollection<graph_slam_msgs::Edge> edges_collection("graph_slam", "edges", "localhost", 27019, 60.0);
    for (auto edge_it = graph.edgeIterator(); edge_it.first != edge_it.second; ++edge_it.first) {
        graph_slam_msgs::Edge edge_msg = Conversions::toMsg(edge_it.first->second);
        edges_collection.insert(edge_msg, makeMetadata(edge_msg));
    }

    // Add new sensor transformations.
    mr::MessageCollection<graph_slam_msgs::SensorTransform> sensors_collection("graph_slam", "sensors", "localhost", 27019, 60.0);
    for (auto sensor_it = graph.sensorIterator(); sensor_it.first != sensor_it.second; ++sensor_it.first) {
        graph_slam_msgs::SensorTransform sensor_tranform_msg;
        sensor_tranform_msg.sensor_name = sensor_it.first->first;
        sensor_tranform_msg.transform = Conversions::toMsg(sensor_it.first->second);
        sensors_collection.insert(sensor_tranform_msg, makeMetadata(sensor_tranform_msg));
    }
}

void GraphSlamNode::load()
{
    clear();
    mongo::Query query;

    // Get all stored nodes.
    mr::MessageCollection<graph_slam_msgs::Node> nodes_collection("graph_slam", "nodes", "localhost", 27019, 60.0);
    std::vector<NodeMetaPtr> stored_nodes = nodes_collection.pullAllResults(query);
    for (auto node_msg_with_meta : stored_nodes) {
        graph_slam_msgs::Node node_msg = *node_msg_with_meta;
        ROS_INFO("add node %s", node_msg.id.c_str());
        graph.addNode(Conversions::fromMsg(node_msg));
        place_recognizer_->addPlace(graph.node(node_msg.id));
    }
    graph.mostRecentNode() = "";

    // Get all stored edges.
    mr::MessageCollection<graph_slam_msgs::Edge> edges_collection("graph_slam", "edges", "localhost", 27019, 60.0);
    std::vector<EdgeMetaPtr> stored_edges = edges_collection.pullAllResults(query);
    for (auto edge_msg_with_meta : stored_edges) {
        graph_slam_msgs::Edge edge_msg = *edge_msg_with_meta;
        ROS_INFO("add edge %s", edge_msg.id.c_str());
        graph.addEdge(Conversions::fromMsg(edge_msg));
    }

    // Get all stored sensor transforms.
    mr::MessageCollection<graph_slam_msgs::SensorTransform> sensors_collection("graph_slam", "sensors", "localhost", 27019, 60.0);
    std::vector<SensorMetaPtr> stored_sensors = sensors_collection.pullAllResults(query);
    for (auto sensor_msg_with_meta : stored_sensors) {
        graph_slam_msgs::SensorTransform sensor_msg = *sensor_msg_with_meta;
        ROS_INFO("add sensor %s", sensor_msg.sensor_name.c_str());
        graph.addSensor(sensor_msg.sensor_name, Conversions::fromMsg(sensor_msg.transform));
    }
}

void GraphSlamNode::graphSlamConfigCallback(graph_slam::GraphSlamConfig &config, uint32_t level)
{
    config_ = config;
    if (config.use_cloud_registration) {
        cloud_transformation_estimator_ = boost::shared_ptr<TransformationEstimator>(new LaserTransformationEstimator(boost::bind(&GraphSlamNode::newCloudEdgeCallback, this, _1))),
        cloud_registration_timer_ = _nh.createTimer(ros::Duration(0.2), &GraphSlamNode::cloudRegistrationTimerCallback, this);
    }
    if (config.use_parameter_optimization) {
        parameter_optimization_timer_ = _nh.createTimer(ros::Duration(10.), &GraphSlamNode::parameterOptimizationTimerCallback, this);
    }
}

void GraphSlamNode::linkEstimationConfigCallback(graph_slam_tools::FeatureLinkEstimationConfig &config, uint32_t level)
{
    transformation_estimator_->setConfig(config);
}

void GraphSlamNode::occupancyGridConfigCallback(graph_slam_tools::OccupancyGridProjectorConfig &config, uint32_t level)
{
    projector_->setConfig(config);
}

void GraphSlamNode::placeRecognizerConfigCallback(graph_slam_tools::PlaceRecognizerConfig &config, uint32_t level)
{
    place_recognizer_->setConfig(config);
}

void GraphSlamNode::graphOptimizerConfigCallback(graph_slam_tools::GraphOptimizerConfig &config, uint32_t level)
{
    optimizer_->setConfig(config);
}

void GraphSlamNode::optimizationTimerCallback(const ros::TimerEvent& e)
{
    if (graph.size() > 1) {
        graph_mutex_.lock();
        ROS_DEBUG("start optimization");
        if (optimizer_->optimize(graph, boost::bind(&GraphSlamNode::finishedGraphOptimization, this))) {
            most_recent_optimizer_node_ = graph.mostRecentNode();
        }
        graph_mutex_.unlock();
    }
}

void GraphSlamNode::cloudRegistrationTimerCallback(const ros::TimerEvent& e)
{
    if (graph.size() > 1) {
        graph_mutex_.lock();
        boost::shared_ptr<LaserTransformationEstimator> c = boost::dynamic_pointer_cast<LaserTransformationEstimator>(cloud_transformation_estimator_);
        for (auto sensor_it = graph.sensorIterator(); sensor_it.first != sensor_it.second; ++sensor_it.first) {
            c->sensor_transforms_[sensor_it.first->first] = sensor_it.first->second;
        }

        // Choose random node.
        int random_node = rand() % graph.size();
        auto it = graph.nodeIterator();
        std::advance(it.first, random_node);
        if (it.first->first > most_recent_loop_closure_) {
            graph_mutex_.unlock();
            return;
        }
        SlamNode &node1 = it.first->second;
        std::vector<polygon> p1;
        LaserscanDataPtr laser_data_from;
        for (auto sensor_data : node1.sensor_data_) {
            if (sensor_data->type_ == graph_slam_msgs::SensorData::SENSOR_TYPE_LASERSCAN) {
                laser_data_from = boost::dynamic_pointer_cast<LaserscanData>(sensor_data);
                p1 = toPolygon(laser_data_from->scan_, node1.pose_);
                break;
            }
        }

        // Access nodes in random order.
        std::vector<std::string> ids = graph.nodeIds();
        std::random_shuffle(ids.begin(), ids.end());
        graph_mutex_.unlock();

        bool found = false;
        for (auto node_id : ids) {
            graph_mutex_.lock();
            if (node_id <= most_recent_loop_closure_ && graph.existsNode(node_id)) {
                // Only try if there is no link between those nodes.
                if (node1.id_ != node_id && node1.edges_.find(node_id) == node1.edges_.end()) {
                    SlamNode &node2 = graph.node(node_id);
                    if (fabs((node1.stamp_ - node2.stamp_).toSec()) >= config_.new_edge_time &&
                            (node1.pose_.inverse() * node2.pose_).translation().norm() < 10.) {
                        for (auto sensor_data : node2.sensor_data_) {
                            if (sensor_data->type_ == graph_slam_msgs::SensorData::SENSOR_TYPE_LASERSCAN) {
                                LaserscanDataPtr laser_data_to = boost::dynamic_pointer_cast<LaserscanData>(sensor_data);
                                if (checkViewCompatibility(laser_data_from->scan_, laser_data_to->scan_)) {
                                    std::vector<polygon> p2 = toPolygon(laser_data_to->scan_, node2.pose_);
                                    double area = intersectionArea(p1, p2);

                                    if (area > 3.) {
                                        displayPolygon(p1, surfelPolyPub);
                                        displayPolygon(p2, surfelPolyPub2);

                                        cloud_transformation_estimator_->estimateEdge(node1, node2);
                                        found = true;
                                        break;
                                    }
                                }
                            }
                        }

                        if (found) {
                            graph_mutex_.unlock();
                            break;
                        }
                    }
                }
            }
            graph_mutex_.unlock();
        }
    }
}

void GraphSlamNode::parameterOptimizationTimerCallback(const ros::TimerEvent& e)
{
    graph_mutex_.lock();
    if (graph.size() > 1) {
        sensor_optimizer_->optimize(graph, boost::bind(&GraphSlamNode::finishedSensorOptimization, this));
    }
    graph_mutex_.unlock();
}

void GraphSlamNode::clear()
{
    graph.clear();
    place_recognizer_->clear();
}

void GraphSlamNode::finishedGraphOptimization()
{
    ROS_DEBUG("optimization finished");
    graph_mutex_.lock();
    optimizer_->storeOptimizationResults(graph);

    if (graph.existsNode(most_recent_optimizer_node_)) {
        Eigen::Isometry3d last_map_transform = map_transform_;
        map_transform_ = graph.node(most_recent_optimizer_node_).pose_ * graph.node(most_recent_optimizer_node_).sub_pose_.inverse();
        Eigen::Isometry3d diff_transform = map_transform_ * last_map_transform.inverse();

        // Apply new map transform to all new nodes.
        for (auto node_it = graph.nodeIterator(); node_it.first != node_it.second; ++node_it.first) {
            if (node_it.first->first > most_recent_optimizer_node_) {
                node_it.first->second.pose_ = diff_transform * node_it.first->second.pose_;
            }
        }
    }

    graph_slam_msgs::Graph display_graph = Conversions::toMsg(graph, false);
    graph_display_pub_.publish(display_graph);

    graph_mutex_.unlock();
}

void GraphSlamNode::finishedSensorOptimization()
{
    ROS_DEBUG("sensor optimization finished");
    graph_mutex_.lock();
    sensor_optimizer_->storeOptimizationResults(graph);
    graph_mutex_.unlock();
}

std::vector<polygon> GraphSlamNode::toPolygon(sensor_msgs::LaserScan &scan, const Eigen::Affine3d &pose)
{
    std::vector<polygon> res;
    float deg = scan.angle_min;
    std::deque<point_xy> points;
    for (auto r : scan.ranges) {
        if (!isnan(r) && r < scan.range_max) {
            double d_far = r + 0.5;
            Eigen::Vector3d point_far = pose * Eigen::Vector3d(cos(deg) * d_far, sin(deg) * d_far, 0).homogeneous();
            double d_near = r - 0.5;
            Eigen::Vector3d point_near = pose * Eigen::Vector3d(cos(deg) * d_near, sin(deg) * d_near, 0).homogeneous();

            points.push_front(point_xy(point_far(0), point_far(1)));
            points.push_back(point_xy(point_near(0), point_near(1)));
        } else {
            if (points.size() > 4) {
                points.push_back(points.front());
                polygon p;
                boost::geometry::assign_points(p, points);
                boost::geometry::correct(p);
                res.push_back(p);
                points.clear();
            }
        }
        deg +=scan.angle_increment;
    }
    if (points.size() > 4) {
        points.push_back(points.front());
        polygon p;
        boost::geometry::assign_points(p, points);
        boost::geometry::correct(p);
        res.push_back(p);
    }

    return res;
}

bool GraphSlamNode::checkViewCompatibility(sensor_msgs::LaserScan &a, sensor_msgs::LaserScan &b) {
    return true;
}

double GraphSlamNode::intersectionArea(std::vector<polygon> &a, std::vector<polygon> &b)
{
    double res = 0.;
    std::deque<polygon> output;
    for (auto pa : a) {
        for (auto pb : b) {
            boost::geometry::intersection(pa, pb, output);

            BOOST_FOREACH(polygon const& p, output) {
                res += boost::geometry::area(p);
            }
        }
    }
    return res;
}


void GraphSlamNode::displayPolygon(std::vector<polygon> &a, ros::Publisher &pub)
{
    geometry_msgs::PolygonStamped poly;
    poly.header.frame_id = "/map";
    poly.header.stamp = ros::Time();
    for (auto p1 : a) {
        for(size_t i = 0; i < p1.outer().size(); i++){
            geometry_msgs::Point32 point;
            point.x = p1.outer().at(i).x();
            point.y = p1.outer().at(i).y();
            point.z = 0;
            poly.polygon.points.push_back(point);
        }
    }
    pub.publish(poly);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "graph_slam_node");
    ros::NodeHandle nh;
    GraphSlamNode graph_slam(nh);

    ros::ServiceServer map_request_serv = nh.advertiseService<graph_slam_msgs::MapRequestService::Request, graph_slam_msgs::MapRequestService::Response>("map_request", boost::bind(&GraphSlamNode::mapRequestCallback, &graph_slam, _1, _2));
    ros::ServiceServer clear_serv = nh.advertiseService<graph_slam_msgs::EmptyService::Request, graph_slam_msgs::EmptyService::Response>("clear", boost::bind(&GraphSlamNode::clearCallback, &graph_slam, _1, _2));
    ros::ServiceServer calibrate_serv = nh.advertiseService<graph_slam_msgs::EmptyService::Request, graph_slam_msgs::EmptyService::Response>("calibrate_request", boost::bind(&GraphSlamNode::calibrateRequestCallback, &graph_slam, _1, _2));
    ros::ServiceServer load_serv = nh.advertiseService<graph_slam_msgs::EmptyService::Request, graph_slam_msgs::EmptyService::Response>("load", boost::bind(&GraphSlamNode::loadCallback, &graph_slam, _1, _2));
    ros::ServiceServer store_serv = nh.advertiseService<graph_slam_msgs::EmptyService::Request, graph_slam_msgs::EmptyService::Response>("store", boost::bind(&GraphSlamNode::storeCallback, &graph_slam, _1, _2));

    dynamic_reconfigure::Server<graph_slam_tools::FeatureLinkEstimationConfig> server_le(ros::NodeHandle("~/feature_link_estimation"));
    dynamic_reconfigure::Server<graph_slam_tools::FeatureLinkEstimationConfig>::CallbackType f_le;
    f_le = boost::bind(&GraphSlamNode::linkEstimationConfigCallback, &graph_slam, _1, _2);
    server_le.setCallback(f_le);

    dynamic_reconfigure::Server<graph_slam_tools::OccupancyGridProjectorConfig> server_og(ros::NodeHandle("~/occupancy_grid_estimation"));
    dynamic_reconfigure::Server<graph_slam_tools::OccupancyGridProjectorConfig>::CallbackType f_og;
    f_og = boost::bind(&GraphSlamNode::occupancyGridConfigCallback, &graph_slam, _1, _2);
    server_og.setCallback(f_og);

    dynamic_reconfigure::Server<graph_slam_tools::PlaceRecognizerConfig> server_pr(ros::NodeHandle("~/place_recognizer"));
    dynamic_reconfigure::Server<graph_slam_tools::PlaceRecognizerConfig>::CallbackType f_pr;
    f_pr = boost::bind(&GraphSlamNode::placeRecognizerConfigCallback,  &graph_slam, _1, _2);
    server_pr.setCallback(f_pr);

    dynamic_reconfigure::Server<graph_slam_tools::GraphOptimizerConfig> server_go(ros::NodeHandle("~/graph_optimizer"));
    dynamic_reconfigure::Server<graph_slam_tools::GraphOptimizerConfig>::CallbackType f_go;
    f_go = boost::bind(&GraphSlamNode::graphOptimizerConfigCallback,  &graph_slam, _1, _2);
    server_go.setCallback(f_go);

    dynamic_reconfigure::Server<graph_slam::GraphSlamConfig> server_gs;
    dynamic_reconfigure::Server<graph_slam::GraphSlamConfig>::CallbackType f_gs;
    f_gs = boost::bind(&GraphSlamNode::graphSlamConfigCallback,  &graph_slam, _1, _2);
    server_gs.setCallback(f_gs);

    ros::AsyncSpinner spinner(8); // Use 8 threads
    spinner.start();
    ros::waitForShutdown();
}
