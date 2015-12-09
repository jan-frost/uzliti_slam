// Copyright (c) 2014, Institute of Computer Engineering (ITI), Universität zu Lübeck
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

#include <graph_slam/graph_slam_node.h>

#include <graph_slam_common/conversions.h>
#include <graph_slam_msgs/Scope.h>
#include <graph_slam_msgs/SensorRequest.h>
#include <graph_slam_msgs/NodePair.h>
#include <thread>
#include <message_filters/subscriber.h>

//! Please note: we cannot include this any earlier, because flann seems to have an ambiguity issue towards g2o.
#include <transformation_estimation/cloud_transformation_estimator.h>
#include <transformation_estimation/laser_transformation_estimator.h>

using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::minutes;
using std::chrono::system_clock;

struct PairSort {
  bool operator() (std::pair<std::string, double> i, std::pair<std::string, double> j) { return (i.second < j.second);}
} PairSort;

GraphSlamNode::GraphSlamNode(ros::NodeHandle nh, std::string name, bool sync_to_database) :
    nh_(nh),
    sync_to_database_(sync_to_database),
    graph("/map", name, sync_to_database),
    optimizer_(new G2oOptimizer()),
    sensor_optimizer_(new SensorTransformOptimizer()),
    transformation_estimator_(new FeatureTransformationEstimator(boost::bind(&GraphSlamNode::newEdgeCallback, this, _1))),
    cloud_transformation_estimator_(new LaserTransformationEstimator(boost::bind(&GraphSlamNode::newCloudEdgeCallback, this, _1))),
    projector_(new OccupancyGridProjector(nh)),
    tfl_(ros::Duration(10.0))
{
    current_node_ = "";
    last_node_ = "";
    most_recent_loop_closure_ = "";
    current_odometry_pose_ = Eigen::Isometry3d::Identity();
    last_odometry_pose_ = Eigen::Isometry3d::Identity();
    odom_covariance_ = Eigen::MatrixXd::Zero(6,6);
    odom_information_ = Eigen::MatrixXd::Zero(6,6);

    // Initialize publishers
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("graph_pose", 1);
    graph_display_pub_ = nh_.advertise<graph_slam_msgs::Graph>("/display_graph", 1);
    sensor_request_pub_ = nh_.advertise<graph_slam_msgs::SensorRequest>("/sensor_request", 1);

    ros::NodeHandle nh_private("~");

    nh_private.param<bool>("is_sub_graph", is_sub_graph_, false);
    nh_private.param<bool>("is_super_graph", is_super_graph_, false);
    nh_private.param<bool>("construct_nodes", construct_nodes_, true);
    nh_private.param<bool>("publish_map_transform", publish_map_transform_, true);
    nh_private.param<bool>("publish_occupancy_grid", publish_occupancy_grid_, true);

    if (is_sub_graph_) {
        scope_request_pub_ = nh_.advertise<graph_slam_msgs::Scope>("request_scope", 1);
        sub_graph_pub_ = nh_.advertise<graph_slam_msgs::Graph>("sub_graph", 1);
        scope_vis_pub_ = nh_.advertise<visualization_msgs::Marker>("scope_marker", 1);

        scope_request_timer_ = nh_.createTimer(ros::Duration(1.), &GraphSlamNode::scopeRequestTimerCallback, this);

        current_scope_ = 8.;
    }

    if (is_super_graph_) {
        scope_pub_ = nh.advertise<graph_slam_msgs::Graph>("scope_graph", 1);
        graph_received_pub_ = nh.advertise<graph_slam_msgs::GraphReceived>("graph_received", 1);

        last_scope_center_ = Eigen::Vector3d(0.,0.,0.);
        last_scope_radius_ = 0.;
    }

    if (construct_nodes_) {
        odom_timer_ = nh.createTimer(ros::Duration(.1), &GraphSlamNode::odomTimerCallback, this);
    }

    bool do_merge, do_laser_registration;
    nh_private.param<bool>("do_merge", do_merge, true);
    nh_private.param<bool>("do_laser_registration", do_laser_registration, true);

    // Get parameters.
    std::string pr_method;
    nh_private.param<std::string>("place_recognition_method", pr_method, "gist");
    ROS_INFO("place recognition method: %s", pr_method.c_str());
    if (pr_method == "gist") {
        place_recognizer_ = boost::shared_ptr<PlaceRecognizer>(new BinaryGistRecognizer());
    } else if (pr_method == "gfr") {
        place_recognizer_ = boost::shared_ptr<PlaceRecognizer>(new GlobalFeatureRepositoryRecognizer());
    } else if (pr_method == "lsh") {
        place_recognizer_ = boost::shared_ptr<PlaceRecognizer>(new LshSetRecognizer());
    } else if (pr_method == "bow") {
        ROS_ERROR("bag-of-words place recognizer not implemented");
    } else {
        ROS_ERROR("unknown place recognizer");
    }

    std::string database_path;
    nh_private.param<std::string>("database_path", database_path, "db_temp");
    bool clear_database;
    nh_private.param<bool>("clear_database", clear_database, true);
    graph.initializeStorage(database_path, clear_database);

    load();

    graph.addSensor("/base_footprint", Eigen::Isometry3d::Identity());

    nh_private.param<std::string>("odom_frame", odom_frame_, "odom");
    ROS_INFO("odom_frame: %s", odom_frame_.c_str());
    nh_private.param<std::string>("odom_childframe", odom_childframe_, "base_footprint");
    ROS_INFO("odom_childframe: %s", odom_childframe_.c_str());
    odom_transform_.frame_id_ = "/map";
    odom_transform_.child_frame_id_ = odom_frame_;

    bool near_scan;
    nh_private.param<bool>("near_scan", near_scan, false);
    boost::shared_ptr<LaserTransformationEstimator> c = boost::dynamic_pointer_cast<LaserTransformationEstimator>(cloud_transformation_estimator_);
    c->do_near_ = near_scan;
    ROS_INFO("graph slam: using near scan for ICP registration: %d", near_scan);

    nh_private.param<double>("min_accept_valid", min_accept_valid_, std::numeric_limits<double>::max());

    graph_slam_msgs::Graph display_graph = Conversions::toMsg(graph, false);
    graph_display_pub_.publish(display_graph);

    last_frame_id_ = "/base_footprint";

    // Reload place recognizer.
    ROS_INFO("global scope: initializing place recognizer");
    place_recognizer_->clear();
    for (auto node_it = graph.nodeIterator(); node_it.first != node_it.second; node_it.first++) {
        SlamNode &node = node_it.first->second;
        place_recognizer_->addPlace(node);
    }

    new_nodes_since_last_optimization_ = 0;

    graph_initialized_ = false;
    current_scan_index_ = 1;
    current_merge_index_ = 1;
}

void GraphSlamNode::odomTimerCallback(const ros::TimerEvent& e)
{
    ros::Time current_time = ros::Time::now();

    // Get last odometry reading from TF.
    ros::Time t = ros::Time(0);
    if (!Conversions::getTransform(tfl_, current_odometry_pose_, odom_frame_, odom_childframe_, t)) {
        ROS_ERROR_ONCE("failed to lookup TF %s -> %s", odom_frame_.c_str(), odom_childframe_.c_str());
        return;
    }

    // Calculate the difference in translation and rotation as compared to the last node pose.
    double diffTranslation = (last_odometry_pose_.translation() - current_odometry_pose_.translation()).norm();
    Eigen::AngleAxisd diffAngleAxis(current_odometry_pose_.linear().transpose() * last_odometry_pose_.linear());
    double diffRotation = fabs(diffAngleAxis.angle()) * 180 / M_PI;

    // Add a new node if we have moved too much.
    if (graph.size() == 0 || diffTranslation >= config_.new_node_distance_T || diffRotation >= config_.new_node_distance_R) {
        ROS_DEBUG("request sensor data");
        graph_slam_msgs::SensorRequest request;
        request.header.frame_id = last_frame_id_;
        request.header.stamp = current_time;
        sensor_request_pub_.publish(request);
        last_odometry_pose_ = current_odometry_pose_;
    }

    // Publish the SLAM-to-odometry pose offset.
    if (publish_map_transform_) {
        g2o::Vector7d odom_map_transform = g2o::internal::toVectorQT(graph.diffTransform());
        odom_transform_.stamp_ = current_time;
        odom_transform_.setOrigin(tf::Vector3(odom_map_transform(0), odom_map_transform(1), odom_map_transform(2)));
        odom_transform_.setRotation(tf::Quaternion(odom_map_transform(3), odom_map_transform(4), odom_map_transform(5), odom_map_transform(6)));
            broadcaster.sendTransform(odom_transform_);

        // Publish the global pose separately.
        geometry_msgs::PoseStamped currentPose;
        currentPose.header.frame_id = "/map";
        currentPose.header.stamp = current_time;
        Eigen::Isometry3d tempPose = graph.diffTransform() * current_odometry_pose_;
        currentPose.pose = Conversions::toMsg(tempPose);
        pose_pub_.publish(currentPose);
    }
}

void GraphSlamNode::sensorDataCallback(const graph_slam_msgs::SensorDataArrayConstPtr &sensor_data_array)
{
    std::lock_guard<std::mutex> lock(graph_mutex_);
    auto start = system_clock::now();

    bool first_run = true;
    if (sensor_data_array->data.size() > 0) {
        // Add one node per data piece.
        std::vector<SensorDataPtr> sensor_data = Conversions::fromMsg(*sensor_data_array);
        for (auto data : sensor_data) {
            std::string camera_frame = data->sensor_frame_;
            Eigen::Isometry3d sensor_diff_transform = Eigen::Isometry3d::Identity();
            if (!graph.existsSensor(camera_frame)) {
                Eigen::Isometry3d sensor_transform = Eigen::Isometry3d::Identity();
                if (Conversions::getTransform(tfl_, sensor_transform, odom_childframe_, camera_frame, data->stamp_)) {
                    graph.addSensor(camera_frame, sensor_transform);
                } else {
                    ROS_ERROR("tried to add sensor data of type %d with unknown frame_id: %s", (int)data->type_, camera_frame.c_str());
                    continue;
                }
            } else {
                Eigen::Isometry3d new_sensor_transform;
                if (Conversions::getTransform(tfl_, new_sensor_transform, odom_childframe_, camera_frame, data->stamp_)) {
                    sensor_diff_transform = new_sensor_transform * graph.sensor(camera_frame).inverse();
                } else {
                    ROS_ERROR("failed to set camera diff transform");
                }
            }

            last_frame_id_ = data->sensor_frame_;

            if (sensor_diff_transform.translation().norm() < 0.1) {
                data->displacement_ = sensor_diff_transform;
            } else {
                data->displacement_ = Eigen::Isometry3d::Identity();
            }

            // Check if this sensor data belongs to a new node.
            if (!first_run && graph.size() > 0 && fabs((data->stamp_ - graph.node(current_node_).stamps_.front()).toSec()) < 0.001) {
                graph.node(current_node_).addSensorData(data);
                graph.updateDatabaseNode(current_node_);
            } else {                
                // Get odometry and map positions.
                Eigen::Isometry3d odometry_pose = Eigen::Isometry3d::Identity();
                if (!Conversions::getTransform(tfl_, odometry_pose, odom_frame_, odom_childframe_, data->stamp_)) {
                    ROS_ERROR("tried to add node without TF");
                    continue;
                }

                if (graph.existsNode(current_node_)) {
                    SlamNode &current_node = graph.node(current_node_);

                    // Perform place recognition for the node that is to be replaced as the current node.
                    if (config_.do_feature_loop_closure) {
                        place_recognizer_->addNode(current_node);

                        // Get new connections from place recognizer and estimate transforms.
                        if (place_recognizer_->hasRecognizedPlaces()) {
                            std::vector<std::pair<std::string, std::string> > neighbors = place_recognizer_->recognizedPlaces();
                            for (auto node_ids : neighbors) {
                                if (graph.existsNode(node_ids.first) && graph.existsNode(node_ids.second)) {
                                    transformation_estimator_->estimateEdge(graph.node(node_ids.first), graph.node(node_ids.second));
                                }
                            }
                        }
                    }

                    // Estimate links to close nodes.
                    if (config_.do_distance_loop_closure) {
                        auto close_node_ids = graph.getNodesWithinRadius(current_node_, config_.distance_loop_closure_radius);
                        for (auto node_id : close_node_ids) {
                            if (graph.existsNode(node_id)) {
                                SlamNode &close_node = graph.node(node_id);
                                if (fabs((current_node.stamps_.front() - close_node.stamps_.front()).toSec()) > config_.new_edge_time) {
                                    Eigen::Isometry3d diff_pose = close_node.pose_.inverse() * current_node.pose_;
                                    Eigen::AngleAxisd diff_angle_axis(diff_pose.linear());
                                    double diff_rotation = 180. * diff_angle_axis.angle() / M_PI;

                                    if (fabs(diff_rotation) < 30.) {
                                        transformation_estimator_->estimateEdge(close_node, current_node);
                                    }
                                }
                            }
                        }
                    }
                }

                // Keep last node id;
                last_node_ = current_node_;
                current_node_ = boost::lexical_cast<std::string>(data->stamp_.toSec()) + boost::uuids::to_string(uuid_generator());

                // Construct a new node at the current position.
                SlamNode newNode(data->stamp_, current_node_, odometry_pose, graph.diffTransform() * odometry_pose);
                newNode.addSensorData(data);

                // Add new node to the graph.
                graph.addNode(newNode);
                new_nodes_since_last_optimization_++;
                nodes_to_send_.insert(current_node_);
                ROS_DEBUG("NODE ID: %s", newNode.id_.c_str());

                // Add links.
                if (graph.existsNode(last_node_)) {
                    // Add odometry link from previous Node to new Node (if this is not the first node).
                    Eigen::Isometry3d odom_movement = graph.node(last_node_).sub_pose_.inverse() * graph.node(current_node_).sub_pose_;
                    double diff_time = std::min(2.0, std::max(0.1, fabs((graph.node(current_node_).stamps_.front() - graph.node(last_node_).stamps_.front()).toSec())));

//                    double diff_translation = odom_movement.translation().squaredNorm();
//                    Eigen::AngleAxisd diffAngleAxis(odom_movement.linear());
//                    double diff_rotation = diffAngleAxis.angle() * diffAngleAxis.angle();

                    Eigen::MatrixXd odom_information = Eigen::MatrixXd::Identity(6,6);
                    double base = 0.02;
                    double factor = 0.05;
                    odom_information.block<3,3>(0,0) /= base*base;
                    odom_information.block<3,3>(3,3) /= base*base*factor*factor;
                    odom_information /= diff_time * diff_time;

//                    if (diff_translation <= 5. * config_.new_node_distance_T && diff_rotation <= 5 * config_.new_node_distance_R) {
//                        odom_information = 100. * Eigen::MatrixXd::Identity(6,6);
//                    }

                    SlamEdge odom_edge(boost::lexical_cast<std::string>(data->stamp_.toSec()) + boost::uuids::to_string(uuid_generator()),
                                       last_node_, current_node_,
                                       odom_movement,
                                       odom_information,
//                                       (1. / (diff_time * diff_time)) * odom_information_,
                                       graph_slam_msgs::Edge::TYPE_2D_WHEEL_ODOMETRY,
                                       "odom", "odom",
                                       graph.node(current_node_).stamps_.front() - graph.node(last_node_).stamps_.front());
                    odom_edge.valid_ = true;
                    graph.addEdge(odom_edge);
                    edges_to_send_.insert(odom_edge.id_);

                    graph.node(current_node_).uncertainty_ = graph.node(last_node_).uncertainty_ + config_.scope_size_factor * odom_movement.translation().norm();
                }                              
            }
            first_run = false;
        }
    } else {
        ROS_WARN("no sensor data");
    }

    graph.storeMetaInformation();

    if (config_.new_node_compute_cloud_edge) {
        if (graph.existsNode(current_node_)) {
            estimateScanEdge(graph.node(current_node_));
        }
    }

    if (is_sub_graph_) {
        auto start = system_clock::now();

        auto start_local = system_clock::now();
        graph.reevaluateUncertainty(current_node_);
        ROS_DEBUG_NAMED("runtime", "  reevaluate uncertainty: %d ms", (int)duration_cast<milliseconds>(system_clock::now() - start_local).count());
        start_local = system_clock::now();

        // Send all nodes and edges that were not received yet.
        if (edges_to_send_.size() > 0 || nodes_to_send_.size() > 0) {
            graph_slam_msgs::Graph out_of_scope_graph;
            out_of_scope_graph.header.stamp = ros::Time::now();
            out_of_scope_graph.header.frame_id = graph.frame();

            for (auto node_id : nodes_to_send_) {
                if (graph.existsNode(node_id)) {
                    SlamNode node = graph.node(node_id);
                    out_of_scope_graph.nodes.push_back(Conversions::toMsg(node));
                }
            }

            for (auto edge_id : edges_to_send_) {
                if (graph.existsEdge(edge_id)) {
                    SlamEdge edge = graph.edge(edge_id);
                    out_of_scope_graph.edges.push_back(Conversions::toMsg(edge));
                }
            }

            // Add sensor transforms.
            Conversions::toMsg(graph.odom(), out_of_scope_graph.odometry_parameters);

            std::pair<SensorIterator, SensorIterator> sensor_it;
            for (sensor_it = graph.sensorIterator(); sensor_it.first != sensor_it.second; sensor_it.first++) {
                out_of_scope_graph.sensor_tranforms.push_back(Conversions::toMsg(sensor_it.first->first, sensor_it.first->second));
            }

            sub_graph_pub_.publish(out_of_scope_graph);
        }
        ROS_DEBUG_NAMED("runtime", "  send nodes: %d ms", (int)duration_cast<milliseconds>(system_clock::now() - start_local).count());
        ROS_DEBUG_NAMED("runtime", "SENSOR DATA CALLBACK (LOCAL SCOPE): %d ms", (int)duration_cast<milliseconds>(system_clock::now() - start).count());
    }

    ROS_DEBUG_NAMED("runtime", "SENSOR DATA CALLBACK: %d ms", (int)duration_cast<milliseconds>(system_clock::now() - start).count());
}

void GraphSlamNode::scopeCallback(const graph_slam_msgs::GraphConstPtr &graph_msg)
{
    auto start = system_clock::now();

    for (unsigned int i = 0; i < graph_msg->nodes.size(); i++) {
        const graph_slam_msgs::Node &node_msg = graph_msg->nodes[i];
        {
            std::lock_guard<std::mutex> lock(graph_mutex_);
            if (graph.existsNode(node_msg.id)) {
                graph.node(node_msg.id).pose_ = Conversions::fromMsg(node_msg.pose);
                graph.node(node_msg.id).uncertainty_ = node_msg.uncertainty;
                graph.node(node_msg.id).fixed_ = node_msg.fixed;
            } else if (node_msg.sensor_data.data.size() > 0) {
                SlamNode node = Conversions::fromMsg(node_msg);
                graph.addNode(node);
                if (config_.do_feature_loop_closure) {
                    place_recognizer_->addPlaceQueue(node);
                }
            }
        }
    }
    for (unsigned int i = 0; i < graph_msg->edges.size(); i++) {
        {
            std::lock_guard<std::mutex> lock(graph_mutex_);
            if (!graph.existsEdge(graph_msg->edges[i].id)) {
                SlamEdge edge = Conversions::fromMsg(graph_msg->edges[i]);
                graph.addEdge(edge);
            }
        }
    }
    ROS_DEBUG_NAMED("runtime", "SCOPE CALLBACK (LOCAL SCOPE): %d ms", (int)duration_cast<milliseconds>(system_clock::now() - start).count());
}

void GraphSlamNode::graphReceivedCallback(const graph_slam_msgs::GraphReceivedConstPtr &graph_msg)
{
    graph_mutex_.lock();
    for (auto node_id : graph_msg->node_ids) {
        nodes_to_send_.erase(node_id);
    }
    for (auto edge_id : graph_msg->edge_ids) {
        edges_to_send_.erase(edge_id);
    }
    graph_mutex_.unlock();
}

void GraphSlamNode::subGraphCallback(const graph_slam_msgs::GraphConstPtr &graph_msg)
{
    graph_mutex_.lock();
    auto start = system_clock::now();

    // Send ACK.
    graph_slam_msgs::GraphReceived gr;
    for (unsigned int i = 0; i < graph_msg->nodes.size(); i++) {
        gr.node_ids.push_back(graph_msg->nodes[i].id);
    }
    for (unsigned int i = 0; i < graph_msg->edges.size(); i++) {
        gr.edge_ids.push_back(graph_msg->edges[i].id);
    }
    graph_received_pub_.publish(gr);

    // Load new nodes.
    for (unsigned int i = 0; i < graph_msg->nodes.size(); i++) {
        graph_slam_msgs::Node node_msg = graph_msg->nodes[i];

        if (!graph.existsNode(node_msg.id)) {
            if (!graph.isMerged(node_msg.id)) {
                SlamNode node = Conversions::fromMsg(node_msg);
                node.fixed_ = false;
                node.optimized_ = false;
                node.sub_pose_ = node.pose_;
                graph.addNode(node);

                // Perform place recognition for the node that is to be replaced as the current node.
                if (config_.do_feature_loop_closure) {
                    place_recognizer_->addNode(node);
                }
            } else {
                ROS_ERROR("! merged node (load node)");
            }
        } else {
            graph.node(node_msg.id).uncertainty_ = node_msg.uncertainty;
            for (auto edge : node_msg.edge_ids) {
                graph.node(node_msg.id).edges_.insert(edge);
            }
        }
    }

    // Load new edges.
    for (unsigned int i = 0; i < graph_msg->edges.size(); i++) {
        if (!graph.existsEdge(graph_msg->edges[i].id)) {
            SlamEdge edge = Conversions::fromMsg(graph_msg->edges[i]);

            if (graph.existsEdge(edge.id_from_, edge.id_to_, edge.type_)) {
                continue;
            }

            if (!graph.isMerged(edge.id_from_) && !graph.isMerged(edge.id_to_)) {
                if (edge.type_ == graph_slam_msgs::Edge::TYPE_3D_TRANSLATION) {
                    if (checkEdgeHeuristic(edge)) {
                        graph.addEdge(edge);
                    }
                } else {
                    graph.addEdge(edge);
                }
            } else {
                ROS_ERROR("! merged node (load edge)");
            }
        }
    }

    // Get new connections from place recognizer and estimate transforms.
    if (place_recognizer_->hasRecognizedPlaces()) {
        std::vector<std::pair<std::string, std::string> > neighbors = place_recognizer_->recognizedPlaces();
        for (auto node_ids : neighbors) {
            if (graph.existsNode(node_ids.first) && graph.existsNode(node_ids.second) &&
                    !graph.existsEdge(node_ids.first, node_ids.second)) {
                ROS_DEBUG("potential link: %s -> %s", node_ids.first.c_str(), node_ids.second.c_str());
                transformation_estimator_->estimateEdge(graph.node(graph.getMergedNodeId(node_ids.first)), graph.node(graph.getMergedNodeId(node_ids.second)));
            }
        }
    }

    graph.odom() = Conversions::fromMsg(graph_msg->odometry_parameters);
    for (unsigned int i = 0; i < graph_msg->sensor_tranforms.size(); i++) {
        if (!graph.existsSensor(graph_msg->sensor_tranforms[i].sensor_name)) {
            graph.addSensor(graph_msg->sensor_tranforms[i].sensor_name, Conversions::fromMsg(graph_msg->sensor_tranforms[i].transform));
            cloud_transformation_estimator_->sensor_transforms_[graph_msg->sensor_tranforms[i].sensor_name] = graph.sensor(graph_msg->sensor_tranforms[i].sensor_name);
        }
    }

    ROS_DEBUG_NAMED("time", "SUB GRAPH CALLBACK: %d ms", (int)duration_cast<milliseconds>(system_clock::now() - start).count());
    graph_mutex_.unlock();
}

void GraphSlamNode::scopeRequestCallback(const graph_slam_msgs::ScopeConstPtr &scope_msg)
{
    graph_mutex_.lock();
    auto start = system_clock::now();
    std::vector<std::string> scope_ids = scope_msg->scope_ids;
    Eigen::Isometry3d current_map_pose = Conversions::fromMsg(scope_msg->current_pose);
    last_scope_center_ = current_map_pose.translation();
    last_scope_radius_ = scope_msg->radius;

    graph_slam_msgs::Graph scope_graph;
    scope_graph.header.stamp = ros::Time::now();
    scope_graph.header.frame_id = graph.frame();

    std::pair<NodeIterator, NodeIterator> node_it;
    for (node_it = graph.nodeIterator(); node_it.first != node_it.second; node_it.first++) {
        SlamNode &node = node_it.first->second;
        if ((node.pose_.translation() - current_map_pose.translation()).norm() < scope_msg->radius) {
            graph_slam_msgs::Node node_msg;
            if (std::find(scope_ids.begin(), scope_ids.end(), node.id_) == scope_ids.end()) {
                node_msg = Conversions::toMsg(node, true);
                node_msg.fixed = true;
                scope_graph.nodes.push_back(node_msg);
            }
        }
    }

    for (auto scope_id : scope_ids) {
        if (graph.existsNode(scope_id)) {
            SlamNode &node = graph.node(scope_id);
            graph_slam_msgs::Node node_msg = Conversions::toMsg(node, false);
            if (node.optimized_ && fabs((ros::Time::now() - node.stamps_.front()).toSec()) > 10.) {
                node_msg.fixed = true;
            }
            scope_graph.nodes.push_back(node_msg);
        }
    }

    scope_pub_.publish(scope_graph);

    ROS_DEBUG_NAMED("time", "SCOPE REQUEST CALLBACK: %d ms", (int)duration_cast<milliseconds>(system_clock::now() - start).count());
    graph_mutex_.unlock();
}

void GraphSlamNode::scopeRequestTimerCallback(const ros::TimerEvent &e)
{
    graph_mutex_.lock();

    auto start = system_clock::now();

    // Request scope.
    if (graph.size() > 1) {
        current_scope_ = std::max(config_.scope_size_min, config_.scope_size_factor * graph.node(current_node_).uncertainty_);
        graph_slam_msgs::Scope scope;
        scope.current_pose = Conversions::toMsg(graph.node(current_node_).pose_);
        scope.radius = current_scope_;
        std::pair<NodeIterator, NodeIterator> node_it;
        for (node_it = graph.nodeIterator(); node_it.first != node_it.second; node_it.first++) {
            SlamNode &node = node_it.first->second;
            scope.scope_ids.push_back(node.id_);
        }
        scope_request_pub_.publish(scope);

        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time();
        marker.ns = "my_namespace";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose = scope.current_pose;
        marker.scale.x = 2 * current_scope_;
        marker.scale.y = 2 * current_scope_;
        marker.scale.z = 2 * current_scope_;
        marker.color.a = 0.2;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        scope_vis_pub_.publish(marker);

        ROS_DEBUG_NAMED("runtime", "  issue scope request: %d ms", (int)duration_cast<milliseconds>(system_clock::now() - start).count());

    }
    graph_mutex_.unlock();

    // Delete out of scope nodes that were successfully sent.
    std::set<std::string> out_of_scope_nodes;
    {
        std::lock_guard<std::mutex> lock(graph_mutex_);
        out_of_scope_nodes = graph.getOutOfScopeNodes(current_node_, current_scope_ + 4.);
    }
    ROS_DEBUG_NAMED("runtime", "  determine outo of scope nodes: %d ms", (int)duration_cast<milliseconds>(system_clock::now() - start).count());

    std::set<std::string> out_of_scope_edges;
    double pr_runtime = 0;
    double remove_runtime = 0;
    int count_remove = 0;
    for (auto node_id : out_of_scope_nodes) {
        std::lock_guard<std::mutex> lock(graph_mutex_);

        if (graph.existsNode(node_id)) {
            if (nodes_to_send_.find(node_id) == nodes_to_send_.end()) {
                for (auto edge_id : graph.node(node_id).edges_) {
                    out_of_scope_edges.insert(edge_id);
                }
                auto sub_start = system_clock::now();
                if (config_.do_feature_loop_closure) {
                    place_recognizer_->removePlaceQueue(graph.node(node_id));
                    pr_runtime += 0.001 * duration_cast<microseconds>(system_clock::now() - sub_start).count();
                }
                sub_start = system_clock::now();
                graph.removeNode(node_id);
                remove_runtime += 0.001 * duration_cast<microseconds>(system_clock::now() - sub_start).count();
                count_remove++;
            }
        }
    }
    ROS_DEBUG_NAMED("runtime", "  remove %d nodes: %d ms (%0.2f, %0.2f)", count_remove, (int)duration_cast<milliseconds>(system_clock::now() - start).count(), pr_runtime, remove_runtime);

    for (auto edge_id : out_of_scope_edges) {
        {
            std::lock_guard<std::mutex> lock(graph_mutex_);
            if (graph.existsEdge(edge_id)) {
                graph.removeEdge(edge_id);
            }
        }
    }

    ROS_DEBUG_NAMED("runtime", "SCOPE REQUEST TIMER: %d ms", (int)duration_cast<milliseconds>(system_clock::now() - start).count());
}

void GraphSlamNode::mergeTimerCallback(const ros::TimerEvent &e)
{
    if (!graph_initialized_) {
        return;
    }

    if (graph.size() > 1) {
        auto start = system_clock::now();
        double runtime_hop = 0;
        double runtime_merge = 0;
        bool merged = false;
        // Pick current node.
        if (current_merge_index_ >= graph.size()) {
            current_merge_index_ = 1;
        }
        int max_hops = 10;

        while (current_merge_index_ < graph.size()) {
            auto runtime_start = system_clock::now();

            std::string id_first;
            std::string id_root;
            while (current_merge_index_ < graph.size()) {
                std::lock_guard<std::mutex> lock(graph_mutex_);

                auto it = graph.nodeIterator();
                id_root = it.first->first;  // Never merge root node.
                std::advance(it.first, current_merge_index_);
                id_first = it.first->first;

                // Do not merge, if the node is too close to the local scope.
                if ((it.first->second.pose_.translation() - last_scope_center_).norm() > (last_scope_radius_ + 6.)) {
                    ROS_DEBUG_NAMED("merge", "found merge node");
                    break;
                } else {
                    current_merge_index_++;
                }
            }

            if (current_merge_index_ >= graph.size()) {
                break;
            }

            // Get neighborhood.
            std::unordered_set<std::string> neighbor_nodes;
            {
                std::lock_guard<std::mutex> lock(graph_mutex_);

                auto runtime_hop_start = system_clock::now();
                graph.getNodesWithinHops(id_first, neighbor_nodes, max_hops);
                runtime_hop = (0.001 * duration_cast<microseconds>(system_clock::now() - runtime_hop_start).count());
            }

            // Merge with first node within radius.
            for (auto id_second : neighbor_nodes) {
                // Do not merge two equal nodes.
                if (id_first == id_second) {
                    continue;
                }

                // Do not merge with root.
                if (id_root == id_second) {
                    continue;
                }

                // Do not merge if already merged.
                if (graph.isMerged(id_second)) {
                    ROS_ERROR("tried to merge merged node!");
                    continue;
                }

                {
                    std::lock_guard<std::mutex> lock(graph_mutex_);

                    // Check if the nodes are close.
                    if ((graph.node(id_first).pose_.translation() - graph.node(id_second).pose_.translation()).norm() >= 0.25) {
                        continue;
                    }
                    Eigen::AngleAxisd diff_angle_axis(graph.node(id_first).pose_.linear().transpose() * graph.node(id_second).pose_.linear());
                    double diff_rotation = fabs(diff_angle_axis.angle()) * 180 / M_PI;
                    if (diff_rotation > 15.) {
                        continue;
                    }
                }

                // Always merge to older node.
                if (id_first < id_second) {
                    std::swap(id_first, id_second);
                }

                if (mergeNodes(id_first, id_second)) {
                    auto runtime_merge_start = system_clock::now();
                    graph.mergeNodes(id_second, id_first);
                    runtime_merge = (0.001 * duration_cast<microseconds>(system_clock::now() - runtime_merge_start).count());
                    merged = true;
                    ROS_DEBUG_NAMED("time", "MERGE TIMER CALLBACK: %d ms", (int)duration_cast<milliseconds>(system_clock::now() - start).count());
                }
                break;
            }

            ROS_DEBUG("current merge index: %d / %d", current_merge_index_, graph.size());

            if (!merged) {
//                std::lock_guard<std::mutex> lock(graph_mutex_);

                current_merge_index_++;
//                graph.updateDatabaseNode(id_first); // Update database to keep node poses up to date.b
            } else {
                break;
            }
        }
    }
}

void GraphSlamNode::newEdgeCallback(SlamEdge &edge)
{
    {
        std::lock_guard<std::mutex> lock(graph_mutex_);

        if (graph.isMerged(edge.id_from_) || graph.isMerged(edge.id_to_)) {
            ROS_ERROR("! merged node (feature)");
            return;
        }

        if (graph.existsEdge(edge.id_from_, edge.id_to_, edge.type_)) {
            return;
        }
    }

    auto start = system_clock::now();
    ROS_DEBUG("edge estimate: %f (%s -> %s)", edge.matching_score_, edge.id_from_.c_str(), edge.id_to_.c_str());

    // Only make link, if the nodes match well enough.
    if (edge.matching_score_ >= config_.min_matching_score) {
        // Do not add a new edge if we have moved too much.
        Eigen::AngleAxisd diffAngleAxis(edge.transform_.linear());
        double diffRotation = fabs(diffAngleAxis.angle()) * 180 / M_PI;
        ROS_DEBUG_NAMED("graph/edge callback", "diff: (%f, %f)", edge.transform_.translation().norm(), diffRotation);
        if (edge.transform_.translation().norm() <= config_.max_edge_distance_T && diffRotation <= config_.max_edge_distance_R) {            
            std::lock_guard<std::mutex> lock(graph_mutex_);
            ROS_DEBUG_NAMED("runtime", "  acquire lock: %d ms", (int)duration_cast<milliseconds>(system_clock::now() - start).count());

            if (checkEdgeHeuristic(edge)) {
                ROS_DEBUG_NAMED("runtime", "  check heuristic: %d ms", (int)duration_cast<milliseconds>(system_clock::now() - start).count());
                if (edge.matching_score_ >= min_accept_valid_) {
                    edge.valid_ = true;
                }
                edge.id_ = boost::lexical_cast<std::string>(ros::Time::now().toSec()) + boost::uuids::to_string(uuid_generator());
                graph.addEdge(edge);
                edges_to_send_.insert(edge.id_);
                ROS_DEBUG_NAMED("runtime", "  add to graph: %d ms", (int)duration_cast<milliseconds>(system_clock::now() - start).count());

                if (edge.id_from_ > most_recent_loop_closure_) {
                    most_recent_loop_closure_ = edge.id_from_;
                }
                if (edge.id_to_ > most_recent_loop_closure_) {
                    most_recent_loop_closure_ = edge.id_to_;
                }
            } else {
                ROS_WARN("check edge heuristic failed");
            }
        }
    }
    ROS_DEBUG_NAMED("runtime", "FEATURE EDGE CALLBACK: %d ms", (int)duration_cast<milliseconds>(system_clock::now() - start).count());
}

void GraphSlamNode::newCloudEdgeCallback(SlamEdge &edge)
{
    std::lock_guard<std::mutex> lock(graph_mutex_);

    // Re-estimate, if nodes were merged.
    if (graph.isMerged(edge.id_from_) || graph.isMerged(edge.id_to_)) {
        if (edge.matching_score_ >= config_.cloud_edge_min_matching_score) {
            cloud_transformation_estimator_->estimateEdge(graph.node(graph.getMergedNodeId(edge.id_from_)),
                                                          graph.node(graph.getMergedNodeId(edge.id_to_)));
        }
        return;
    }

    // Replace existing laser edges.
    if (graph.existsEdge(edge.id_from_, edge.id_to_)) {
        if (graph.existsNode(edge.id_from_)) {
            SlamNode node = graph.node(edge.id_from_);
            for (auto edge_id : node.edges_) {
                ROS_DEBUG("try removing edge %s", edge_id.c_str());
                if (graph.existsEdge(edge_id)) {
                    SlamEdge other_edge = graph.edge(edge_id);
                    std::string other_node_id = other_edge.id_from_ == node.id_ ? other_edge.id_to_ : other_edge.id_from_;
                    if (other_node_id == edge.id_to_ && other_edge.type_ == graph_slam_msgs::Edge::TYPE_2D_LASER) {
                        graph.removeEdge(edge_id);
                        ROS_DEBUG("removing laser edge");
                    }
                }
            }
        }
    }

    auto start = system_clock::now();
    if (edge.matching_score_ >= config_.cloud_edge_min_matching_score) {
        if (graph.existsNode(edge.id_from_) && graph.existsNode(edge.id_to_)) {
            edge.id_ = boost::lexical_cast<std::string>(ros::Time::now().toSec()) + boost::uuids::to_string(uuid_generator());
            graph.addEdge(edge);
            edges_to_send_.insert(edge.id_);
        } else {
            ROS_DEBUG_NAMED("scan estimation", "if failed");
        }
    }
    ROS_DEBUG_NAMED("runtime", "SCAN EDGE CALLBACK: %d ms", (int)duration_cast<milliseconds>(system_clock::now() - start).count());
}

void GraphSlamNode::load()
{    
    graph_mutex_.lock();
    graph.loadFromDatabase();

    // Reload place recognizer.
    place_recognizer_->clear();
    for (auto node_it = graph.nodeIterator(); node_it.first != node_it.second; node_it.first++) {
        SlamNode &node = node_it.first->second;
        place_recognizer_->addPlace(node);
        ROS_INFO("load place %s / %d", node.id_.c_str(), node.sensor_data_.size());
    }
    graph_mutex_.unlock();
}

bool GraphSlamNode::mergeNodes(std::string id_first, std::string id_second)
{
    auto start2 = system_clock::now();
    ROS_DEBUG("merge nodes %s / %s", id_first.c_str(), id_second.c_str());

    {
        std::lock_guard<std::mutex> lock(graph_mutex_);
        SlamNode &first = graph.node(id_first);
        SlamNode second = graph.node(id_second);

        // Check if all edges are available.
        for (auto edge_id : second.edges_) {
            if (!graph.existsEdge(edge_id)) {
                ROS_WARN("edges missing while merge nodes, removing missing edge");
                graph.removeEdge(edge_id);
            }
        }
        ROS_DEBUG_NAMED("time", "  MERGE1: %d ms", (int)duration_cast<milliseconds>(system_clock::now() - start2).count());

    //    // Estimate laser edge.
    //    SlamEdge new_edge;
    //    laser_transformation_estimator_->do_near_ = true;
    //    laser_transformation_estimator_->estimateEdgeImpl(first, second, new_edge);
    //    laser_transformation_estimator_->do_near_ = false;
    //    if (new_edge.matching_score_ >= 50) {
    //        // Set new second node pose based on registration.
    //        second.pose_ = first.pose_ * (new_edge.displacement_from_ * new_edge.transform_ * new_edge.displacement_to_.inverse());
    //    }

        // Estimate difference between poses.
        double first_merge_count = first.stamps_.size();
        double second_merge_count = second.stamps_.size();
        double ratio = second_merge_count / (first_merge_count + second_merge_count);
        Eigen::Quaterniond q_new = Eigen::Quaterniond(first.pose_.linear()).slerp(ratio , Eigen::Quaterniond(second.pose_.linear()));
        Eigen::Vector3d t_new = (1 - ratio) * first.pose_.translation() + ratio * second.pose_.translation();
        Eigen::Translation3d trans(t_new);
        Eigen::AngleAxisd rot(q_new);
        Eigen::Isometry3d new_pose = trans * rot;
        Eigen::Isometry3d first_displacement = new_pose.inverse() * first.pose_;
        Eigen::Isometry3d second_displacement = new_pose.inverse() * second.pose_;

    //    ROS_INFO("merge ratio %f", ratio);
    //    std::cout << "first" << std::endl << first.pose_.matrix() << std::endl;
    //    std::cout << "second" << std::endl << second.pose_.matrix() << std::endl;
    //    std::cout << "new" << std::endl << new_pose.matrix() << std::endl;
    //    std::cout << "first displacement" << std::endl << first_displacement.matrix() << std::endl;
    //    std::cout << "second displacement" << std::endl << second_displacement.matrix() << std::endl;

        // Update first node.
        first.pose_ = new_pose;
        first.uncertainty_ = std::min(first.uncertainty_, second.uncertainty_);
        first.stamps_.insert(first.stamps_.end(), second.stamps_.begin(), second.stamps_.end());
        std::sort(first.stamps_.begin(), first.stamps_.end());
        for (auto edge_id : first.edges_) {
            if (graph.existsEdge(edge_id)) {
                SlamEdge &edge = graph.edge(edge_id);
                if (edge.id_from_ == id_first) {
                    edge.displacement_from_ = first_displacement * edge.displacement_from_;
                } else {
                    edge.displacement_to_ = first_displacement * edge.displacement_to_;
                }
                graph.updateDatabaseEdge(edge_id);
            }
        }
        for (auto data : first.sensor_data_) {
            data->displacement_ = first_displacement * data->displacement_;
        }

        // Update edges.
        std::vector<std::string> to_delete;
        for (auto edge_id : second.edges_) {
            if (graph.existsEdge(edge_id)) {
                SlamEdge &edge = graph.edge(edge_id);
                if (edge.id_from_ == id_first || edge.id_to_ == id_first) {             // Remove edge, if it connects the to merged nodes.
                    to_delete.push_back(edge.id_);
                } else {                                                                // Else, add displacement and update ids.
                    std::string other_id = edge.id_from_ == id_second ? edge.id_to_ : edge.id_from_;
                    // Check, if edge with the same node neighbor and type already exists.
                    if (graph.existsEdge(first.id_, other_id, edge.type_)/* || edge.type_ == graph_slam_msgs::Edge::TYPE_2D_LASER*/) {
                        ROS_DEBUG("merge: deleting edge %s", edge.id_.c_str());
                        to_delete.push_back(edge.id_);
                    } else {
                        if (edge.id_from_ == id_second) {
                            edge.displacement_from_ = second_displacement * edge.displacement_from_;
                            edge.id_from_ = id_first;
                        } else {
                            edge.displacement_to_ = second_displacement * edge.displacement_to_;
                            edge.id_to_ = id_first;
                        }
                        first.edges_.insert(edge.id_);

                        // Update edge in database.
                        graph.updateDatabaseEdge(edge.id_);
                    }
                }
            }
        }
        for (auto edge_id : to_delete) {
            graph.removeEdge(edge_id);
        }
        ROS_DEBUG_NAMED("time", "  MERGE3: %d ms", (int)duration_cast<milliseconds>(system_clock::now() - start2).count());

        // Move sensor data from second to first node.
        for (auto data : second.sensor_data_) {
            // Merge laser scan data, if there is already a laserscan.
            if (data->type_ == graph_slam_msgs::SensorData::SENSOR_TYPE_LASERSCAN) {
                bool exists_laserscan = false;
                for (size_t i = 0; i < first.sensor_data_.size(); i++) {
                    if (first.sensor_data_[i]->type_ == graph_slam_msgs::SensorData::SENSOR_TYPE_LASERSCAN) {
                        data->displacement_ = second_displacement * data->displacement_;
                        LaserscanDataPtr laser_first = boost::dynamic_pointer_cast<LaserscanData>(first.sensor_data_[i]);
                        LaserscanDataPtr laser_second = boost::dynamic_pointer_cast<LaserscanData>(data);
                        GraphGridMapper::mergeLaserScans(laser_first, laser_second);
                        exists_laserscan = true;
                        break;
                    }
                }

                if (!exists_laserscan) {
                    data->displacement_ = second_displacement * data->displacement_;
                    first.sensor_data_.push_back(data);
                }
            }

            //TODO Merge feature data.
            else if (data->type_ == graph_slam_msgs::SensorData::SENSOR_TYPE_FEATURE) {
                bool exists_features = false;
                for (size_t i = 0; i < first.sensor_data_.size(); i++) {
                    if (first.sensor_data_[i]->type_ == graph_slam_msgs::SensorData::SENSOR_TYPE_FEATURE) {
                        exists_features = true;
                        break;
                    }
                }

                if (!exists_features) {
                    data->displacement_ = second_displacement * data->displacement_;
                    first.sensor_data_.push_back(data);
                }
            }

            //TODO Merge gist descriptor data.
            else if (data->type_ == graph_slam_msgs::SensorData::SENSOR_TYPE_BINARY_GIST) {
                bool exists_gist_descriptor = false;
                for (size_t i = 0; i < first.sensor_data_.size(); i++) {
                    if (first.sensor_data_[i]->type_ == graph_slam_msgs::SensorData::SENSOR_TYPE_BINARY_GIST) {
                        exists_gist_descriptor = true;
                        break;
                    }
                }

                if (!exists_gist_descriptor) {
                    data->displacement_ = second_displacement * data->displacement_;
                    first.sensor_data_.push_back(data);
                }
            }
        }
        ROS_DEBUG_NAMED("time", "  MERGE4: %d ms", (int)duration_cast<milliseconds>(system_clock::now() - start2).count());

        // Remove old node from place recognizer.
        if (config_.do_feature_loop_closure) {
            place_recognizer_->removePlaceQueue(graph.node(id_second));
            ROS_DEBUG_NAMED("time", "  MERGE5: %d ms", (int)duration_cast<milliseconds>(system_clock::now() - start2).count());
        }

        // Remove old node.
        graph.removeNode(id_second);

        // Update merged node.
        graph.updateDatabaseNode(id_first);
        ROS_DEBUG_NAMED("time", "  MERGE6: %d ms", (int)duration_cast<milliseconds>(system_clock::now() - start2).count());
    }
    return true;
}

bool GraphSlamNode::checkEdgeHeuristic(const SlamEdge &edge)
{
    bool res = false;
    if (graph.existsNode(edge.id_from_) && graph.existsNode(edge.id_to_)) {
        graph.astar(edge.id_from_, edge.id_to_);
        double dist = graph.node(edge.id_to_).distance_;
        if (dist != std::numeric_limits<double>::max()) {
            Eigen::Isometry3d diff_pose = graph.node(edge.id_from_).pose_.inverse() * graph.node(edge.id_to_).pose_;
            Eigen::AngleAxisd diffAngleAxis(diff_pose.linear());
            double diff_rotation = 180. * diffAngleAxis.angle() / M_PI;
            if (2 * config_.scope_size_factor * dist + 1.0 > diff_pose.translation().norm() &&
                    10 * config_.scope_size_factor * dist + 30.0 > diff_rotation) {
                res = true;
            } else {
                res = false;
            }
        } else {
            res = true;
        }
    }
    return res;
}

void GraphSlamNode::graphSlamConfigCallback(graph_slam::GraphSlamConfig &config, uint32_t level)
{
    std::lock_guard<std::mutex> lock(graph_mutex_);

    // Start and stop the timers. This does nothing, if the timers were already started/stopped.
    // Reset the period in case the timer was already started.

    if (config.reevaluate_cloud_edges) {
        laser_registration_timer_ = nh_.createTimer(ros::Duration(1. / config.reevaluate_cloud_edges_freq), &GraphSlamNode::cloudRegistrationTimerCallback, this);
        laser_registration_timer_.setPeriod(ros::Duration(1. / config.reevaluate_cloud_edges_freq));
    } else {
        laser_registration_timer_.stop();
    }

    if (config.merge_nodes) {
        merge_timer_ = nh_.createTimer(ros::Duration(1. / config.merge_nodes_freq), &GraphSlamNode::mergeTimerCallback, this);
        merge_timer_.setPeriod(ros::Duration(1. / config.merge_nodes_freq));
    } else {
        merge_timer_.stop();
    }

    if (config.optimize_graph) {
        optimization_timer_ = nh_.createTimer(ros::Duration(1. / config.optimize_graph_freq), &GraphSlamNode::optimizationTimerCallback, this);
        optimization_timer_.setPeriod(ros::Duration(1. / config.optimize_graph_freq));
    } else {
        optimization_timer_.stop();
    }

    config_ = config;
}

void GraphSlamNode::linkEstimationConfigCallback(transformation_estimation::FeatureLinkEstimationConfig &config, uint32_t level)
{
    transformation_estimator_->setConfig(config);
}

void GraphSlamNode::occupancyGridConfigCallback(map_projection::OccupancyGridProjectorConfig &config, uint32_t level)
{
    projector_->setConfig(config);
}

void GraphSlamNode::placeRecognizerConfigCallback(place_recognition::PlaceRecognizerConfig &config, uint32_t level)
{
    place_recognizer_->setConfig(config);
}

void GraphSlamNode::graphOptimizerConfigCallback(graph_optimization::GraphOptimizerConfig &config, uint32_t level)
{
    optimizer_->setConfig(config);
}

void GraphSlamNode::optimizationTimerCallback(const ros::TimerEvent& e)
{
    if (graph.size() > 1) {
        std::lock_guard<std::mutex> lock(graph_mutex_);

        auto start = system_clock::now();
        ROS_DEBUG("start optimization");
        if (optimizer_->optimize(graph, boost::bind(&GraphSlamNode::finishedGraphOptimization, this))) {
            most_recent_optimizer_node_ = current_node_;
        }
        ROS_DEBUG_NAMED("runtime", "SCAN EDGE CALLBACK: %d ms", (int)duration_cast<milliseconds>(system_clock::now() - start).count());
    }
}

void GraphSlamNode::cloudRegistrationTimerCallback(const ros::TimerEvent& e)
{
    if (!graph_initialized_) {
        return;
    }

    if (graph.size() > 1) {
        graph_mutex_.lock();
        for (auto sensor_it = graph.sensorIterator(); sensor_it.first != sensor_it.second; ++sensor_it.first) {
            cloud_transformation_estimator_->sensor_transforms_[sensor_it.first->first] = sensor_it.first->second;
        }

        // Pick current node.
        if (current_scan_index_ >= graph.size()) {
            current_scan_index_ = 0;
        }

        auto it = graph.nodeIterator();
        int scan_node_id = current_scan_index_;
        std::advance(it.first, scan_node_id);
        current_scan_index_++;

        SlamNode node = it.first->second;
        estimateScanEdge(node);
        graph_mutex_.unlock();
    }
}

void GraphSlamNode::estimateScanEdge(SlamNode node1)
{
    ROS_DEBUG_NAMED("time", "SCAN EDGE ESTIMATION");
    auto start = system_clock::now();

    boost::shared_ptr<LaserTransformationEstimator> c = boost::dynamic_pointer_cast<LaserTransformationEstimator>(cloud_transformation_estimator_);
    for (auto sensor_it = graph.sensorIterator(); sensor_it.first != sensor_it.second; ++sensor_it.first) {
        c->sensor_transforms_[sensor_it.first->first] = sensor_it.first->second;
    }

    // Find laser data.
    LaserscanDataPtr laser_data_from;
    for (auto sensor_data : node1.sensor_data_) {
        if (sensor_data->type_ == graph_slam_msgs::SensorData::SENSOR_TYPE_LASERSCAN) {
            laser_data_from = boost::dynamic_pointer_cast<LaserscanData>(sensor_data);
            break;
        }
    }

    // Return, if no laser data is available.
    if (!laser_data_from) {
        return;
    }

    // Access nodes in radius.
    std::vector<std::string> ids = graph.getNodesWithinRadius(node1.id_, 8);
    ROS_DEBUG_NAMED("time", "neighbor search: %f ms", (double)duration_cast<microseconds>(system_clock::now() - start).count() / 1000.);
    start = system_clock::now();

    // Choose best matching laserscans.
    std::vector<std::pair<std::string, double> > potential_neighbors;
    for (auto node_id : ids) {
        if (graph.existsNode(node_id)) {
            SlamNode &node2 = graph.node(node_id);

            Eigen::Isometry3d diff_pose = node1.pose_.inverse() * node2.pose_;
            Eigen::AngleAxisd diffAngleAxis(diff_pose.linear());
            double diff_rotation = 180. * diffAngleAxis.angle() / M_PI;

            if (fabs(diff_rotation) > 120.) {
                for (auto sensor_data : node2.sensor_data_) {
                    if (sensor_data->type_ == graph_slam_msgs::SensorData::SENSOR_TYPE_LASERSCAN) {
                        LaserscanDataPtr laser_data_to = boost::dynamic_pointer_cast<LaserscanData>(sensor_data);

                        double center_dist = (node1.pose_ * laser_data_from->displacement_ * laser_data_from->scan_center_.homogeneous() -
                                              node2.pose_ * laser_data_to->displacement_ * laser_data_to->scan_center_.homogeneous()).norm();

                        if (center_dist < 2.0) {
                            potential_neighbors.push_back(std::make_pair(node_id, center_dist));
                        }
                    }
                }
            }
        }
    }
    std::sort(potential_neighbors.begin(), potential_neighbors.end(), PairSort);
    ROS_DEBUG_NAMED("time", "potential neighbors: %f ms", (double)duration_cast<microseconds>(system_clock::now() - start).count() / 1000.);
    start = system_clock::now();

    ROS_DEBUG("number of potential laser edges: %d", potential_neighbors.size());

    for (int i = 0; i < std::min(2, (int)potential_neighbors.size()); i++) {
        SlamNode &node2 = graph.node(potential_neighbors[i].first);
        cloud_transformation_estimator_->estimateEdge(node1, node2);
    }
    ROS_DEBUG_NAMED("time", "add to transformation estimator: %f ms", (double)duration_cast<microseconds>(system_clock::now() - start).count() / 1000.);    
}

void GraphSlamNode::finishedGraphOptimization()
{
    ROS_DEBUG("optimization finished");
    auto start = system_clock::now();
    std::lock_guard<std::mutex> lock(graph_mutex_);

    optimizer_->storeOptimizationResults(graph);
    graph.storeMetaInformation();
    graph_initialized_ = true;
    graph.reevaluateUncertainty();

    if (graph.existsNode(most_recent_optimizer_node_)) {
        Eigen::Isometry3d last_map_transform = graph.diffTransform();
        graph.diffTransform() = graph.node(most_recent_optimizer_node_).pose_ * graph.node(most_recent_optimizer_node_).sub_pose_.inverse();
        Eigen::Isometry3d diff_transform = graph.diffTransform() * last_map_transform.inverse();

        // Apply new map transform to all new nodes.
        for (auto node_it = graph.nodeIterator(); node_it.first != node_it.second; ++node_it.first) {
            if (node_it.first->first > most_recent_optimizer_node_ && !node_it.first->second.fixed_) {
                node_it.first->second.pose_ = diff_transform * node_it.first->second.pose_;
            }
        }
    }

    if (graph_display_pub_.getNumSubscribers() > 0) {
        graph_slam_msgs::Graph display_graph = Conversions::toMsg(graph, false);
        graph_display_pub_.publish(display_graph);
    }

    if (publish_occupancy_grid_) {
        projector_->project(graph);
    }

    ROS_DEBUG_NAMED("runtime", "GRAPH OPTIMIZATION FINISHED: %d ms", (int)duration_cast<milliseconds>(system_clock::now() - start).count());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "graph_slam_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    bool sync_to_database;
    nh_private.param<bool>("sync_to_database", sync_to_database, false);
    std::string name;
    nh_private.param<std::string>("name", name, "slam");

    GraphSlamNode graph_slam(nh, name, sync_to_database);

    dynamic_reconfigure::Server<transformation_estimation::FeatureLinkEstimationConfig> server_le(ros::NodeHandle("~/feature_link_estimation"));
    dynamic_reconfigure::Server<transformation_estimation::FeatureLinkEstimationConfig>::CallbackType f_le;
    f_le = boost::bind(&GraphSlamNode::linkEstimationConfigCallback, &graph_slam, _1, _2);
    server_le.setCallback(f_le);

    dynamic_reconfigure::Server<map_projection::OccupancyGridProjectorConfig> server_og(ros::NodeHandle("~/occupancy_grid_estimation"));
    dynamic_reconfigure::Server<map_projection::OccupancyGridProjectorConfig>::CallbackType f_og;
    f_og = boost::bind(&GraphSlamNode::occupancyGridConfigCallback, &graph_slam, _1, _2);
    server_og.setCallback(f_og);

    dynamic_reconfigure::Server<place_recognition::PlaceRecognizerConfig> server_pr(ros::NodeHandle("~/place_recognizer"));
    dynamic_reconfigure::Server<place_recognition::PlaceRecognizerConfig>::CallbackType f_pr;
    f_pr = boost::bind(&GraphSlamNode::placeRecognizerConfigCallback,  &graph_slam, _1, _2);
    server_pr.setCallback(f_pr);

    dynamic_reconfigure::Server<graph_optimization::GraphOptimizerConfig> server_go(ros::NodeHandle("~/graph_optimizer"));
    dynamic_reconfigure::Server<graph_optimization::GraphOptimizerConfig>::CallbackType f_go;
    f_go = boost::bind(&GraphSlamNode::graphOptimizerConfigCallback,  &graph_slam, _1, _2);
    server_go.setCallback(f_go);

    dynamic_reconfigure::Server<graph_slam::GraphSlamConfig> server_gs;
    dynamic_reconfigure::Server<graph_slam::GraphSlamConfig>::CallbackType f_gs;
    f_gs = boost::bind(&GraphSlamNode::graphSlamConfigCallback,  &graph_slam, _1, _2);
    server_gs.setCallback(f_gs);

    tf::TransformListener listener(ros::Duration(30.));

    bool is_sub_graph, is_super_graph, construct_nodes;
    nh_private.param<bool>("is_sub_graph", is_sub_graph, false);
    nh_private.param<bool>("is_super_graph", is_super_graph, false);
    nh_private.param<bool>("construct_nodes", construct_nodes, true);

    message_filters::Subscriber<graph_slam_msgs::SensorDataArray> sensor_sub;
    tf::MessageFilter<graph_slam_msgs::SensorDataArray> *sensor_sub_filter;
    if (construct_nodes) {
        sensor_sub.subscribe(nh, "/sensor_data", 10);
        sensor_sub_filter = new tf::MessageFilter<graph_slam_msgs::SensorDataArray>(sensor_sub, listener, graph_slam.odom_frame_, 10);
        sensor_sub_filter->registerCallback(boost::bind(&GraphSlamNode::sensorDataCallback, &graph_slam, _1));
    }

    ros::Subscriber sub_graph_sub;
    ros::Subscriber scope_request_sub;
    if (is_super_graph) {
        sub_graph_sub = nh.subscribe<graph_slam_msgs::Graph>("sub_graph", 10, boost::bind(&GraphSlamNode::subGraphCallback, &graph_slam, _1));
        scope_request_sub = nh.subscribe<graph_slam_msgs::Scope>("request_scope", 1, boost::bind(&GraphSlamNode::scopeRequestCallback, &graph_slam, _1));
    }

    ros::Subscriber scope_sub;
    ros::Subscriber received_sub;
    if (is_sub_graph) {
        scope_sub = nh.subscribe<graph_slam_msgs::Graph>("scope_graph", 1, boost::bind(&GraphSlamNode::scopeCallback, &graph_slam, _1));
        received_sub = nh.subscribe<graph_slam_msgs::GraphReceived>("graph_received", 1, boost::bind(&GraphSlamNode::graphReceivedCallback, &graph_slam, _1));
    }

    ros::AsyncSpinner spinner(20); // Use 20 threads
    spinner.start();
    ros::waitForShutdown();
}
