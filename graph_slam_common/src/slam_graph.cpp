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

#include <graph_slam_common/slam_graph.h>

#include <chrono>
#include <graph_slam_common/conversions.h>
#include <boost/heap/fibonacci_heap.hpp>
#include <unordered_map>
#include <fstream>
#include <rosbag/bag.h>

using std::chrono::duration_cast;
using std::chrono::microseconds;
using std::chrono::milliseconds;
using std::chrono::minutes;
using std::chrono::system_clock;

struct DMatchComparator
{
    bool operator()(const cv::DMatch &left, const cv::DMatch &right) const
    {
        return left.distance > right.distance;
    }
};

struct compare_node
{
    bool operator()(const SlamNodeWithPriority& n1, const SlamNodeWithPriority& n2) const
    {
        return n1.weight_ > n2.weight_;
    }
};

SlamGraph::SlamGraph(std::string frame, std::string name, bool store_to_database) :
    frame_(frame),
    name_(name),
    sync_to_database_(store_to_database)
{
    clear();
}

SlamGraph::~SlamGraph()
{
}

void SlamGraph::initializeStorage(std::string storage_path, bool clear_database)
{
    ros::NodeHandle nh;
    storage_.reset(new RosbagStorage(nh, storage_path + "/" + name_, clear_database, false));
}

void SlamGraph::clear()
{
    nodes_.clear();
    edges_.clear();
    most_recent_node_ = "";
    odometry_parameters_ << 1, 0, 0, 0., 0., 0.;

    sensor_transforms_.clear();
    gps_offset_ = Eigen::Isometry3d::Identity();
    sub_transform_ = Eigen::Isometry3d::Identity();
}

void SlamGraph::clearDatabase()
{
    storage_->clear();
}

std::string &SlamGraph::frame()
{
    return frame_;
}

std::vector<std::string> SlamGraph::nodeIds()
{
    std::vector<std::string> ids;
    for (auto node_it = nodes_.begin(); node_it != nodes_.end(); ++node_it) {
        ids.push_back(node_it->first);
    }
    return ids;
}

SlamNode &SlamGraph::node(std::string id)
{
    if (merged_nodes_map_.find(id) != merged_nodes_map_.end()) {
        ROS_ERROR("tried to acces merged node: %s is ", id.c_str(), merged_nodes_map_[id].c_str());
        std::cout << "bla" << std::endl;
    }
    if (!existsNode(id)) {
        ROS_ERROR("tried to access node that does not exist: %s", id.c_str());
        std::cout << "bla" << std::endl;
        SlamNode * n = NULL;
        std::cout << n->uncertainty_ << std::endl;

    }
    return nodes_[id];
}

SlamEdge &SlamGraph::edge(std::string id)
{
    return edges_[id];
}

g2o::Vector6d &SlamGraph::odom()
{
    return odometry_parameters_;
}

Eigen::Isometry3d &SlamGraph::sensor(std::string id)
{
    return sensor_transforms_[id];
}

Eigen::Isometry3d &SlamGraph::sensorInitial(std::string id)
{
    return sensor_transforms_initial_[id];
}

std::string &SlamGraph::mostRecentNode()
{
    return most_recent_node_;
}

Eigen::Isometry3d &SlamGraph::diffTransform()
{
    return sub_transform_;
}

void SlamGraph::addNode(SlamNode node)
{
    if (merged_nodes_map_.find(node.id_) != merged_nodes_map_.end()) {
        ROS_ERROR("add merged node");
        return;
    }
    if (nodes_.find(node.id_) != nodes_.end()) {
        ROS_WARN("add existing node");
    } else if (sync_to_database_) {
        storage_->storeNode(node);
    }

    ROS_DEBUG("add node %s", node.id_.c_str());
    nodes_.insert(std::make_pair(node.id_, node));

    if (node.id_ > most_recent_node_) {
        most_recent_node_ = node.id_;
    }
}

void SlamGraph::updateDatabaseNode(std::string id)
{
    // Delete node, if it exists.
    if (sync_to_database_) {
        if (existsNode(id)) {
            storage_->storeNode(nodes_[id]);
        }
    }
}

void SlamGraph::removeNode(std::string id)
{
    if (existsNode(id)) {
        ROS_DEBUG_NAMED("slam_graph", "remove node %s", id.c_str());
        nodes_.erase(id);
    }

    if (sync_to_database_) {
        auto runtime_start = system_clock::now();
        storage_->removeNode(id);
        double runtime = (0.001 * duration_cast<microseconds>(system_clock::now() - runtime_start).count());
        ROS_DEBUG_NAMED("slam_graph", "remove node: %f ms", runtime);
    }
}

void SlamGraph::mergeNodes(std::string id_first, std::string id_second)
{
    merged_nodes_map_[id_first] = id_second;
    ROS_DEBUG_NAMED("slam_graph", "merge remap %s -> %s", id_first.c_str(), id_second.c_str());
}

std::string SlamGraph::getMergedNodeId(std::string id)
{
    std::string res = id;
    if (isMerged(res)) {
        res = merged_nodes_map_[id];
    }
    return res;
}

bool SlamGraph::isMerged(std::string id)
{
    return (merged_nodes_map_.find(id) != merged_nodes_map_.end());
}

bool SlamGraph::existsNode(std::string id)
{
    return (nodes_.find(id) != nodes_.end());
}

std::set<std::string> SlamGraph::getOutOfScopeNodes(std::string center_node_id, float radius)
{
    std::set<std::string> out_of_scope_nodes;
    if (existsNode(center_node_id)) {
        SlamNode &center_node = nodes_[center_node_id];
        for (auto node_it = nodes_.begin(); node_it != nodes_.end(); node_it++) {
            SlamNode &node = node_it->second;
            if ((node.pose_.translation() - center_node.pose_.translation()).norm() > radius) {
                out_of_scope_nodes.insert(node.id_);
            }
        }
    }
    return out_of_scope_nodes;
}

void SlamGraph::getNodesWithinHopsHelper(std::string start, std::unordered_set<std::string> &neighbor_nodes, int hops, bool only_valid)
{
    neighbor_nodes.insert(start);
    SlamNode &node = nodes_[start];
    node.visited_ = true;

    if (hops > 0) {
        for (auto edge_id : node.edges_) {
            if (existsEdge(edge_id)) {
                SlamEdge &edge = edges_[edge_id];
                if (!only_valid || edge.valid_) {
                    std::string next_node_id = edge.id_from_ == node.id_ ? edge.id_to_ : edge.id_from_;
                    if (existsNode(next_node_id)) {
                        SlamNode &next_node = nodes_[next_node_id];
                        if (!next_node.visited_) {
                            getNodesWithinHopsHelper(next_node_id, neighbor_nodes, hops - 1, only_valid);
                        }
                    }
                }
            }
        }
    }
}

void SlamGraph::getNodesWithinHops(std::string start, std::unordered_set<std::string> &neighbor_nodes, int hops, bool only_valid)
{
    if (existsNode(start)) {
        for (auto node_it = nodes_.begin(); node_it != nodes_.end(); ++node_it) {
            node_it->second.visited_ = false;
        }

        getNodesWithinHopsHelper(start, neighbor_nodes, hops, only_valid);
    }
}

std::vector<std::string> SlamGraph::getNodesWithinRadius(std::string v, double r)
{
    std::vector<std::string> res;
    if (existsNode(v)) {
        const SlamNode &node = nodes_[v];
        for (auto node_it = nodes_.begin(); node_it != nodes_.end(); ++node_it) {
            if (node_it->first != v && (node_it->second.pose_.translation() - node.pose_.translation()).norm() < r) {
                res.push_back(node_it->first);
            }
        }
    }
    return res;
}

std::vector<std::string> SlamGraph::getNodesAfter(std::string id)
{
    std::vector<std::string> nodes_after;

    auto node_it = nodes_.find(id);
    if (node_it == nodes_.end()) {
        for (node_it = nodes_.begin(); node_it != nodes_.end(); ++node_it) {
            nodes_after.push_back(node_it->first);
        }
    } else {
        for (; node_it != nodes_.end(); ++node_it) {
            nodes_after.push_back(node_it->first);
        }
    }

    return nodes_after;
}

void SlamGraph::addEdge(SlamEdge edge)
{
    if (existsEdge(edge.id_)) {
        ROS_WARN("add existing edge");
        return;
    }

    ROS_DEBUG("add edge %s", edge.id_.c_str());

    // Check for merged nodes.
    if (merged_nodes_map_.find(edge.id_from_) != merged_nodes_map_.end()) {
        ROS_ERROR("tried to add edge for merged node");

//        if (edge.displacement_from_.translation().norm() > 0) {
//            ROS_ERROR("displacement from exists");
//        }
//        edge.displacement_from_ = edge.displacement_from_ * merged_nodes_map_[edge.id_from_].second;
//        edge.id_from_ = merged_nodes_map_[edge.id_from_].first;
    }
    if (merged_nodes_map_.find(edge.id_to_) != merged_nodes_map_.end()) {
        ROS_ERROR("tried to add edge for merged node");

//        if (edge.displacement_to_.translation().norm() > 0) {
//            ROS_ERROR("displacement to exists");
//        }
//        edge.displacement_to_ = edge.displacement_to_ * merged_nodes_map_[edge.id_to_].second;
//        edge.id_to_ = merged_nodes_map_[edge.id_to_].first;
    }

    edges_[edge.id_] = edge;

    if (existsNode(edge.id_from_)) {
        nodes_[edge.id_from_].edges_.insert(edge.id_);
    }

    if (existsNode(edge.id_to_)) {
        nodes_[edge.id_to_].edges_.insert(edge.id_);
    }

    if (sync_to_database_) {
        storage_->storeEdge(edge);
    }
}

void SlamGraph::updateDatabaseEdge(std::string id)
{
    if (sync_to_database_) {
        // Add node.
        if (existsEdge(id)) {
            storage_->storeEdge(edges_[id]);
        }
    }
}

void SlamGraph::removeEdge(std::string id)
{
    if (existsEdge(id)) {
        ROS_DEBUG_NAMED("slam_graph", "remove edge %s", id.c_str());
        SlamEdge edge = edges_[id];
        edges_.erase(id);

        if (existsNode(edge.id_from_)) {
            nodes_[edge.id_from_].edges_.erase(edge.id_);

    //        if (sync_to_database_) {
    //            updateDatabaseNode(edge.id_from_);
    //        }
        }

        if (existsNode(edge.id_to_)) {
            nodes_[edge.id_to_].edges_.erase(edge.id_);

    //        if (sync_to_database_) {
    //            updateDatabaseNode(edge.id_to_);
    //        }
        }
    } else {
        ROS_ERROR("remove non-existing edge");
    }

    if (sync_to_database_) {
        auto runtime_start = system_clock::now();
        storage_->removeEdge(id);
        double runtime = (0.001 * duration_cast<microseconds>(system_clock::now() - runtime_start).count());
        ROS_DEBUG_NAMED("slam_graph", "remove edge: %f ms", runtime);
    }
}

bool SlamGraph::existsEdge(std::string id)
{
    return (edges_.find(id) != edges_.end());
}

bool SlamGraph::existsEdge(std::string from, std::string to)
{
    if (existsNode(from) && existsNode(to)) {
        for (auto edge_id : nodes_[from].edges_) {
            if (existsEdge(edge_id)) {
                SlamEdge &edge = edges_[edge_id];
                if ((edge.id_from_ == from && edge.id_to_ == to) ||
                        (edge.id_from_ == to && edge.id_to_ == from)) {
                    return true;
                }
            }
        }
    }
    return false;
}

bool SlamGraph::existsEdge(std::string from, std::string to, unsigned char type)
{
    if (existsNode(from) && existsNode(to)) {
        for (auto edge_id : nodes_[from].edges_) {
            if (existsEdge(edge_id)) {
                SlamEdge &edge = edges_[edge_id];
                if (((edge.id_from_ == from && edge.id_to_ == to) ||
                        (edge.id_from_ == to && edge.id_to_ == from)) &&
                        edge.type_ == type) {
                    return true;
                }
            }
        }
    }
    return false;
}

bool SlamGraph::existsEdgeWithinHops(std::string from, std::string to, int hops, bool only_valid)
{
    std::unordered_set<std::string> neighbor_nodes;
    getNodesWithinHops(from, neighbor_nodes, hops, only_valid);

    return (neighbor_nodes.find(to) != neighbor_nodes.end());
}

void SlamGraph::addSensor(std::string id, Eigen::Isometry3d transform)
{
    sensor_transforms_[id] = transform;
    sensor_transforms_initial_[id] = transform;
}

void SlamGraph::removeSensor(std::string id)
{
    if (existsSensor(id)) {
        sensor_transforms_.erase(id);
        sensor_transforms_initial_.erase(id);
    }
}

bool SlamGraph::existsSensor(std::string id)
{
    return (sensor_transforms_.find(id) != sensor_transforms_.end());
}

int SlamGraph::size()
{
    return (int)nodes_.size();
}

int SlamGraph::edgeCount()
{
    return (int)edges_.size();
}

std::pair<NodeIterator, NodeIterator> SlamGraph::nodeIterator()
{
    return std::make_pair<NodeIterator, NodeIterator>(nodes_.begin(), nodes_.end());
}

std::pair<NodeReverseIterator, NodeReverseIterator> SlamGraph::nodeReverseIterator()
{
    return std::make_pair<NodeReverseIterator, NodeReverseIterator>(nodes_.rbegin(), nodes_.rend());
}

std::pair<EdgeIterator, EdgeIterator> SlamGraph::edgeIterator()
{
    return std::make_pair<EdgeIterator, EdgeIterator>(edges_.begin(), edges_.end());
}

std::pair<EdgeReverseIterator, EdgeReverseIterator> SlamGraph::edgeReverseIterator()
{
    return std::make_pair<EdgeReverseIterator, EdgeReverseIterator>(edges_.rbegin(), edges_.rend());
}

std::pair<SensorIterator, SensorIterator> SlamGraph::sensorIterator()
{
    return std::make_pair<SensorIterator, SensorIterator>(sensor_transforms_.begin(), sensor_transforms_.end());
}

void SlamGraph::setGlobalOffset(const Eigen::Isometry3d &offset)
{
    gps_offset_ = offset;
}

Eigen::Isometry3d SlamGraph::getGlobalOffset()
{
    return gps_offset_;
}

void SlamGraph::setGPSZone(const std::string &zone)
{
    gps_zone_ = zone;
}

std::string SlamGraph::getGPSZone()
{
    return gps_zone_;
}

void SlamGraph::reevaluateUncertainty()
{
    if (size() > 0) {
        dijkstra(nodes_.begin()->first);

        for (auto node_it = nodes_.begin(); node_it != nodes_.end(); ++node_it) {
            if (node_it->second.distance_ != std::numeric_limits<double>::max()) {
                node_it->second.uncertainty_ = node_it->second.distance_;
            }
        }
    }
}

void SlamGraph::reevaluateUncertainty(std::string id)
{
//    if (!existsNode(id)) {
//        return;
//    }

//    SlamNode &node = nodes_[id];
//    for (auto edge_id : node.edges_) {
//        if (existsEdge(edge_id)) {
//            SlamEdge &edge = edges_[edge_id];
////            if (edge.valid_) {
//                std::string next_node_id = edge.id_from_ == node.id_ ? edge.id_to_ : edge.id_from_;
//                double new_uncertainty = node.uncertainty_ + 1. * (1 / sqrt(edge.information_(0,0)));
//                if (existsNode(next_node_id) && new_uncertainty < nodes_[next_node_id].uncertainty_) {
//                    nodes_[next_node_id].uncertainty_ = new_uncertainty;
//                    reevaluateUncertainty(next_node_id);
//                }
////            }
//        }
//    }
    if (existsNode(id)) {
        // Get oldest reachable node.
        for (auto node_it = nodes_.begin(); node_it != nodes_.end(); ++node_it) {
            node_it->second.visited_ = false;
        }
        std::string start = id;
        oldestReachableNode(id, start);

        double initial_uncertainty = nodes_[start].uncertainty_;
        dijkstra(start);

        for (auto node_it = nodes_.begin(); node_it != nodes_.end(); ++node_it) {
            if (node_it->second.distance_ != std::numeric_limits<double>::max()) {
                node_it->second.uncertainty_ = initial_uncertainty + node_it->second.distance_;
            }
        }
    }
}

std::vector<std::string> SlamGraph::getNeighbors(std::string id, bool only_valid)
{
    std::vector<std::string> res;
    if (!existsNode(id)) {
        return res;
    }

    SlamNode &node = nodes_[id];
    for (auto edge_id : node.edges_) {
        if (existsEdge(edge_id)) {
            SlamEdge &edge = edges_[edge_id];
            if ((!only_valid || edge.valid_) && edge.type_ != graph_slam_msgs::Edge::TYPE_2D_LASER) {
                std::string next_node_id = edge.id_from_ == node.id_ ? edge.id_to_ : edge.id_from_;
                if (existsNode(next_node_id)) {
                    res.push_back(next_node_id);
                }
            }
        }
    }
    return res;
}

std::vector<std::string> SlamGraph::getNeighborsDirected(std::string id, bool only_valid)
{
    std::vector<std::string> res_all = getNeighbors(id, only_valid);
    std::vector<std::string> res;
    for (auto u : res_all) {
        if (u > id) {
            res.push_back(u);
        }
    }
    return res;
}

graph_slam_msgs::GraphMeta SlamGraph::toMetaData(tf::TransformListener &tfl)
{
    graph_slam_msgs::GraphMeta meta;
    meta.header.stamp = ros::Time::now();
    meta.header.frame_id = frame_;
    meta.name = name_;

    Eigen::Isometry3d map_pose = Eigen::Isometry3d::Identity();
    ros::Time request_time = ros::Time(0);
    Conversions::getTransform(tfl, map_pose, "/map", "/base_footprint", request_time);
    meta.map_transform = Conversions::toMsg(map_pose);

    for (auto sensor_it = sensor_transforms_.begin(); sensor_it != sensor_transforms_.end(); ++sensor_it) {
        graph_slam_msgs::SensorTransform sensor_transform_msg;
        sensor_transform_msg.sensor_name = sensor_it->first;
        sensor_transform_msg.transform = Conversions::toMsg(sensor_it->second);
        meta.sensor_transforms.push_back(sensor_transform_msg);
    }
    for (auto sensor_it = sensor_transforms_initial_.begin(); sensor_it != sensor_transforms_initial_.end(); ++sensor_it) {
        graph_slam_msgs::SensorTransform sensor_transform_msg;
        sensor_transform_msg.sensor_name = sensor_it->first;
        sensor_transform_msg.transform = Conversions::toMsg(sensor_it->second);
        meta.sensor_transforms_initial.push_back(sensor_transform_msg);
    }
    Conversions::toMsg(odometry_parameters_, meta.odometry_parameters);

    return meta;
}

void SlamGraph::updateMetaData(graph_slam_msgs::GraphMeta meta)
{
    frame_ = meta.header.frame_id;
    name_ = meta.name;
    sub_transform_ = Conversions::fromMsg(meta.map_transform);
    for (auto sensor : meta.sensor_transforms) {
        addSensor(sensor.sensor_name, Conversions::fromMsg(sensor.transform));
    }
    for (auto sensor : meta.sensor_transforms_initial) {
        sensorInitial(sensor.sensor_name) = Conversions::fromMsg(sensor.transform);
    }
    odometry_parameters_ = Conversions::fromMsg(meta.odometry_parameters);
}

void SlamGraph::loadFromDatabase()
{
    storage_->loadGraph(*this);
}

void SlamGraph::storeToDatabase()
{
    // Add meta data.
    ROS_INFO("store meta data");
    storeMetaInformation();

    // Add nodes.
    for (auto node_it = nodes_.begin(); node_it != nodes_.end(); ++node_it) {
        storage_->storeNode(node_it->second);
    }

    // Add edges.
    for (auto edge_it = edges_.begin(); edge_it != edges_.end(); ++edge_it) {
        storage_->storeEdge(edge_it->second);
    }
}

void SlamGraph::storeMetaInformation()
{
    storage_->storeMetaData(*this);
}

void SlamGraph::topologicalSortUtil(std::string v, std::stack<std::string> &stack)
{
    // Mark the current node as visited
    if (existsNode(v)) {
        SlamNode &v_node = nodes_[v];
        if (!v_node.visited_) {
            v_node.visited_ = true;

            // Recur for all the vertices adjacent to this vertex
            for (auto u : getNeighborsDirected(v, true)) {
                topologicalSortUtil(u, stack);
            }

            // Push current vertex to stack which stores topological sort
            stack.push(v);
        }
    }
}

void SlamGraph::primAddEdges(SlamNode &node, std::priority_queue<SlamEdgeWithPriority> &queue)
{
    for (auto add_edge_id : node.edges_) {
        if (existsEdge(add_edge_id)) {
            SlamEdge &edge = edges_[add_edge_id];
            if (existsNode(edge.id_from_) && existsNode(edge.id_to_)) {
                SlamNode &from = nodes_[edge.id_from_];
                SlamNode &to = nodes_[edge.id_to_];
                if (!from.visited_ || !to.visited_) {
                    queue.push(SlamEdgeWithPriority(add_edge_id, (from.pose_.translation() - to.pose_.translation()).norm()));
                }
            }
        }
    }
}

void SlamGraph::distanceToSource(std::string start)
{
//    if (existsNode(start)) {
//        // Mark all the vertices as not visited, distance to infinity.
//        for (auto node_it = nodes_.begin(); node_it != nodes_.end(); ++node_it) {
//            node_it->second.visited_ = false;
//            node_it->second.distance_ = std::numeric_limits<double>::max();
//        }

//        nodes_[start].distance_ = 0.;
//        std::stack<std::string> stack;
//        topologicalSortUtil(start, stack);

//        // Process vertices in topological order
//        while (!stack.empty()) {
//            // Get the next vertex from topological order
//            auto v = stack.top();
//            stack.pop();

//            // Update distances of all adjacent vertices
//            if (nodes_[v].distance_ != std::numeric_limits<double>::max()) {
//                for (auto u : getNeighborsDirected(v, true)) {
//                    if (existsNode(u)) {
//                        double dist = (nodes_[v].pose_.translation() - nodes_[u].pose_.translation()).norm();
//                        double new_dist = nodes_[v].distance_ + dist;
//                        if (new_dist < nodes_[u].distance_) {
//                            nodes_[u].distance_ = new_dist;
//                        }
//                    }
//                }
//            }
//        }
//    }

    if (existsNode(start)) {
        // Mark all the vertices as not visited, distance to infinity.
        for (auto node_it = nodes_.begin(); node_it != nodes_.end(); ++node_it) {
            node_it->second.visited_ = false;
            node_it->second.distance_ = std::numeric_limits<double>::max();
        }

        std::priority_queue<SlamEdgeWithPriority> edge_queue;
        nodes_[start].visited_ = true;
        nodes_[start].distance_ = 0.;
        primAddEdges(nodes_[start], edge_queue);

        int visited_count = 1;
        while (!edge_queue.empty() && visited_count < size()) {
            SlamEdgeWithPriority edge_prio = edge_queue.top();
            edge_queue.pop();
            SlamEdge &edge = edges_[edge_prio.id_];
            SlamNode &from = nodes_[edge.id_from_];
            SlamNode &to = nodes_[edge.id_to_];
            if (!from.visited_ && to.visited_) {
                from.visited_ = true;
                from.distance_ = to.distance_ + edge_prio.weight_;
                primAddEdges(from, edge_queue);
                visited_count++;
            } else if (!to.visited_ && from.visited_) {
                to.visited_ = true;
                to.distance_ = from.distance_ + edge_prio.weight_;
                primAddEdges(to, edge_queue);
                visited_count++;
            }
        }
    }
}

void SlamGraph::dijkstra(std::string start) {
    if (existsNode(start)) {
        // Mark all the vertices as not visited, distance to infinity.
        for (auto node_it = nodes_.begin(); node_it != nodes_.end(); ++node_it) {
            node_it->second.visited_ = false;
            node_it->second.distance_ = std::numeric_limits<double>::max();
        }

        nodes_[start].visited_ = true;
        nodes_[start].distance_ = 0.;

        std::priority_queue<SlamNodeWithPriority> node_queue;
        node_queue.push(SlamNodeWithPriority(start, nodes_[start].distance_));

        while (!node_queue.empty()) {
            SlamNodeWithPriority v = node_queue.top();
            node_queue.pop();

            if (v.weight_ == std::numeric_limits<double>::max()) {
                break;
            }

            for (auto u : getNeighbors(v.id_, true)) {
                double dist = (nodes_[v.id_].pose_.translation() - nodes_[u].pose_.translation()).norm();
                double new_dist = nodes_[v.id_].distance_ + dist;
                if (new_dist < nodes_[u].distance_) {
                    nodes_[u].distance_ = new_dist;
                    node_queue.push(SlamNodeWithPriority(u, nodes_[u].distance_));
                }
            }
        }
    }
}

void SlamGraph::dijkstra(std::string source, std::string target)
{
    if (existsNode(source)) {
        // Mark all the vertices as not visited, distance to infinity.
        for (auto node_it = nodes_.begin(); node_it != nodes_.end(); ++node_it) {
            node_it->second.visited_ = false;
            node_it->second.distance_ = std::numeric_limits<double>::max();
        }

        nodes_[source].visited_ = true;
        nodes_[source].distance_ = 0.;

        std::priority_queue<SlamNodeWithPriority> node_queue;
        node_queue.push(SlamNodeWithPriority(source, nodes_[source].distance_));

        while (!node_queue.empty()) {
            SlamNodeWithPriority v = node_queue.top();
            node_queue.pop();

            if (v.id_ == target) {
                break;
            }

            if (v.weight_ == std::numeric_limits<double>::max()) {
                break;
            }

            for (auto u : getNeighbors(v.id_, true)) {
                double dist = (nodes_[v.id_].pose_.translation() - nodes_[u].pose_.translation()).norm();
                double new_dist = nodes_[v.id_].distance_ + dist;
                if (new_dist < nodes_[u].distance_) {
                    nodes_[u].distance_ = new_dist;
                    node_queue.push(SlamNodeWithPriority(u, nodes_[u].distance_));
                }
            }
        }
    }
}


double SlamGraph::heuristic_cost(std::string source, std::string target, double epsilon)
{
    return epsilon * (nodes_[source].pose_.translation() - nodes_[target].pose_.translation()).norm();
}

void SlamGraph::astar(std::string source, std::string target, double epsilon)
{
    if (!existsNode(source) || !existsNode(target)) {
        return;
    }

    std::unordered_set<std::string> open_set;
    open_set.insert(source);
    std::unordered_set<std::string> closed_set;
    std::unordered_map<std::string, double> g_score;
    g_score[source] = 0;

    boost::heap::fibonacci_heap<SlamNodeWithPriority, boost::heap::compare<compare_node> > f_score;
    f_score.push(SlamNodeWithPriority(source, heuristic_cost(source, target, epsilon)));

    bool success = false;
    while (!open_set.empty()) {
         SlamNodeWithPriority v = f_score.top();
         if (v.id_ == target) {
             success = true;
             break;
         }

         f_score.pop();
         open_set.erase(v.id_);
         closed_set.insert(v.id_);

         for (auto u : getNeighbors(v.id_, true)) {
             if (closed_set.find(u) != closed_set.end() || !existsNode(u)) {
                 continue;
             }

             double tentative_g_score = g_score[v.id_] + heuristic_cost(v.id_, u, 1.);
             if (open_set.find(u) == open_set.end() || tentative_g_score < g_score[u]) {
                 g_score[u] = tentative_g_score;
                 f_score.push(SlamNodeWithPriority(u, heuristic_cost(u, target, epsilon)));
                 open_set.insert(u);
             }
         }
    }

    if (success) {
        nodes_[target].distance_ = g_score[target];
    } else {
        nodes_[target].distance_ = std::numeric_limits<double>::max();
    }
}

void SlamGraph::oldestReachableNode(std::string v, std::string &oldest_node)
{
    // Mark the current node as visited
    if (existsNode(v)) {
        if (v < oldest_node) {
            oldest_node = v;
        }

        SlamNode &v_node = nodes_[v];
        if (!v_node.visited_) {
            v_node.visited_ = true;

            // Recur for all the vertices adjacent to this vertex
            for (auto u : getNeighbors(v)) {
                oldestReachableNode(u, oldest_node);
            }
        }
    }
}


