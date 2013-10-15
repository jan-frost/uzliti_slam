#include <graph_slam_tools/graph/slam_graph.h>

#include <chrono>
#include <graph_slam_tools/conversions.h>

struct DMatchComparator
{
    bool operator()(const cv::DMatch &left, const cv::DMatch &right) const
    {
        return left.distance > right.distance;
    }
};

SlamGraph::SlamGraph(std::string frame)
{
    frame_ = frame;
    clear();
}

SlamGraph::~SlamGraph()
{

}

void SlamGraph::clear()
{
    nodes_.clear();
    edges_.clear();
    most_recent_node_ = "";
    odometry_parameters_ << 1., 1., 1., 0., 0., 0.;
    sensor_transforms_.clear();
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
    if (!existsNode(id)) {
        ROS_ERROR("tried to access node that does not exist");
        assert(false);
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

Eigen::Isometry3d SlamGraph::diffTransform()
{
    return sub_transform_;
}

void SlamGraph::addNode(SlamNode node)
{
    if (merged_nodes_map_.find(node.id_) == merged_nodes_map_.end()) {
        ROS_DEBUG("add node %s", node.id_.c_str());

        nodes_[node.id_] = node;

        if (node.id_ > most_recent_node_) {
            most_recent_node_ = node.id_;
        }
    } else {
        ROS_ERROR("tried to add merged node %s", node.id_.c_str());
    }
}

void SlamGraph::removeNode(std::string id)
{
    ROS_DEBUG("remove node %s", id.c_str());
    if (existsNode(id)) {
        nodes_.erase(id);
    } else {
        ROS_ERROR("tried to remove non-existing node %s", id.c_str());
    }
}

void SlamGraph::mergeNodes(std::string id_first, std::string id_second)
{
    ROS_DEBUG("merge nodes %s / %s", id_first.c_str(), id_second.c_str());

    // Set first node as the older node.
    if (id_first > id_second) {
        std::swap(id_first, id_second);
    }
    SlamNode &first = nodes_[id_first];
    SlamNode second = nodes_[id_second];

    // Check if all edges are available.
    for (auto edge_id : second.edges_) {
        if (!existsEdge(edge_id)) {
            ROS_WARN("cannot merge nodes (edges missing) %s / %s", id_first.c_str(), id_second.c_str());
            return;
        }
    }

    // Estimate difference between poses.
    Eigen::Isometry3d displacement = first.pose_.inverse() * second.pose_;

    // Save information.
    merged_nodes_map_[second.id_] = std::make_pair(first.id_, displacement);

    // Move sensor data from second to first node.
    for (auto data : second.sensor_data_) {
        data->displacement_ = displacement * data->displacement_;
        first.sensor_data_.push_back(data);
    }

    // Update edges.
    for (auto edge_id : second.edges_) {
        if (existsEdge(edge_id)) {
            SlamEdge edge = edges_[edge_id];
            if (edge.id_from_ == id_first || edge.id_to_ == id_first) {             // Remove edge, if it connects the to merged nodes.
                first.edges_.erase(edge.id_);
                removeEdge(edge_id);
            } else {                                                                // Else, add displacement and update ids.
                if (edge.id_from_ == id_second) {
//                    edge.transform_ = displacement * edge.transform_;
                    edge.displacement_from_ = displacement * edge.displacement_from_;
                    edge.id_from_ = id_first;
                }
                if (edge.id_to_ == id_second) {
//                    edge.transform_ = edge.transform_ * displacement.inverse();
                    edge.displacement_to_ = displacement * edge.displacement_to_;
                    edge.id_to_ = id_first;
                }
                removeEdge(edge_id);
                addEdge(edge);
            }
        }
    }

    // Remove old nodes.
    removeNode(id_second);

    //TODO perform further optimization and merge sensor data.
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

void SlamGraph::addEdge(SlamEdge edge)
{
    ROS_DEBUG("add edge %s", edge.id_.c_str());

    // Check for merged nodes.
    if (merged_nodes_map_.find(edge.id_from_) != merged_nodes_map_.end()) {
//        edge.transform_ = merged_nodes_map_[edge.id_from_].second * edge.transform_;
        if (edge.displacement_from_.translation().norm() > 0) {
            ROS_ERROR("displacement from exists");
        }
        edge.displacement_from_ = edge.displacement_from_ * merged_nodes_map_[edge.id_from_].second;
        edge.id_from_ = merged_nodes_map_[edge.id_from_].first;
    }
    if (merged_nodes_map_.find(edge.id_to_) != merged_nodes_map_.end()) {
        if (edge.displacement_to_.translation().norm() > 0) {
            ROS_ERROR("displacement to exists");
        }
//        edge.transform_ = edge.transform_ * merged_nodes_map_[edge.id_to_].second.inverse();
        edge.displacement_to_ = edge.displacement_to_ * merged_nodes_map_[edge.id_to_].second;
        edge.id_to_ = merged_nodes_map_[edge.id_to_].first;
    }

    edges_[edge.id_] = edge;

    if (existsNode(edge.id_from_)) {
        nodes_[edge.id_from_].edges_.insert(edge.id_);
    }

    if (existsNode(edge.id_to_)) {
        nodes_[edge.id_to_].edges_.insert(edge.id_);
    }
}

void SlamGraph::removeEdge(std::string id)
{
    ROS_DEBUG("remove edge %s", id.c_str());

    if (existsEdge(id)) {
        edges_.erase(id);
    } else {
        ROS_ERROR("tried to remove non-existing edge %s", id.c_str());
    }
}

bool SlamGraph::existsEdge(std::string id)
{
    return (edges_.find(id) != edges_.end());
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


