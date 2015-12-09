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

#ifndef SLAM_GRAPH_H
#define SLAM_GRAPH_H

#include <cs.h>
#include <graph_slam_msgs/Graph.h>
#include <graph_slam_msgs/GraphMeta.h>
#include <graph_slam_common/slam_node.h>
#include <map>
#include <thread>
#include <mutex>
#include <boost/bimap.hpp>
#include <g2o/types/slam3d/se3quat.h>
#include <unordered_set>
#include <queue>
#include <tf/transform_listener.h>
#include <graph_slam_common/rosbag_storage.h>
#include <stack>

typedef std::string IdType;
typedef std::map<std::string, SlamNode>::iterator  NodeIterator;
typedef std::map<std::string, SlamNode>::reverse_iterator  NodeReverseIterator;
typedef std::map<std::string, SlamEdge>::iterator  EdgeIterator;
typedef std::map<std::string, SlamEdge>::reverse_iterator  EdgeReverseIterator;
typedef std::map<std::string, Eigen::Isometry3d>::iterator  SensorIterator;

class SlamGraph {
public:
    SlamGraph(std::string frame, std::string name, bool store_to_database = false);

    ~SlamGraph();

    void initializeStorage(std::string storage_path = "rosbag_storage", bool clear_database = false);

    void clear();

    void clearDatabase();

    std::string & frame();

    std::vector<std::string> nodeIds();

    SlamNode & node(std::string id);

    SlamEdge & edge(std::string id);

    g2o::Vector6d & odom();

    Eigen::Isometry3d & sensor(std::string id);

    Eigen::Isometry3d & sensorInitial(std::string id);

    std::string & mostRecentNode();

    Eigen::Isometry3d &diffTransform();

    void addNode(SlamNode node);

    void updateDatabaseNode(std::string id);

    void removeNode(std::string id);

    void mergeNodes(std::string id_first, std::string id_second);

    std::string getMergedNodeId(std::string id);

    bool isMerged(std::string id);

    bool existsNode(std::string id);

    std::set<std::string> getOutOfScopeNodes(std::string center_node_id, float radius);

    void getNodesWithinHopsHelper(std::string start, std::unordered_set<std::string> &neighbor_nodes, int hops, bool only_valid);
    void getNodesWithinHops(std::string start, std::unordered_set<std::string> &neighbor_nodes, int hops, bool only_valid = true);
    std::vector<std::string> getNodesWithinRadius(std::string v, double r);

    std::vector<std::string> getNodesAfter(std::string id);

    void addEdge(SlamEdge edge);

    void updateDatabaseEdge(std::string id);

    void removeEdge(std::string id);

    bool existsEdge(std::string id);

    bool existsEdge(std::string from, std::string to);

    bool existsEdge(std::string from, std::string to, unsigned char type);

    bool existsEdgeWithinHops(std::string from, std::string to, int hops, bool only_valid = false);

    void addSensor(std::string id, Eigen::Isometry3d transform);

    void removeSensor(std::string id);

    bool existsSensor(std::string id);

    int size();

    int edgeCount();

    std::pair<NodeIterator, NodeIterator> nodeIterator();

    std::pair<NodeReverseIterator, NodeReverseIterator> nodeReverseIterator();

    std::pair<EdgeIterator, EdgeIterator> edgeIterator();

    std::pair<EdgeReverseIterator, EdgeReverseIterator> edgeReverseIterator();

    std::pair<SensorIterator, SensorIterator> sensorIterator();

    void setGlobalOffset(const Eigen::Isometry3d &offset);
    Eigen::Isometry3d getGlobalOffset();

    void setGPSZone(const std::string &zone);
    std::string getGPSZone();

    void reevaluateUncertainty();
    void reevaluateUncertainty(std::string id);

    std::vector<std::string> getNeighbors(std::string id, bool only_valid = false);

    std::vector<std::string> getNeighborsDirected(std::string id, bool only_valid = false);

    graph_slam_msgs::GraphMeta toMetaData(tf::TransformListener &tfl);
    void updateMetaData(graph_slam_msgs::GraphMeta meta);

    void loadFromDatabase();

    void storeToDatabase();

    void storeMetaInformation();

    void topologicalSortUtil(std::string v, std::stack<std::string> &stack);
    void primAddEdges(SlamNode &node, std::priority_queue<SlamEdgeWithPriority> &queue);
    void distanceToSource(std::string start);
    void dijkstra(std::string start);
    void dijkstra(std::string source, std::string target);
    double heuristic_cost(std::string source, std::string target, double epsilon);
    void astar(std::string source, std::string target, double epsilon = 1.);

    void oldestReachableNode(std::string current_node, std::string &oldest_node);

private:
    /**< List of nodes, where the node identifier acts as the key to the corresponding SlamNode object.*/
    std::map<std::string, SlamNode, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string, SlamNode> > > nodes_;
    std::map<std::string, std::string> merged_nodes_map_;

    /**< List of edges, where the edge identifier acts as the key to the corresponding SlamEdge object.*/
    std::map<std::string, SlamEdge, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string, SlamEdge> > > edges_;

    /**< Map of sensor transforms, which encode the position of a sensor relative to the robot center. */
    std::map<std::string, Eigen::Isometry3d, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string, Eigen::Isometry3d> > > sensor_transforms_;
    std::map<std::string, Eigen::Isometry3d, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string, Eigen::Isometry3d> > > sensor_transforms_initial_;

    Eigen::Isometry3d sub_transform_;

    Eigen::Isometry3d gps_offset_;
    std::string gps_zone_;
    bool has_gps_measurement_;

    /**< Calibration data for the wheel odometry. */
    g2o::Vector6d odometry_parameters_;

    std::string frame_;
    std::string name_;
    bool sync_to_database_;
    std::string most_recent_node_;

    boost::shared_ptr<SlamStorage> storage_;
};

#endif // SLAM_GRAPH_H
