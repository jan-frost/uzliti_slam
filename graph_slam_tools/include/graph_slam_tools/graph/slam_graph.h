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

#ifndef SLAM_GRAPH_H
#define SLAM_GRAPH_H

#include <cs.h>
#include <graph_slam_tools/graph/slam_node.h>
#include <map>
#include <thread>
#include <mutex>
#include <boost/bimap.hpp>
#include <g2o/types/slam3d/se3quat.h>

typedef std::string IdType;
typedef std::map<std::string, SlamNode>::iterator  NodeIterator;
typedef std::map<std::string, SlamNode>::reverse_iterator  NodeReverseIterator;
typedef std::map<std::string, SlamEdge>::iterator  EdgeIterator;
typedef std::map<std::string, SlamEdge>::reverse_iterator  EdgeReverseIterator;
typedef std::map<std::string, Eigen::Isometry3d>::iterator  SensorIterator;

class SlamGraph {
public:
    SlamGraph(std::string frame);

    ~SlamGraph();

    void clear();

    std::string & frame();

    std::vector<std::string> nodeIds();

    SlamNode & node(std::string id);

    SlamEdge & edge(std::string id);

    g2o::Vector6d & odom();

    Eigen::Isometry3d & sensor(std::string id);

    Eigen::Isometry3d & sensorInitial(std::string id);

    std::string & mostRecentNode();

    Eigen::Isometry3d diffTransform();

    void addNode(SlamNode node);

    void removeNode(std::string id);

    void mergeNodes(std::string id_first, std::string id_second);

    bool existsNode(std::string id);

    std::set<std::string> getOutOfScopeNodes(std::string center_node_id, float radius);

    void addEdge(SlamEdge edge);

    void removeEdge(std::string id);

    bool existsEdge(std::string id);

    void addSensor(std::string id, Eigen::Isometry3d transform);

    void removeSensor(std::string id);

    bool existsSensor(std::string id);

    int size();

    std::pair<NodeIterator, NodeIterator> nodeIterator();

    std::pair<NodeReverseIterator, NodeReverseIterator> nodeReverseIterator();

    std::pair<EdgeIterator, EdgeIterator> edgeIterator();

    std::pair<EdgeReverseIterator, EdgeReverseIterator> edgeReverseIterator();

    std::pair<SensorIterator, SensorIterator> sensorIterator();

private:
    /**< List of nodes, where the node identifier acts as the key to the corresponding SlamNode object.*/
    std::map<std::string, SlamNode, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string, SlamNode> > > nodes_;
    std::map<std::string, std::pair<std::string, Eigen::Isometry3d>, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string, std::pair<std::string, Eigen::Isometry3d> > >  > merged_nodes_map_;

    /**< List of edges, where the edge identifier acts as the key to the corresponding SlamEdge object.*/
    std::map<std::string, SlamEdge, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string, SlamEdge> > > edges_;

    /**< Map of sensor transforms, which encode the position of a sensor relative to the robot center. */
    std::map<std::string, Eigen::Isometry3d, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string, Eigen::Isometry3d> > > sensor_transforms_;
    std::map<std::string, Eigen::Isometry3d, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string, Eigen::Isometry3d> > > sensor_transforms_initial_;

    Eigen::Isometry3d sub_transform_;

    /**< Calibration data for the wheel odometry. */
    g2o::Vector6d odometry_parameters_;

    std::string frame_;
    std::string most_recent_node_;
};

#endif // SLAM_GRAPH_H
