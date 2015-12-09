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

#include <graph_slam_common/mongodb_storage.h>

#include <graph_slam_common/slam_graph.h>
#include <graph_slam_common/conversions.h>

MongodbStorage::MongodbStorage(ros::NodeHandle nh) :
    SlamStorage(nh, "")
{    
    conn_ = mr::makeDbConnection(nh, "localhost", 27019, 60.0);
    node_collection_.reset(new mr::MessageCollection<graph_slam_msgs::Node>("global_slam", "nodes", "localhost", 27019, 60.0));
    edge_collection_.reset(new mr::MessageCollection<graph_slam_msgs::Edge>("global_slam", "edges", "localhost", 27019, 60.0));
    meta_collection_.reset(new mr::MessageCollection<graph_slam_msgs::GraphMeta>("global_slam", "meta", "localhost", 27019, 60.0));
}

MongodbStorage::~MongodbStorage()
{

}

void MongodbStorage::clear()
{
    meta_collection_->removeMessages(mr::Query());
    node_collection_->removeMessages(mr::Query());
    edge_collection_->removeMessages(mr::Query());
}

void MongodbStorage::storeNode(const SlamNode &node)
{
    // Delete node, if it exists.
    node_collection_->removeMessages(QUERY("id" << node.id_));

    // Add node.
    graph_slam_msgs::Node node_msg = Conversions::toMsg(node);
    node_collection_->insert(node_msg, makeMetadata(node_msg));
}

void MongodbStorage::storeEdge(const SlamEdge &edge)
{
    // Delete edge, if it exists.
    edge_collection_->removeMessages(QUERY("id" << edge.id_));

    // Add edge to database.
    graph_slam_msgs::Edge edge_msg = Conversions::toMsg(edge);
    edge_collection_->insert(edge_msg, makeMetadata(edge_msg));
}

void MongodbStorage::storeMetaData(const SlamGraph &graph)
{
    meta_collection_->removeMessages(mr::Query());
    graph_slam_msgs::GraphMeta meta = graph.toMetaData(tfl_);
    meta_collection_->insert(meta, makeMetadata(meta));
}

void MongodbStorage::removeNode(std::string id)
{
    // Delete node, if it exists.
    node_collection_->removeMessages(QUERY("id" << id));
}

void MongodbStorage::removeEdge(std::string id)
{
    // Delete edge, if it exists.
    edge_collection_->removeMessages(QUERY("id" << id));
}

void MongodbStorage::loadGraph(SlamGraph &graph)
{
    graph.clear();
    mongo::Query query;

    // Get meta data.
    try {
        graph_slam_msgs::GraphMeta meta_msg = *meta_collection_->findOne(mr::Query());
        graph.updateMetaData(meta_msg);
    } catch (mr::NoMatchingMessageException e) {
        ROS_ERROR(e.what());
    }

    // Get all stored nodes.
    std::vector<NodeMetaPtr> stored_nodes = node_collection_->pullAllResults(query, true);
    for (auto node_msg_with_meta : stored_nodes) {
        graph_slam_msgs::Node node_msg = *node_collection_->findOne(mr::Query("id", node_msg_with_meta->lookupString("id")));
        ROS_INFO_NAMED("mongodb_storage", "add node %s", node_msg.id.c_str());

        SlamNode node = Conversions::fromMsg(node_msg);
        graph.addNode(node);
    }

    // Get all stored edges.
    std::vector<EdgeMetaPtr> stored_edges = edge_collection_->pullAllResults(query);
    for (auto edge_msg_with_meta : stored_edges) {
        graph_slam_msgs::Edge edge_msg = *edge_msg_with_meta;
        ROS_INFO_NAMED("mongodb_storage", "add edge %s", edge_msg.id.c_str());

        SlamEdge edge = Conversions::fromMsg(edge_msg);
        graph.addEdge(edge);
    }
}

mongo_ros::Metadata MongodbStorage::makeMetadata(const graph_slam_msgs::Node &n)
{
    return mr::Metadata("id", n.id, "x", n.pose.position.x, "y", n.pose.position.y, "z", n.pose.position.z);
}

mr::Metadata MongodbStorage::makeMetadata(const graph_slam_msgs::Edge& e)
{
    return mr::Metadata("id", e.id, "id_from", e.id_from, "id_to", e.id_to);
}

mongo_ros::Metadata MongodbStorage::makeMetadata(const graph_slam_msgs::SensorTransform &s)
{
    return mr::Metadata("sensor_name", s.sensor_name);
}

mongo_ros::Metadata MongodbStorage::makeMetadata(const graph_slam_msgs::GraphMeta &g)
{
    return mr::Metadata("name", g.name);
}
