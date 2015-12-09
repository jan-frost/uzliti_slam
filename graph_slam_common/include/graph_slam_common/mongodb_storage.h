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

#ifndef MONGODBSTORAGE_H
#define MONGODBSTORAGE_H

#include <graph_slam_common/slam_graph_storage.h>
#include <mongo_ros/message_collection.h>
#include <graph_slam_msgs/GraphMeta.h>
#include <graph_slam_msgs/Node.h>
#include <graph_slam_msgs/Edge.h>

namespace mr=mongo_ros;

typedef mr::MessageWithMetadata<graph_slam_msgs::Node> NodeWithMetadata;
typedef NodeWithMetadata::ConstPtr NodeMetaPtr;
typedef mr::MessageWithMetadata<graph_slam_msgs::Edge> EdgeWithMetadata;
typedef EdgeWithMetadata::ConstPtr EdgeMetaPtr;
typedef mr::MessageWithMetadata<graph_slam_msgs::SensorTransform> SensorWithMetadata;
typedef SensorWithMetadata::ConstPtr SensorMetaPtr;

class MongodbStorage : public SlamStorage
{
public:
    MongodbStorage(ros::NodeHandle nh);
    ~MongodbStorage();

    void clear();

    void storeNode(const SlamNode &node);
    void storeEdge(const SlamEdge &edge);
    void storeMetaData(const SlamGraph &graph);

    void removeNode(std::string id);
    void removeEdge(std::string id);

    void loadGraph(SlamGraph &graph);

protected:
    static mr::Metadata makeMetadata(const graph_slam_msgs::Node& n);
    static mr::Metadata makeMetadata(const graph_slam_msgs::Edge& e);
    static mr::Metadata makeMetadata(const graph_slam_msgs::SensorTransform& s);
    static mr::Metadata makeMetadata(const graph_slam_msgs::GraphMeta& g);

    boost::shared_ptr<mongo::DBClientConnection> conn_;
    boost::shared_ptr<mr::MessageCollection<graph_slam_msgs::Node> > node_collection_;
    boost::shared_ptr<mr::MessageCollection<graph_slam_msgs::Edge> > edge_collection_;
    boost::shared_ptr<mr::MessageCollection<graph_slam_msgs::GraphMeta> > meta_collection_;
};

#endif // MONGODBSTORAGE_H
