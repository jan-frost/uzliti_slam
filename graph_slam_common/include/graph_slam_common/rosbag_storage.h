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

#ifndef ROSBAG_STORAGE_H
#define ROSBAG_STORAGE_H

#include <graph_slam_common/slam_graph_storage.h>
#include <graph_slam_msgs/GraphMeta.h>
#include <graph_slam_msgs/Node.h>
#include <graph_slam_msgs/Edge.h>
#include <mutex>
#include <thread>

class RosbagStorage : public SlamStorage
{
public:
    RosbagStorage(ros::NodeHandle nh, std::string storage_path, bool clear_storage = false, bool buffered = false);
    ~RosbagStorage();

    void clear();

    void storeNode(const SlamNode &node);
    void storeEdge(const SlamEdge &edge);
    void storeMetaData(const SlamGraph &graph);

    void removeNode(std::string id);
    void removeEdge(std::string id);

    void loadGraph(SlamGraph &graph);

protected:
    void initialize(std::string storage_path, bool clear_storage = false);
    void bufferThread();

    bool buffered_;
    std::mutex buffer_mutex_;
    std::mutex rosbag_mutex_;

    std::vector<graph_slam_msgs::Node> node_buffer_;
    std::vector<graph_slam_msgs::Edge> edge_buffer_;
    std::vector<graph_slam_msgs::GraphMeta> meta_buffer_;

    std::thread buffer_thread_;
    bool running_;
};

#endif // ROSBAG_STORAGE_H
