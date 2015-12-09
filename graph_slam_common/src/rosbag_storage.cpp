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

#include <graph_slam_common/rosbag_storage.h>

#include <graph_slam_common/slam_graph.h>
#include <graph_slam_common/conversions.h>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <rosbag/bag.h>
#include <rosbag/query.h>
#include <rosbag/view.h>
#include <rosbag/message_instance.h>
#include <boost/foreach.hpp>

#define foreach BOOST_FOREACH

namespace fs = boost::filesystem;

RosbagStorage::RosbagStorage(ros::NodeHandle nh, std::string storage_path, bool clear_storage, bool buffered) :
    SlamStorage(nh, storage_path)
{
    buffered_ = buffered;
    running_ = true;
    initialize(storage_path_, clear_storage);
    buffer_thread_ = std::thread(&RosbagStorage::bufferThread, this);
}

RosbagStorage::~RosbagStorage()
{
    std::lock_guard<std::mutex> lock(rosbag_mutex_);
    running_ = false;
    buffer_thread_.join();
}

void RosbagStorage::clear()
{
    std::lock_guard<std::mutex> lock(buffer_mutex_);

    initialize(storage_path_, true);
    node_buffer_.clear();
    edge_buffer_.clear();
    meta_buffer_.clear();
}

void RosbagStorage::storeNode(const SlamNode &node)
{
    std::lock_guard<std::mutex> lock(rosbag_mutex_);

    graph_slam_msgs::Node node_msg = Conversions::toMsg(node);

    rosbag::Bag bag;
    fs::path storage_file(storage_path_);
    storage_file /= "nodes";
    storage_file /= node.id_;
    bag.open(fs::complete(storage_file).string(), rosbag::bagmode::Write);
    bag.write("node", ros::Time::now() + ros::Duration(0, 1), node_msg);
    bag.close();
}

void RosbagStorage::storeEdge(const SlamEdge &edge)
{
    std::lock_guard<std::mutex> lock(rosbag_mutex_);

    graph_slam_msgs::Edge node_msg = Conversions::toMsg(edge);

    rosbag::Bag bag;
    fs::path storage_file(storage_path_);
    storage_file /= "edges";
    storage_file /= edge.id_;
    bag.open(fs::complete(storage_file).string(), rosbag::bagmode::Write);
    bag.write("edge", ros::Time::now() + ros::Duration(0, 1), node_msg);
    bag.close();
}

void RosbagStorage::storeMetaData(const SlamGraph &graph)
{
    std::lock_guard<std::mutex> lock(rosbag_mutex_);

    graph_slam_msgs::GraphMeta meta_msg = graph.toMetaData(tfl_);

    rosbag::Bag bag;
    fs::path storage_file(storage_path_);
    storage_file /= "meta";
    storage_file /= "meta";
    bag.open(fs::complete(storage_file).string(), rosbag::bagmode::Write);
    bag.write("meta", ros::Time::now() + ros::Duration(0, 1), meta_msg);
    bag.close();
}

void RosbagStorage::removeNode(std::string id)
{
    std::lock_guard<std::mutex> lock(rosbag_mutex_);

    fs::path storage_file(storage_path_);
    storage_file /= "nodes";
    storage_file /= id;

    if (fs::exists(storage_file)) {
        ROS_DEBUG_NAMED("rosbag_storage", "removing node %s", id.c_str());
        fs::remove(storage_file);
    }
}

void RosbagStorage::removeEdge(std::string id)
{
    std::lock_guard<std::mutex> lock(rosbag_mutex_);

    fs::path storage_file(storage_path_);
    storage_file /= "edges";
    storage_file /= id;

    if (fs::exists(storage_file)) {
        ROS_DEBUG_NAMED("rosbag_storage", "removing edge %s", id.c_str());
        fs::remove(storage_file);
    }
}

void RosbagStorage::loadGraph(SlamGraph &graph)
{
    fs::path storage_path(storage_path_);

    ROS_INFO_NAMED("rosbag_storage", "loading nodes");
    auto node_storage = storage_path / "nodes";
    for (boost::filesystem::directory_iterator it(node_storage); it != boost::filesystem::directory_iterator(); ++it) {
        rosbag::Bag bag;
        bag.open(it->path().string(), rosbag::bagmode::Read);
        try {
            std::vector<std::string> topics;
            topics.push_back(std::string("node"));
            rosbag::View view(bag, rosbag::TopicQuery(topics));

            foreach (rosbag::MessageInstance const m, view) {
                graph_slam_msgs::Node::ConstPtr node_msg = m.instantiate<graph_slam_msgs::Node>();
                if (node_msg != NULL) {
                    ROS_INFO_NAMED("rosbag_storage", "loading node %s", node_msg->id.c_str());
                    graph.addNode(Conversions::fromMsg(*node_msg));
                    break;
                }
            }
        } catch (rosbag::BagUnindexedException bue) {
            ROS_ERROR("node bag unindexed: %s", it->path().string().c_str());
        }
        bag.close();
    }

    ROS_INFO_NAMED("rosbag_storage", "loading edges");
    auto edge_storage = storage_path / "edges";
    for (boost::filesystem::directory_iterator it(edge_storage); it != boost::filesystem::directory_iterator(); ++it) {
        rosbag::Bag bag;
        bag.open(it->path().string(), rosbag::bagmode::Read);
        try {
            std::vector<std::string> topics;
            topics.push_back(std::string("edge"));
            rosbag::View view(bag, rosbag::TopicQuery(topics));

            foreach (rosbag::MessageInstance const m, view) {
                graph_slam_msgs::Edge::ConstPtr edge_msg = m.instantiate<graph_slam_msgs::Edge>();
                if (edge_msg != NULL) {
                    ROS_INFO_NAMED("rosbag_storage", "loading edge %s", edge_msg->id.c_str());
                    graph.addEdge(Conversions::fromMsg(*edge_msg));
                }
            }
        } catch (rosbag::BagUnindexedException bue) {
            ROS_ERROR("edge bag unindexed: %s", it->path().string().c_str());
        }

        bag.close();
    }

    ROS_INFO_NAMED("rosbag_storage", "loading meta");
    auto meta_storage = storage_path / "meta";
    for (boost::filesystem::directory_iterator it(meta_storage); it != boost::filesystem::directory_iterator(); ++it) {
        rosbag::Bag bag;
        bag.open(it->path().string(), rosbag::bagmode::Read);
        try {

            std::vector<std::string> topics;
            topics.push_back(std::string("meta"));
            rosbag::View view(bag, rosbag::TopicQuery(topics));

            foreach (rosbag::MessageInstance const m, view) {
                graph_slam_msgs::GraphMeta::ConstPtr meta_msg = m.instantiate<graph_slam_msgs::GraphMeta>();
                if (meta_msg != NULL) {
                    ROS_INFO_NAMED("rosbag_storage", "loading meta data");
                    graph.updateMetaData(*meta_msg);
                }
            }
        } catch (rosbag::BagUnindexedException bue) {
            ROS_ERROR("node bag unindexed: %s", it->path().string().c_str());
        }

        bag.close();
    }
}

void RosbagStorage::initialize(std::string storage_path, bool clear_storage)
{
    fs::path storage_dir(storage_path);
    ROS_INFO_NAMED("rosbag_storage", "initializing storage to path: %s", storage_dir.string().c_str());

    if (clear_storage && fs::is_directory(storage_dir)) {
        ROS_INFO_NAMED("rosbag_storage", "clearing data storage");
        fs::remove_all(storage_dir);
    }

    // Check base folder.
    auto base_folder = storage_dir.branch_path();
    if (!fs::exists(base_folder)) {
        fs::create_directory(base_folder);
    }

    // Check SLAM folders.
    if (!fs::exists(storage_dir)) {
        fs::create_directory(storage_dir);
        fs::create_directory(storage_dir / "nodes");
        fs::create_directory(storage_dir / "edges");
        fs::create_directory(storage_dir / "meta");
    }
}

void RosbagStorage::bufferThread()
{

}
