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

#ifndef SLAM_GRAPH_STORAGE_H
#define SLAM_GRAPH_STORAGE_H

#include <ros/ros.h>
#include <tf/transform_listener.h>

class SlamGraph;
class SlamNode;
class SlamEdge;

class SlamStorage
{
public:
    SlamStorage(ros::NodeHandle nh, std::string storage_path);
    ~SlamStorage();

    virtual void clear() = 0;

    virtual void storeNode(const SlamNode &node) = 0;
    virtual void storeEdge(const SlamEdge &edge) = 0;
    virtual void storeMetaData(const SlamGraph &graph) = 0;

    virtual void removeNode(std::string id) = 0;
    virtual void removeEdge(std::string id) = 0;

    virtual void loadGraph(SlamGraph &graph) = 0;

protected:
    std::string storage_path_;
    tf::TransformListener tfl_;
};

#endif // SLAM_GRAPH_STORAGE_H
