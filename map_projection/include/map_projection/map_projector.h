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

#ifndef MAP_PROJECTOR_H
#define MAP_PROJECTOR_H

#include <graph_slam_common/slam_graph.h>
#include <thread>

/*
 * Abstract base class for map projection in SLAM.
 */

class MapProjector
{
public:
    MapProjector();

    ~MapProjector();

    bool project(SlamGraph &graph);

protected:
    void doProjection(SlamGraph &graph);

    virtual void projectImpl(SlamGraph &graph) = 0;

    std::thread projector_thread_;
    bool thread_running_;
};

#endif
