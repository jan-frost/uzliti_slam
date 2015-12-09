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

#include <map_projection/map_projector.h>

MapProjector::MapProjector()
{
    thread_running_ = false;
}

MapProjector::~MapProjector()
{

}

bool MapProjector::project(SlamGraph &graph)
{
    if (thread_running_) {
        return false;
    }
    thread_running_ = true;
    projector_thread_ = std::thread(std::bind(&MapProjector::doProjection, this, graph));
    projector_thread_.detach();
    return true;
}

void MapProjector::doProjection(SlamGraph &graph)
{
    projectImpl(graph);
    thread_running_ = false;
}
