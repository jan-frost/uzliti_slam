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

#ifndef OCCUPANCY_GRID_PROJECTOR_H
#define OCCUPANCY_GRID_PROJECTOR_H

#include <graph_slam_tools/projection/map_projector.h>

#include <dynamic_reconfigure/server.h>
#include <graph_slam_tools/OccupancyGridProjectorConfig.h>
#include <graph_slam_tools/projection/graph_grid_mapper.h>
#include <ros/ros.h>

class OccupancyGridProjector : public MapProjector
{
public:
    OccupancyGridProjector(ros::NodeHandle nh);
    ~OccupancyGridProjector();
    void setConfig(graph_slam_tools::OccupancyGridProjectorConfig config);

protected:
    void projectImpl(SlamGraph &graph);

    ros::Publisher map_pub_;    /**< Publisher for the occupancy grid map. */
    GraphGridMapper mapper;     /**< An auxiliary object to handle the generation of an occupancy grid map */
    graph_slam_tools::OccupancyGridProjectorConfig config_;
};

#endif
