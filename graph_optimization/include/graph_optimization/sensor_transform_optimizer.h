// Copyright (c) 2014, Institute of Computer Engineering (ITI), Universität zu Lübeck
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

#ifndef SENSOR_TRANSFORM_OPTIMIZER_H
#define SENSOR_TRANSFORM_OPTIMIZER_H

#include <graph_optimization/g2o_optimizer.h>
#include <graph_optimization/edge_se3_odom_diffrential_calib.h>

class SensorTransformOptimizer : public G2oOptimizer
{
public:
    SensorTransformOptimizer();
    ~SensorTransformOptimizer();

protected:
    void clear();
    void addGraphImpl(SlamGraph &graph);
    void storeImpl(SlamGraph &graph);
    void optimizeImpl();

    void addVertex(const SlamNode &node);
    void addOdometryEdge(std::string id, SlamGraph &graph);

    g2o_vertex_bimap sensor_id_to_g2o_map_trans_;
    g2o_vertex_bimap sensor_id_to_g2o_map_rot_;
    g2o::VertexOdomParams *odom_params_vertex_;
};

#endif
