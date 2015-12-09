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

#ifndef GRAPH_OPTIMIZER
#define GRAPH_OPTIMIZER

#include <graph_slam_common/slam_graph.h>
#include <graph_optimization/GraphOptimizerConfig.h>
#include <thread>
#include <chrono>

class GraphOptimizer
{
public:
    GraphOptimizer();

    ~GraphOptimizer();

    bool optimize(SlamGraph &graph, boost::function<void ()> callback);

    void storeOptimizationResults(SlamGraph &graph);

    void setConfig(graph_optimization::GraphOptimizerConfig config);

protected:    
    void graphOptimizationThread();

    virtual void addGraphImpl(SlamGraph &graph) = 0;

    virtual void storeImpl(SlamGraph &graph) = 0;

    virtual void optimizeImpl() = 0;

    bool running;
    std::thread graph_optimization_thread_;
    std::mutex opt_mutex_;
    bool do_optimization_;
    boost::function<void ()> callback_;
    graph_optimization::GraphOptimizerConfig config_;
};

#endif
