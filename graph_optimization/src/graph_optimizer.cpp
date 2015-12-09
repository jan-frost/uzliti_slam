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

#include <graph_optimization/graph_optimizer.h>

GraphOptimizer::GraphOptimizer()
{
    graph_optimization_thread_ = std::thread(&GraphOptimizer::graphOptimizationThread, this);
    do_optimization_ = false;
    running = true;
}

GraphOptimizer::~GraphOptimizer()
{
    running = false;
    graph_optimization_thread_.join();
}

bool GraphOptimizer::optimize(SlamGraph &graph, boost::function<void ()> callback)
{
    std::lock_guard<std::mutex> lock(opt_mutex_);

    if (do_optimization_) {
        return false;   // Optimization is already running.
    }

    callback_ = callback;
    addGraphImpl(graph);
    do_optimization_ = true;
    return true;
}

void GraphOptimizer::storeOptimizationResults(SlamGraph &graph)
{
    storeImpl(graph);
}

void GraphOptimizer::setConfig(graph_optimization::GraphOptimizerConfig config)
{
    config_ = config;
}

void GraphOptimizer::graphOptimizationThread()
{
    while (running) {
        opt_mutex_.lock();
        if (do_optimization_) {
            opt_mutex_.unlock();
            optimizeImpl();
            callback_();
            opt_mutex_.lock();
            do_optimization_ = false;
        }
        opt_mutex_.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}
