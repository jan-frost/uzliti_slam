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

#ifndef GRAPH_OPTIMIZER
#define GRAPH_OPTIMIZER

#include <graph_slam_tools/graph/slam_graph.h>
#include <graph_slam_tools/GraphOptimizerConfig.h>
#include <thread>
#include <chrono>

class GraphOptimizer
{
public:
    GraphOptimizer() {
        graph_optimization_thread_ = std::thread(&GraphOptimizer::graphOptimizationThread, this);
        do_optimization_ = false;
    }

    ~GraphOptimizer() { }

    bool optimize(SlamGraph &graph, boost::function<void ()> callback) {
        opt_mutex_.lock();
        if (do_optimization_) {
            opt_mutex_.unlock();
            return false;   // Optimization is already running.
        }

        callback_ = callback;
        addGraphImpl(graph);
        do_optimization_ = true;
        opt_mutex_.unlock();
        return true;
    }

    void storeOptimizationResults(SlamGraph &graph) {
        storeImpl(graph);
    }

    void setConfig(graph_slam_tools::GraphOptimizerConfig config) {
        config_ = config;
    }

protected:    
    void graphOptimizationThread() {
        while (true) {
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

    virtual void addGraphImpl(SlamGraph &graph) = 0;

    virtual void storeImpl(SlamGraph &graph) = 0;

    virtual void optimizeImpl() = 0;

    std::thread graph_optimization_thread_;
    std::mutex opt_mutex_;
    bool do_optimization_;
    boost::function<void ()> callback_;
    graph_slam_tools::GraphOptimizerConfig config_;
};

#endif
