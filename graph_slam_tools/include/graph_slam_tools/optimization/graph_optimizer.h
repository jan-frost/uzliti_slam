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
