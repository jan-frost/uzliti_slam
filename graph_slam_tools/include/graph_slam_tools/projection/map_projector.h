#ifndef MAP_PROJECTOR_H
#define MAP_PROJECTOR_H

#include <graph_slam_tools/graph/slam_graph.h>
#include <thread>

/*
 * Abstract base class for place recognition in SLAM.
 */

class MapProjector
{
public:
    MapProjector() {
        thread_running_ = false;
    }

    ~MapProjector() { }

    bool project(SlamGraph &graph) {
        if (thread_running_) {
            return false;
        }
        thread_running_ = true;
        projector_thread_ = std::thread(std::bind(&MapProjector::doProjection, this, graph));
        projector_thread_.detach();
        return true;
    }

protected:
    void doProjection(SlamGraph &graph) {
        projectImpl(graph);
        thread_running_ = false;
    }

    virtual void projectImpl(SlamGraph &graph) = 0;

    std::thread projector_thread_;
    bool thread_running_;
};

#endif
