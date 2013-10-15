#ifndef TRANSFORMATION_ESTIMATION_H
#define TRANSFORMATION_ESTIMATION_H

#include <graph_slam_tools/graph/slam_node.h>
#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>

struct BinaryFeatureComparator
{
    bool operator()(const cv::DMatch &left, const cv::DMatch &right) const
    {
        return left.distance < right.distance;
    }
};

class TransformationEstimator
{
public:
    TransformationEstimator(boost::function<void (SlamEdge)> callback) :
        callback_(callback)
    {
        running_ = true;
        estimation_thread_ = std::thread(&TransformationEstimator::estimationThread, this);
    }

    ~TransformationEstimator() {
        running_ = false;
        estimation_thread_.join();
    }

    void estimateEdge(SlamNode &from, SlamNode &to) {
        estimation_mutex_.lock();
        est_queue_.push_back(std::make_pair(from, to));
        estimation_mutex_.unlock();
    }

protected:
    void estimationThread() {
        while (running_) {
            estimation_mutex_.lock();
            if (!est_queue_.empty()) {
                std::pair<SlamNode, SlamNode> potential_neighbors = est_queue_.back();
                est_queue_.pop_back();
                estimation_mutex_.unlock();
                SlamEdge edge;
                if (estimateEdgeImpl(potential_neighbors.first, potential_neighbors.second, edge)) {
                    callback_(edge);
                }
            } else {
                estimation_mutex_.unlock();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    virtual bool estimateEdgeImpl(SlamNode &from, SlamNode &to, SlamEdge &edge) = 0;

    std::thread estimation_thread_;
    std::mutex estimation_mutex_;
    bool running_;
    std::vector<std::pair<SlamNode, SlamNode> > est_queue_;
    boost::function<void (SlamEdge)> callback_;
};

#endif
