#ifndef TRANSFORMATION_ESTIMATION_H
#define TRANSFORMATION_ESTIMATION_H

#include <graph_slam_tools/graph/slam_node.h>
#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>
#include <unordered_set>

template <typename T, typename U>
struct std::hash<std::pair<T, U> > {
public:
    size_t operator()(std::pair<T, U> x) {
        return std::hash<T>()(x.first) ^ std::hash<U>()(x.second);
    }
};

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
    TransformationEstimator(boost::function<void (SlamEdge)> callback);

    ~TransformationEstimator();

    void estimateEdge(SlamNode &from, SlamNode &to);

    virtual bool estimateEdgeImpl(SlamNode &from, SlamNode &to, SlamEdge &edge) = 0;

protected:
    void estimationThread();

    std::thread estimation_thread_;
    std::mutex estimation_mutex_;
    bool running_;
    std::vector<std::pair<SlamNode, SlamNode> > est_queue_;
    std::unordered_set<std::pair<std::string, std::string> > est_tried_;
    boost::function<void (SlamEdge)> callback_;
};

#endif

