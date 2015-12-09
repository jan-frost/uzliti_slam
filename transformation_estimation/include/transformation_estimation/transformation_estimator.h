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

#ifndef TRANSFORMATION_ESTIMATION_H
#define TRANSFORMATION_ESTIMATION_H

#include <graph_slam_common/slam_node.h>
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

    std::map<std::string, Eigen::Isometry3d, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string, Eigen::Isometry3d> > > sensor_transforms_;

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

