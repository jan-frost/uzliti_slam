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

#include <transformation_estimation/transformation_estimator.h>

TransformationEstimator::TransformationEstimator(boost::function<void (SlamEdge)> callback):
    callback_(callback)
{
    running_ = true;
    estimation_thread_ = std::thread(&TransformationEstimator::estimationThread, this);
}

TransformationEstimator::~TransformationEstimator() {
    running_ = false;
    estimation_thread_.join();
}


void TransformationEstimator::estimateEdge(SlamNode &from, SlamNode &to) {
    estimation_mutex_.lock();
    auto est_pair = std::make_pair(from.id_, to.id_);
//    if (est_tried_.find(est_pair) == est_tried_.end()) {
        est_queue_.push_back(std::make_pair(from, to));
        est_tried_.insert(est_pair);
//    }
    estimation_mutex_.unlock();
}

void TransformationEstimator::estimationThread() {
    while (running_) {
        estimation_mutex_.lock();
        if (!est_queue_.empty()) {
            std::pair<SlamNode, SlamNode> potential_neighbors = est_queue_.back();
            est_queue_.pop_back();
            estimation_mutex_.unlock();
            SlamEdge edge;
            if (!estimateEdgeImpl(potential_neighbors.first, potential_neighbors.second, edge)) {
                edge.matching_score_ = 0.;
            }
            callback_(edge);
        } else {
            estimation_mutex_.unlock();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
