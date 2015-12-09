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

#include <graph_slam_tools/transformation/transformation_estimator.h>

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
