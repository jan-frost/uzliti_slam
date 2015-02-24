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

#include <graph_slam_tools/pr/binary_gist_recognizer.h>

BinaryGistRecognizer::BinaryGistRecognizer() : PlaceRecognizer()
{
    clearImpl();
}

BinaryGistRecognizer::~BinaryGistRecognizer()
{
}

void BinaryGistRecognizer::clearImpl()
{
    matcher_.reset(new lsh::LshMatcher());
    matcher_->setDimensions(20, 8, 2);
    place_count_ = 0;
    place_id_map_.clear();
}

void BinaryGistRecognizer::addPlaceImpl(std::string id, std::vector<SensorDataPtr> data)
{
    // Scan for gist descriptor.
    for (auto d : data) {
        if (d->type_ == graph_slam_msgs::SensorData::SENSOR_TYPE_BINARY_GIST) {
            ROS_DEBUG("add place for node %s", id.c_str());
            BinaryGistDataPtr gist_data = boost::dynamic_pointer_cast<BinaryGistData>(d);

            // Add descriptor to matcher.
            std::vector<cv::Mat> desc_vec;
            desc_vec.push_back(gist_data->descriptor_);
            matcher_->add(desc_vec);

            // Update index to id map.
            place_id_map_[place_count_] = id;
            place_count_++;
        }
    }
}

std::vector<std::string> BinaryGistRecognizer::searchImpl(std::string id, std::vector<SensorDataPtr> data)
{
    std::vector<std::string> res;

    // Scan for gist descriptor.
    for (auto d : data) {
        if (d->type_ == graph_slam_msgs::SensorData::SENSOR_TYPE_BINARY_GIST) {
            BinaryGistDataPtr gist_data = boost::dynamic_pointer_cast<BinaryGistData>(d);

            // Search k nearest neighbors.
            std::vector<std::vector<cv::DMatch> > matches;
            matcher_->knnMatch(gist_data->descriptor_, matches, config_.k_nearest_neighbors);

            ROS_DEBUG("binary gist pr matches");
            // Retrieve string ids.
            if (matches.size() > 0) {
                for (auto match : matches[0]) {
                    ROS_DEBUG("  %f - %d, %d", match.distance, match.trainIdx, match.imgIdx);
                    res.push_back(place_id_map_[match.imgIdx]);
                }
            }
        }
    }

    return res;
}
