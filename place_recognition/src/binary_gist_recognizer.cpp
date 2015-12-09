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

#include <place_recognition/binary_gist_recognizer.h>

BinaryGistRecognizer::BinaryGistRecognizer() :
    PlaceRecognizer(),
    flann_params_(),
    flann_matcher_(flann_params_)
{
    local_place_count_ = 0;
}

BinaryGistRecognizer::~BinaryGistRecognizer() { }

void BinaryGistRecognizer::clearImpl()
{
    multiple_desc_map_.clear();
    flann_matcher_ = flann::Index<FlannDistance>(flann_params_);
    local_place_count_ = 0;
}

std::vector<int> BinaryGistRecognizer::searchAndAddPlaceImpl(std::string id, std::vector<SensorDataPtr> data)
{
    std::vector<int> res;

    // Scan for gist descriptor.
    for (auto d : data) {
        if (d->type_ == graph_slam_msgs::SensorData::SENSOR_TYPE_BINARY_GIST) {
            BinaryGistDataPtr gist_data = boost::dynamic_pointer_cast<BinaryGistData>(d);
            flann::Matrix<FlannElement> flann_mat(gist_data->descriptor_.data, gist_data->descriptor_.rows, gist_data->descriptor_.cols);

            if (flann_matcher_.size() > 0) {
                int nn = config_.k_nearest_neighbors;
                Matrix<int> indices(new int[flann_mat.rows*nn], flann_mat.rows, nn);
                Matrix<FlannDistanceType> dists(new FlannDistanceType[flann_mat.rows*nn], flann_mat.rows, nn);
                int count = flann_matcher_.knnSearch(flann_mat, indices, dists, nn, flann::SearchParams(64));
                for (size_t i = 0; i < flann_mat.rows; i++) {
                    for (int j = 0; j < count; j++) {
                        if (dists[i][j] <= config_.T && place_id_map_.left.find(multiple_desc_map_[indices[i][j]]) != place_id_map_.left.end()) {
                            ROS_DEBUG("GIST match %d (%d)", multiple_desc_map_[indices[i][j]], dists[i][j]);
                            res.push_back(multiple_desc_map_[indices[i][j]]);
                        }
                    }
                }
            }

            ROS_DEBUG("add place for node %s", id.c_str());
            // Add descriptor to matcher.
            if (flann_matcher_.veclen() == 0) {
                flann_matcher_ = flann::Index<FlannDistance>(flann_mat, flann_params_);
            } else {
                flann_matcher_.addPoints(flann_mat);
            }

            descriptors_.push_back(gist_data->descriptor_);

            multiple_desc_map_[local_place_count_] = place_count_;
            local_place_count_++;
        }
    }

    return res;
}

void BinaryGistRecognizer::addPlaceImpl(std::string id, std::vector<SensorDataPtr> data)
{
    // Scan for gist descriptor.
    for (auto d : data) {
        if (d->type_ == graph_slam_msgs::SensorData::SENSOR_TYPE_BINARY_GIST) {
            ROS_DEBUG("add place for node %s", id.c_str());
            BinaryGistDataPtr gist_data = boost::dynamic_pointer_cast<BinaryGistData>(d);

            // Add descriptor to matcher.
            flann::Matrix<FlannElement> flann_mat(gist_data->descriptor_.data, gist_data->descriptor_.rows, gist_data->descriptor_.cols);
            if (flann_matcher_.veclen() == 0) {
                flann_matcher_ = flann::Index<FlannDistance>(flann_mat, flann_params_);
            } else {
                flann_matcher_.addPoints(flann_mat);
            }

            descriptors_.push_back(gist_data->descriptor_);

            multiple_desc_map_[local_place_count_] = place_count_;
            local_place_count_++;
        }
    }
}

std::vector<int> BinaryGistRecognizer::searchImpl(std::string id, std::vector<SensorDataPtr> data)
{
    std::vector<int> res;

    // Scan for gist descriptor.
    for (auto d : data) {
        if (d->type_ == graph_slam_msgs::SensorData::SENSOR_TYPE_BINARY_GIST) {
            BinaryGistDataPtr gist_data = boost::dynamic_pointer_cast<BinaryGistData>(d);

            if (flann_matcher_.size() > 0) {
                int nn = config_.k_nearest_neighbors;
                flann::Matrix<FlannElement> query(gist_data->descriptor_.data, gist_data->descriptor_.rows, gist_data->descriptor_.cols);
                Matrix<int> indices(new int[query.rows*nn], query.rows, nn);
                Matrix<FlannDistanceType> dists(new FlannDistanceType[query.rows*nn], query.rows, nn);
                int count = flann_matcher_.knnSearch(query, indices, dists, nn, flann::SearchParams(64));
                for (size_t i = 0; i < query.rows; i++) {
                    for (int j = 0; j < count; j++) {
                        if (dists[i][j] <= config_.T && place_id_map_.left.find(multiple_desc_map_[indices[i][j]]) != place_id_map_.left.end()) {
                            ROS_DEBUG("GIST match %d (%d)", multiple_desc_map_[indices[i][j]], dists[i][j]);
                            res.push_back(multiple_desc_map_[indices[i][j]]);
                        }
                    }
                }
            }
        }
    }

    return res;
}

void BinaryGistRecognizer::removePlaceImpl(std::string id, std::vector<SensorDataPtr> data)
{
    int flann_idx = place_id_map_.right.at(id);
    flann_matcher_.removePoint(flann_idx);
}
