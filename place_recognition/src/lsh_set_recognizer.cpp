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

#include <place_recognition/lsh_set_recognizer.h>

using std::chrono::nanoseconds;

struct CorrespondenceGreater
{
    bool operator()(const std::pair<int,float> &left, const std::pair<int,float> &right) const
    {
        return left.second > right.second;
    }
};

LshSetRecognizer::LshSetRecognizer() : PlaceRecognizer(),
    fast_matcher_(4)
{
    init_ = false;
}

LshSetRecognizer::~LshSetRecognizer() { }

void LshSetRecognizer::clearImpl()
{
    fast_matcher_.clear();
    init_ = false;
}

std::vector<int> LshSetRecognizer::searchAndAddPlaceImpl(std::string id, std::vector<SensorDataPtr> data)
{
    std::vector<int> res;

    if (!init_) {
        ROS_INFO("init lsh");
        fast_matcher_.clear();
        init_ = true;
    }

    for (auto d : data) {
        if (d->type_ == graph_slam_msgs::SensorData::SENSOR_TYPE_FEATURE) {
            FeatureDataPtr feature_data = boost::dynamic_pointer_cast<FeatureData>(d);
            cv::Mat descriptors = feature_data->features_;

            std::vector<int> all_matches(place_count_ + 1, 0);

            // Search most likely neighbors.
            if (descriptors.rows > 150) {
                fast_matcher_.matchAndAdd(descriptors, place_count_, all_matches);
            } else {
                fast_matcher_.match(descriptors, all_matches);
            }

            // Get all matches that have more than T similarity.
            std::vector<std::pair<int, float> > matches;
            for (int i = 0; i < (int)all_matches.size(); i++) {
                if (all_matches[i] > 0) {
                    auto similarity = (float)all_matches[i] / (float) fast_matcher_.getNumTables();
                    if (similarity >= config_.T) {
                        matches.push_back(std::make_pair(i, similarity));
                    }
                }
            }
            ROS_DEBUG_NAMED("place recognition", "%d matches", (int) matches.size());

            if (matches.size() > 0) {
                // Sort matches.
                res.reserve(matches.size());
                std::sort(matches.begin(), matches.end(), CorrespondenceGreater());
                for (auto match : matches) {
                    res.push_back(match.first);
                }
            }
        }
    }

    return res;
}

void LshSetRecognizer::addPlaceImpl(std::string id, std::vector<SensorDataPtr> data)
{
    // Scan for features.
    for (auto d : data) {
        if (d->type_ == graph_slam_msgs::SensorData::SENSOR_TYPE_FEATURE) {
            ROS_DEBUG_NAMED("place recognition", "add place for node %s", id.c_str());
            FeatureDataPtr feature_data = boost::dynamic_pointer_cast<FeatureData>(d);
            cv::Mat descriptors = feature_data->features_;

            if (!init_) {
                ROS_INFO("init lsh");
                fast_matcher_.clear();
                init_ = true;
            }

            // Add node.
            if (descriptors.rows > 150) {
                ROS_DEBUG_NAMED("place recognition", "add %d descriptors", descriptors.rows);
                fast_matcher_.add(descriptors, place_count_);
            }
        }
    }
}

std::vector<int> LshSetRecognizer::searchImpl(std::string id, std::vector<SensorDataPtr> data)
{
    std::vector<int> res;

    if (init_) {
        // Scan for features.
        for (auto d : data) {
            if (d->type_ == graph_slam_msgs::SensorData::SENSOR_TYPE_FEATURE) {
                FeatureDataPtr feature_data = boost::dynamic_pointer_cast<FeatureData>(d);
                cv::Mat descriptors = feature_data->features_;

                // Search most likely neighbors.
                std::vector<int> all_matches(place_count_, 0);
                fast_matcher_.match(descriptors, all_matches);

                // Get all matches that have more than T similarity.
                std::vector<std::pair<int, float> > matches;
                for (int i = 0; i < (int)all_matches.size(); i++) {
                    if (all_matches[i] > 0) {
                        auto similarity = (float)all_matches[i] / (float) fast_matcher_.getNumTables();
                        if (similarity >= config_.T) {
                            matches.push_back(std::make_pair(i, similarity));
                        }
                    }
                }
                ROS_DEBUG_NAMED("place recognition", "%d matches", (int) matches.size());

                // Sort matches.
                if (matches.size() > 0) {
                    res.reserve(matches.size());
                    std::sort(matches.begin(), matches.end(), CorrespondenceGreater());
                    for (auto match : matches) {
                        res.push_back(match.first);
                    }
                }
            }
        }
    }

    return res;
}

void LshSetRecognizer::removePlaceImpl(std::string id, std::vector<SensorDataPtr> data)
{
    if (!init_) {
        return;
    }

    // Remove from LSH matcher.
    for (auto d : data) {
        if (d->type_ == graph_slam_msgs::SensorData::SENSOR_TYPE_FEATURE) {
            FeatureDataPtr feature_data = boost::dynamic_pointer_cast<FeatureData>(d);
            cv::Mat descriptors = feature_data->features_;

            // Remove node.
            fast_matcher_.remove(descriptors, place_id_map_.right.at(id));
        }
    }
}


FastLshTable::FastLshTable(int start_byte, int key_width)
{
    start_byte_ = start_byte;
    key_width_ = key_width;
//    table_.set_empty_key(-1);
}

void FastLshTable::add(const uchar *descriptor, int id)
{
    long_long_array_u index;
    index.ll = 0;
    for (int i = 0; i < key_width_; i++) {
        index.b[i] = descriptor[start_byte_ + i];
    }
    table_[index.ll].push_back(id);
}

void FastLshTable::match(const uchar *descriptor, std::vector<int> &matches)
{
    long_long_array_u index;
    index.ll = 0;
    for (int i = 0; i < key_width_; i++) {
        index.b[i] = descriptor[start_byte_ + i];
    }
    if (table_.find(index.ll) != table_.end()) {
        for (auto ind : table_[index.ll]) {
            matches[ind]++;
        }
    }
}

void FastLshTable::matchAndAdd(const uchar *descriptor, int id, std::vector<int> &matches)
{
    long_long_array_u index;
    index.ll = 0;
    for (int i = 0; i < key_width_; i++) {
        index.b[i] = descriptor[start_byte_ + i];
    }
    std::bitset<64> bits(index.ll);
    if (bits.count() > 3 * key_width_) {

    auto &bucket = table_[index.ll];
    for (auto ind : bucket) {
        matches[ind]++;
    }
    bucket.push_back(id);

    }
}

void FastLshTable::remove(const uchar *descriptor, int id)
{
    long_long_array_u index;
    index.ll = 0;
    for (int i = 0; i < key_width_; i++) {
        index.b[i] = descriptor[start_byte_ + i];
    }
    if (table_.find(index.ll) != table_.end()) {
        table_[index.ll].erase(std::remove(table_[index.ll].begin(), table_[index.ll].end(), id), table_[index.ll].end());
        if (table_[index.ll].size() == 0) {
            table_.erase(index.ll);
        }
    }
}


FastLshSet::FastLshSet(int key_width)
{
    key_width_ = key_width;
    clear();
}

void FastLshSet::clear()
{
    tables_.clear();
    for (int i = 0; i < 32 - key_width_ + 1; i += key_width_) {
        tables_.push_back(boost::shared_ptr<FastLshTable>(new FastLshTable(i, key_width_)));
    }
}

int FastLshSet::getNumTables()
{
    return (int) tables_.size();
}

void FastLshSet::add(cv::Mat &descriptors, int id)
{
    for (int i = 0; i < descriptors.rows; i++) {
        const uchar* descriptor = descriptors.ptr<uchar>(i);
        for (auto tab : tables_) {
            tab->add(descriptor, id);
        }
    }
}

void FastLshSet::match(cv::Mat &descriptors, std::vector<int> &matches)
{
    for (int i = 0; i < descriptors.rows; i++) {
        const uchar* descriptor = descriptors.ptr<uchar>(i);
        for (auto tab : tables_) {
            tab->match(descriptor, matches);
        }
    }
}

void FastLshSet::matchAndAdd(cv::Mat &descriptors, int id, std::vector<int> &matches)
{
    for (int i = 0; i < descriptors.rows; i++) {
        const uchar* descriptor = descriptors.ptr<uchar>(i);
        int j = 0;
        for (auto tab : tables_) {
            tab->matchAndAdd(descriptor, id, matches);
        }
    }
}

void FastLshSet::remove(cv::Mat &descriptors, int id)
{
    for (int i = 0; i < descriptors.rows; i++) {
        const uchar* descriptor = descriptors.ptr<uchar>(i);
        for (auto tab : tables_) {
            tab->remove(descriptor, id);
        }
    }
}





