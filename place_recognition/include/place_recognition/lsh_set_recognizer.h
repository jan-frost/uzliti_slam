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

#ifndef LSH_SET_RECOGNIZER_H
#define LSH_SET_RECOGNIZER_H

#include <place_recognition/place_recognizer.h>

#include <chrono>
#include <unordered_map>

using std::chrono::duration_cast;
using std::chrono::microseconds;
using std::chrono::system_clock;

union long_long_array_u
{
  uint8_t b[8];
  uint64_t ll;
};

class FastLshTable
{
public:
    FastLshTable (int start_byte, int key_width);

    void add(const uchar *descriptor, int id);

    void match(const uchar *descriptor, std::vector<int> &matches);

    void matchAndAdd(const uchar *descriptor, int id, std::vector<int> &matches);

    void remove(const uchar *descriptor, int id);

    std::unordered_map<u_int64_t, std::vector<int> > table_;
    int start_byte_;
    int key_width_;
};

class FastLshSet
{
public:
    FastLshSet(int key_width = 8);

    void clear();

    int getNumTables();

    void add(cv::Mat &descriptors, int id);

    void match(cv::Mat &descriptors, std::vector<int> &matches);

    void matchAndAdd(cv::Mat &descriptors, int id, std::vector<int> &matches);

    void remove(cv::Mat &descriptors, int id);

    std::vector<boost::shared_ptr<FastLshTable> > tables_;
    int key_width_;
};

class LshSetRecognizer : public PlaceRecognizer
{
public:
    LshSetRecognizer();
    ~LshSetRecognizer();

protected:
    void clearImpl();
    std::vector<int> searchAndAddPlaceImpl(std::string id, std::vector<SensorDataPtr> data);
    void addPlaceImpl(std::string id, std::vector<SensorDataPtr> data);
    std::vector<int> searchImpl(std::string id, std::vector<SensorDataPtr> data);
    void removePlaceImpl(std::string id, std::vector<SensorDataPtr> data);

    bool init_;
    FastLshSet fast_matcher_;
};

#endif
