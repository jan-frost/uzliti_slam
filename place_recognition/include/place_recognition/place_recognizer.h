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

#ifndef PLACE_RECOGNIZER_H
#define PLACE_RECOGNIZER_H

#include <boost/bimap.hpp>
#include <graph_slam_common/slam_node.h>
#include <place_recognition/PlaceRecognizerConfig.h>
#include <thread>
#include <stack>
#include <mutex>
#include <deque>
#include <unordered_map>
#include <unordered_set>
#include <boost/functional/hash.hpp>

typedef boost::bimaps::bimap<int,std::string> IdMap;

/*
 * Abstract base class for place recognition in SLAM.
 */

class PlaceRecognizer
{
public:
    PlaceRecognizer();

    ~PlaceRecognizer();

    void setConfig(place_recognition::PlaceRecognizerConfig config);

    void clear();

    void addNode(const SlamNode &node);

    std::vector<std::pair<std::string, std::string> > searchAndAddPlace(std::string id, std::vector<SensorDataPtr> data);

    void addPlace(const SlamNode &node);

    void addPlaceQueue(const SlamNode &node);

    void addPlace(std::string id, std::vector<SensorDataPtr> data);

    std::vector<std::pair<std::string, std::string> > searchPlace(const SlamNode &node);

    std::vector<std::pair<std::string, std::string> > searchPlace(std::string id, std::vector<SensorDataPtr> data);

    void removePlaceQueue(const SlamNode &node);

    void removePlace(std::string id, std::vector<SensorDataPtr> data);

    void setPlaceTime(std::string id, ros::Time stamp);

    std::vector<std::pair<std::string, std::string> > recognizedPlaces();

    bool hasRecognizedPlaces();

protected:    
    void placeRecognitionThread();

    virtual void clearImpl() = 0;

    virtual std::vector<int> searchAndAddPlaceImpl(std::string id, std::vector<SensorDataPtr> data) = 0;

    virtual void addPlaceImpl(std::string id, std::vector<SensorDataPtr> data) = 0;

    virtual std::vector<int> searchImpl(std::string id, std::vector<SensorDataPtr> data) = 0;

    virtual void removePlaceImpl(std::string id, std::vector<SensorDataPtr> data) = 0;

    std::vector<std::pair<std::string, std::string> > filter(std::string id, std::vector<std::string> neighbors);

    std::thread place_recognition_thread_;
    bool running_;
    std::mutex pr_mutex_;
    std::mutex res_mutex_;

    int place_count_;
    IdMap place_id_map_;

    std::deque<std::pair<std::string, std::vector<SensorDataPtr> > > pr_queue_;
    std::vector<std::pair<std::string, std::vector<SensorDataPtr> > > pr_add_queue_;
    std::vector<std::pair<std::string, std::vector<SensorDataPtr> > > pr_remove_queue_;
    std::unordered_map<std::string, ros::Time > pr_time_map_;
    std::vector<std::pair<std::string, std::string> > potential_neighbors_;

    place_recognition::PlaceRecognizerConfig config_;
    std::unordered_set<std::pair<std::string,std::string>, boost::hash<std::pair<std::string,std::string> > > checked_;
};

#endif
