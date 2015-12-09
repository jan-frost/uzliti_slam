#ifndef PLACE_RECOGNIZER_H
#define PLACE_RECOGNIZER_H

#include <boost/bimap.hpp>
#include <graph_slam_tools/graph/slam_node.h>
#include <graph_slam_tools/PlaceRecognizerConfig.h>
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

    void setConfig(graph_slam_tools::PlaceRecognizerConfig config);

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

    void setPerformanceFile(std::string file);

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

    graph_slam_tools::PlaceRecognizerConfig config_;
    std::unordered_set<std::pair<std::string,std::string>, boost::hash<std::pair<std::string,std::string> > > checked_;

    std::string performace_file_;
};

#endif
