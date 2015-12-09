#include <graph_slam_tools/pr/place_recognizer.h>
#include <graph_slam_tools/transformation/feature_transformation_estimator.h>
#include <pcl/common/transformation_from_correspondences.h>
#include <chrono>
#include <fstream>

using std::chrono::duration_cast;
using std::chrono::microseconds;
using std::chrono::system_clock;

#define E_ROT 45
#define E_TRANS 1.5
#define DT 5.

PlaceRecognizer::PlaceRecognizer()
{
    running_ = true;
    place_recognition_thread_ = std::thread(&PlaceRecognizer::placeRecognitionThread, this);
    place_count_ = 0;
    performace_file_ = "";
}

PlaceRecognizer::~PlaceRecognizer() {
    running_ = false;
    place_recognition_thread_.join();
}

void PlaceRecognizer::setConfig(graph_slam_tools::PlaceRecognizerConfig config) {
    config_ = config;
}

void PlaceRecognizer::clear() {
    pr_mutex_.lock();
    res_mutex_.lock();
    clearImpl();

    pr_queue_.clear();
    potential_neighbors_.clear();
    place_id_map_.clear();
    pr_time_map_.clear();
    place_count_ = 0;
    res_mutex_.unlock();
    pr_mutex_.unlock();
}

void PlaceRecognizer::addNode(const SlamNode &node) {
    res_mutex_.lock();
    pr_time_map_[node.id_] = node.stamps_.front();
    pr_queue_.push_back(std::make_pair(node.id_, node.sensor_data_));
    res_mutex_.unlock();
}

std::vector<std::pair<std::string, std::string> > PlaceRecognizer::searchAndAddPlace(std::string id, std::vector<SensorDataPtr> data)
{
    pr_mutex_.lock();
//    if (place_id_map_.left.size() == 0) {
//        pr_mutex_.unlock();
//        return std::vector<std::pair<std::string, std::string> >();
//    }

    if (place_id_map_.right.find(id) != place_id_map_.right.end()) {
        ROS_ERROR("tried to add existing place");
        pr_mutex_.unlock();
        return std::vector<std::pair<std::string, std::string> >();
    }

    std::vector<int> neighbors = searchAndAddPlaceImpl(id, data);
    place_id_map_.insert(IdMap::value_type(place_count_, id));
    place_count_++;

    // Remove self matches.
    std::vector<std::string> neighbors_mapped;
    int pr_count = 0;
    for (auto neighbor : neighbors) {
        if (place_id_map_.left.find(neighbor) != place_id_map_.left.end()) {
            if (fabs((pr_time_map_[place_id_map_.left.at(neighbor)] - pr_time_map_[id]).toSec()) > 5.) {
                neighbors_mapped.push_back(place_id_map_.left.at(neighbor));
                pr_count++;

                // Take only the knn (assumes nearest neighbors to be sorted.
                if (pr_count >= config_.k_nearest_neighbors) {
                    break;
                }
            }
        }
    }

    // Filter matches based on position.
    std::vector<std::pair<std::string, std::string> > res;
    for (auto n : neighbors_mapped) {
        auto neighbor = std::make_pair(n, id);
        if (checked_.find(neighbor) == checked_.end()) {
            res.push_back(neighbor);
            checked_.insert(neighbor);
        }
    }
    pr_mutex_.unlock();

    ROS_DEBUG_NAMED("place recognition", "found %d neighbors", (int)res.size());
    return res;
}

void PlaceRecognizer::addPlace(const SlamNode &node) {
    pr_time_map_[node.id_] = node.stamps_.front();
    addPlace(node.id_, node.sensor_data_);
}

void PlaceRecognizer::addPlaceQueue(const SlamNode &node) {
    res_mutex_.lock();
    pr_time_map_[node.id_] = node.stamps_.front();
    pr_add_queue_.push_back(std::make_pair(node.id_, node.sensor_data_));
    res_mutex_.unlock();
}

void PlaceRecognizer::addPlace(std::string id, std::vector<SensorDataPtr> data)
{
    pr_mutex_.lock();
    if (place_id_map_.right.find(id) == place_id_map_.right.end()) {
        addPlaceImpl(id, data);
        place_id_map_.insert(IdMap::value_type(place_count_, id));
        place_count_++;
    } else {
        ROS_ERROR("tried to add existing place");
    }
    pr_mutex_.unlock();
}

std::vector<std::pair<std::string, std::string> > PlaceRecognizer::searchPlace(const SlamNode &node)
{
    return searchPlace(node.id_, node.sensor_data_);
}

std::vector<std::pair<std::string, std::string> > PlaceRecognizer::searchPlace(std::string id, std::vector<SensorDataPtr> data)
{
    pr_mutex_.lock();
    if (place_id_map_.left.size() == 0) {
        pr_mutex_.unlock();
        return std::vector<std::pair<std::string, std::string> >();
    }

    // Search places based only on sensor data.
    std::vector<int> neighbors = searchImpl(id, data);

    // Remove self matches.
    std::vector<std::string> neighbors_mapped;
    int pr_count = 0;
    for (auto neighbor : neighbors) {
        if (place_id_map_.left.find(neighbor) != place_id_map_.left.end()) {
            if (fabs((pr_time_map_[place_id_map_.left.at(neighbor)] - pr_time_map_[id]).toSec()) > 5.) {
                neighbors_mapped.push_back(place_id_map_.left.at(neighbor));
                pr_count++;

                // Take only the knn (assumes nearest neighbors to be sorted.
                if (pr_count >= config_.k_nearest_neighbors) {
                    break;
                }
            }
        }
    }

    // Filter matches based on position.
    std::vector<std::pair<std::string, std::string> > res;
    for (auto n : neighbors_mapped) {
        auto neighbor = std::make_pair(n, id);
        if (checked_.find(neighbor) == checked_.end()) {
            res.push_back(neighbor);
            checked_.insert(neighbor);
        }
    }
    pr_mutex_.unlock();
    return res;
}

void PlaceRecognizer::removePlaceQueue(const SlamNode &node)
{
    res_mutex_.lock();
    pr_remove_queue_.push_back(std::make_pair(node.id_, node.sensor_data_));
    res_mutex_.unlock();
}

void PlaceRecognizer::removePlace(std::string id, std::vector<SensorDataPtr> data) {
    pr_mutex_.lock();
    if (place_id_map_.right.find(id) != place_id_map_.right.end()) {
        removePlaceImpl(id, data);
        place_id_map_.right.erase(id);
        pr_time_map_.erase(id);
    } else {
        res_mutex_.lock();
        bool found = false;
        int i = 0;
        for (auto new_place : pr_queue_) {
            if (new_place.first == id) {
                found = true;
                break;
            }
            i++;
        }
        if (found) {
            ROS_WARN("removed place that will be added in the future");
            pr_queue_.erase(pr_queue_.begin() + i);
        } else {
            ROS_ERROR("tried to remove a non-existing place: %s", id.c_str());
        }
        res_mutex_.unlock();
    }
    pr_mutex_.unlock();
}

void PlaceRecognizer::setPlaceTime(std::string id, ros::Time stamp)
{
    pr_time_map_[id] = stamp;
}

std::vector<std::pair<std::string, std::string> > PlaceRecognizer::recognizedPlaces()
{
    res_mutex_.lock();
    auto res = potential_neighbors_;
    potential_neighbors_.clear();
    res_mutex_.unlock();
    return res;
}

bool PlaceRecognizer::hasRecognizedPlaces()
{
    return (potential_neighbors_.size() > 0);
}

void PlaceRecognizer::setPerformanceFile(std::string file)
{
    performace_file_ = file;
}

void PlaceRecognizer::placeRecognitionThread() {
    while (running_) {
        // Search for one node.
        if (pr_queue_.size() > 0) {
            res_mutex_.lock();
            auto new_place = pr_queue_.back();
            pr_queue_.pop_back();
            res_mutex_.unlock();

            double runtime = 0;
            auto start = system_clock::now();
            std::vector<std::pair<std::string,std::string> > neighbors = searchAndAddPlace(new_place.first, new_place.second);
            runtime = (0.001 * duration_cast<microseconds>(system_clock::now() - start).count());

//            std::vector<std::pair<std::string,std::string> > neighbors = searchPlace(new_place.first, new_place.second);
//            runtime += (0.001 * duration_cast<microseconds>(system_clock::now() - start).count());

            res_mutex_.lock();
            // Add all new matches to the potential neighbor list.
            for (auto neighbor : neighbors) {
                potential_neighbors_.push_back(neighbor);
            }
            res_mutex_.unlock();

            // Add new place.
//            start = system_clock::now();
//            addPlace(new_place.first, new_place.second);
//            runtime += (0.001 * duration_cast<microseconds>(system_clock::now() - start).count());
            ROS_DEBUG("PLACE RECOGNITION: %f ms", 0.001 * duration_cast<microseconds>(system_clock::now() - start).count());

            if (performace_file_ != "") {
                std::ofstream of;
                of.open(performace_file_, std::ios_base::out | std::ios_base::app);
                of << std::setprecision(16) << ros::Time::now() << " " << runtime << std::endl;
            }
        }

        // Remove one node.
        if (pr_remove_queue_.size() > 0) {
            res_mutex_.lock();
            auto remove_place = pr_remove_queue_.back();
            pr_remove_queue_.pop_back();
            res_mutex_.unlock();

            removePlace(remove_place.first, remove_place.second);
        }

        // Add one node.
        if (pr_add_queue_.size() > 0) {
            res_mutex_.lock();
            auto add_place = pr_add_queue_.back();
            pr_add_queue_.pop_back();
            res_mutex_.unlock();

            addPlace(add_place.first, add_place.second);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
