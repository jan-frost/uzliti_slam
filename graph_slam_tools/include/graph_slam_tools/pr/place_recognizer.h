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

#ifndef PLACE_RECOGNIZER_H
#define PLACE_RECOGNIZER_H

#include <graph_slam_tools/graph/slam_node.h>
#include <graph_slam_tools/PlaceRecognizerConfig.h>
#include <thread>
#include <chrono>
#include <stack>
#include <mutex>

using std::chrono::duration_cast;
using std::chrono::microseconds;
using std::chrono::system_clock;

/*
 * Abstract base class for place recognition in SLAM.
 */

class PlaceRecognizer
{
public:
    PlaceRecognizer() {
        running_ = true;
        place_recognition_thread_ = std::thread(&PlaceRecognizer::placeRecognitionThread, this);
    }

    ~PlaceRecognizer() {
        running_ = false;
        place_recognition_thread_.join();
    }

    void setConfig(graph_slam_tools::PlaceRecognizerConfig config) {
        config_ = config;
    }

    void clear() {
        pr_mutex_.lock();
        clearImpl();

        pr_queue_.clear();
        pr_pose_map_.clear();
        potential_neighbors_.clear();
        last_neighbors_.clear();
        last_queries_.clear();
        recognition_graph_.clear();
        pr_mutex_.unlock();
    }

    void searchAndAddPlace(const SlamNode &node) {
        pr_mutex_.lock();
        pr_queue_.push_back(std::make_pair(node.id_, node.sensor_data_));
        pr_pose_map_[node.id_] = node.pose_;
        pr_mutex_.unlock();
    }

    void addPlace(const SlamNode &node) {
        pr_mutex_.lock();
        addPlaceImpl(node.id_, node.sensor_data_);
        pr_mutex_.unlock();
    }

    std::vector<std::pair<std::string, std::string> > recognizedPlaces() {
        pr_mutex_.lock();
        auto res = potential_neighbors_;
        potential_neighbors_.clear();
        pr_mutex_.unlock();
        return res;
    }

    bool hasRecognizedPlaces() {
        return (potential_neighbors_.size() > 0);
    }

protected:    
    void placeRecognitionThread() {
        while (running_) {
            pr_mutex_.lock();
            if (pr_queue_.size() > 0) {
                auto new_place = pr_queue_.back();
                pr_queue_.pop_back();
                pr_mutex_.unlock();

                // Search places based only on sensor data.
                std::vector<std::string> neighbors = searchImpl(new_place.first, new_place.second);

                // Filter matches based on position.
                std::vector<std::string> neighbors_filtered = filter(new_place.first, neighbors);

                // Add all new matches to the potential neighbor list.
                pr_mutex_.lock();
                for (auto neighbor : neighbors_filtered) {
                    potential_neighbors_.push_back(std::make_pair(neighbor, new_place.first));
                }
                pr_mutex_.unlock();

                // Add new place.
                addPlaceImpl(new_place.first, new_place.second);
            } else {
                pr_mutex_.unlock();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    virtual void clearImpl() = 0;

    virtual void addPlaceImpl(std::string id, std::vector<SensorDataPtr> data) = 0;

    virtual std::vector<std::string> searchImpl(std::string id, std::vector<SensorDataPtr> data) = 0;

    std::vector<std::string> filter(std::string id, std::vector<std::string> neighbors) {
        last_queries_.push_back(id);
        last_neighbors_.push_back(neighbors);

        auto start = system_clock::now();
        recognition_graph_.clear();
        buildRecognitionGraph();
        std::vector<std::string> filtered_neighbors = shortestPaths();

        if ((int)last_neighbors_.size() > config_.history) {
            last_queries_.erase(last_queries_.begin());
            last_neighbors_.erase(last_neighbors_.begin());
        }
        ROS_DEBUG("pr time: %f ms", 0.001 * duration_cast<microseconds>(system_clock::now() - start).count());

        return filtered_neighbors;
    }

    void buildRecognitionGraph() {
        for (int i = 0; i < (int)last_neighbors_.size() - 1; i++) {
            for (int j = 0; j < (int)last_neighbors_[i].size(); j++) {
                Eigen::Isometry3d pose_from = pr_pose_map_[last_neighbors_[i][j]];
                for (int h = i + 1; h < (int)last_neighbors_.size(); h++) {
                    for (int k = 0; k < (int)last_neighbors_[h].size(); k++) {
                        Eigen::Isometry3d pose_to = pr_pose_map_[last_neighbors_[h][k]];
                        if ((pose_from.translation() - pose_to.translation()).norm() < 1.0) {
                            recognition_graph_[std::make_pair(i,j)].push_back(std::make_pair(h,k));
                        }
                    }
                }
            }
        }
    }

    void topologicalSortUtil(std::pair<int,int> v, std::map<std::pair<int,int>, bool> &visited, std::stack<std::pair<int,int> > &stack)
    {
        // Mark the current node as visited
        visited[v] = true;

        // Recur for all the vertices adjacent to this vertex
        for (auto u : recognition_graph_[v]) {
            if (!visited[u]) {
                topologicalSortUtil(u, visited, stack);
            }
        }

        // Push current vertex to stack which stores topological sort
        stack.push(v);
    }

    // The function to find shortest paths from given vertex. It uses recursive
    // topologicalSortUtil() to get topological sorting of given graph.
    std::vector<std::string> shortestPaths()
    {
        std::stack<std::pair<int,int> > stack;
        std::map<std::pair<int,int>, bool> visited;
        std::map<std::pair<int,int>, int> dist;

        // Mark all the vertices as not visited
        for (size_t i = 0; i < last_neighbors_.size(); i++) {
            for (size_t j = 0; j < last_neighbors_[i].size(); j++) {
                visited[std::make_pair(i,j)] = false;
            }
        }

        // Call the recursive helper function to store Topological Sort
        // starting from all vertices one by one
        for (size_t i = 0; i < last_neighbors_.size(); i++) {
            for (size_t j = 0; j < last_neighbors_[i].size(); j++) {
                if (!visited[std::make_pair(i,j)]) {
                    topologicalSortUtil(std::make_pair(i,j), visited, stack);
                }
            }
        }

        // Initialize distances to all vertices as infinite and distance
        // to source as 0
        for (size_t i = 0; i < last_neighbors_.size(); i++) {
            for (size_t j = 0; j < last_neighbors_[i].size(); j++) {
                dist[std::make_pair(i,j)] = std::numeric_limits<int>::max();
            }
        }

        for (size_t j = 0; j < last_neighbors_[0].size(); j++) {
            dist[std::make_pair(0,j)] = 0;
        }

        // Process vertices in topological order
        while (!stack.empty()) {
            // Get the next vertex from topological order
            auto v = stack.top();
            stack.pop();

            // Update distances of all adjacent vertices
            for (auto u : recognition_graph_[v]) {
                if (dist[v] != std::numeric_limits<int>::max()) {
                    int new_dist = (u.first - v.first) + (u.first - v.first - 1);
                    if (dist[u] > dist[v] + new_dist) {
                        dist[u] = dist[v] + new_dist;
                    }
                }
            }
        }

        // Get shortest paths.
        std::vector<std::string> res;
        for (size_t j = 0; j < last_neighbors_.back().size(); j++) {
            if (dist[std::make_pair(last_neighbors_.size() - 1,j)] < 1.5 * last_neighbors_.size()) {
                res.push_back(last_neighbors_.back()[j]);
            }
        }

        return res;
    }

    std::thread place_recognition_thread_;
    bool running_;
    std::mutex pr_mutex_;
    std::vector<std::pair<std::string, std::vector<SensorDataPtr> > > pr_queue_;
    std::map<std::string, Eigen::Isometry3d, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string, Eigen::Isometry3d> > > pr_pose_map_;
    std::vector<std::pair<std::string, std::string> > potential_neighbors_;

    std::vector<std::vector<std::string> > last_neighbors_;
    std::vector<std::string> last_queries_;
    std::map<std::pair<int,int>, std::vector<std::pair<int,int> > > recognition_graph_;

    graph_slam_tools::PlaceRecognizerConfig config_;
};

#endif
