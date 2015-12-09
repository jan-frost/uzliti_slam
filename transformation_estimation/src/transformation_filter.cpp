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

#include <transformation_estimation/transformation_filter.h>

#include <g2o/types/slam3d/isometry3d_mappings.h>
#include <algorithm>

bool EdgeScoreSort(const EdgeData &a, const EdgeData &b) // Sort edges by matchig score (descending)
{
    if (a.edge_.matching_score_ > b.edge_.matching_score_) {
        return true;
    } else {
        return false;
    }
}

bool EdgeTimeSort(const EdgeData &a, const EdgeData &b) // Sort edges by node timestamps (descending)
{
    if (std::max(a.time_from_, a.time_to_) > std::max(b.time_from_, b.time_to_)) {
        return true;
    } else {
        return false;
    }
}

EdgeCluster::EdgeCluster(const SlamEdge &edge, ros::Time stamp_from, ros::Time stamp_to, const SlamNode &from, const SlamNode &to)
{
    cluster_from_start_ = stamp_from;
    cluster_from_end_ = stamp_from;
    cluster_to_start_ = stamp_to;
    cluster_to_end_ = stamp_to;
    cluster_closed_ = false;
    cluster_transformation_ = Eigen::Isometry3d::Identity();
    consensus_ = 0;

    boost::shared_ptr<EdgeData> data(new EdgeData(edge, from.pose_, stamp_from, to.pose_, stamp_to, edge.valid_));
    edges_[edge.id_] = data;
    if (edge.valid_) {
        consensus_++;
    }
}

void EdgeCluster::addEdge(const SlamEdge &edge, ros::Time stamp_from, ros::Time stamp_to, const SlamNode &from, const SlamNode &to)
{
    last_added_ = ros::Time::now();
    cluster_from_start_ = std::min(stamp_from, cluster_from_start_);
    cluster_from_end_ = std::max(stamp_from, cluster_from_end_);
    cluster_to_start_ = std::min(stamp_to, cluster_to_start_);
    cluster_to_end_ = std::max(stamp_to, cluster_to_end_);
    changed_ = true;

    boost::shared_ptr<EdgeData> data(new EdgeData(edge, from.pose_, stamp_from, to.pose_, stamp_to, edge.valid_));
    edges_[edge.id_] = data;
    if (edge.valid_) {
        consensus_++;
    }
}

void EdgeCluster::updateEdge(const SlamEdge &edge, const SlamNode &from, const SlamNode &to)
{
    if (edges_.find(edge.id_) != edges_.end()) {
        auto edge_data = edges_[edge.id_];
        edge_data->pos_from_ = from.pose_;
        edge_data->pos_to_ = to.pose_;
        edge_data->edge_ = edge;
    }
}

void EdgeCluster::removeEdge(std::string id)
{
    if (edges_.find(id) != edges_.end()) {
        if (edges_[id]->valid_) {
            consensus_--;
        }
        edges_.erase(id);
    }
}

void EdgeCluster::merge(EdgeCluster cluster)
{
    cluster_from_start_ = std::min(cluster.cluster_from_start_, cluster_from_start_);
    cluster_from_end_ = std::max(cluster.cluster_from_end_, cluster_from_end_);
    cluster_to_start_ = std::min(cluster.cluster_to_start_, cluster_to_start_);
    cluster_to_end_ = std::max(cluster.cluster_to_end_, cluster_to_end_);
    changed_ = true;
    consensus_ = consensus_ + cluster.consensus_;
    edges_.insert(cluster.edges_.begin(), cluster.edges_.end());
}

bool EdgeCluster::isPartOfCluster(ros::Time stamp_from, ros::Time stamp_to, double max_dt)
{
    return ((stamp_from - cluster_from_start_).toSec() > -max_dt &&
            (stamp_from - cluster_from_end_).toSec() < max_dt &&
            (stamp_to - cluster_to_start_).toSec() > -max_dt &&
            (stamp_to - cluster_to_end_).toSec() < max_dt);
}

int EdgeCluster::size()
{
    return (int)edges_.size();
}

bool EdgeCluster::operator==(const EdgeCluster &other)
{
    return (cluster_from_start_ == other.cluster_from_start_) &&
           (cluster_from_end_ == other.cluster_from_end_) &&
           (cluster_to_start_ == other.cluster_to_start_) &&
           (cluster_to_end_ == other.cluster_to_end_) &&
           (size() == other.size());
}


TransformationFilter::TransformationFilter(double max_dt, int min_size, int max_cluster_size) :
    fte(boost::bind(&TransformationFilter::dummyEdgeCallback, this, _1))
{
    max_dt_ = max_dt;
    min_size_ = min_size;
    max_cluster_size_ = max_cluster_size;
}

void TransformationFilter::add(const SlamEdge &edge, const SlamNode &from, const SlamNode &to)
{
    if (edges_.find(edge.id_) != edges_.end()) {
//        ROS_DEBUG_NAMED("transformation_filter", "updating edge %s", edge.id_.c_str());
        for (auto cluster : edges_[edge.id_]) {
            cluster->updateEdge(edge, from, to);
        }
        return;
    }

    ROS_DEBUG_NAMED("transformation_filter", "adding edge %s", edge.id_.c_str());
    for (auto stamp_from : from.stamps_) {
        for (auto stamp_to : to.stamps_) {
            // Search matching open cluster.
            std::vector<unsigned int> matched_clusters;
            for (unsigned int i = 0; i < clusters_.size(); i++) {
                auto cluster = clusters_[i];
                if (cluster->size() < max_cluster_size_ && cluster->isPartOfCluster(stamp_from, stamp_to, max_dt_)) {
                    matched_clusters.push_back(i);
//                    break;  // Take first cluster, no merge.
                }
            }

            // Add new cluster, if none matched.
            if (matched_clusters.size() == 0) {
                boost::shared_ptr<EdgeCluster> new_cluster(new EdgeCluster(edge, stamp_from, stamp_to, from, to));
                clusters_.push_back(new_cluster);
                edges_[edge.id_].push_back(new_cluster);
            } else {
                // Add to first matched cluster.
                clusters_[matched_clusters[0]]->addEdge(edge, stamp_from, stamp_to, from, to);
                edges_[edge.id_].push_back(clusters_[matched_clusters[0]]);

                // Merge clusters.
                for (int i = (int)matched_clusters.size() - 1; i >= 1 ; i--) {
                    if (clusters_[matched_clusters[0]]->size() + clusters_[matched_clusters[i]]->size() < max_cluster_size_) {
                        ROS_DEBUG_NAMED("transformation_filter", "merge cluster %d with %d", matched_clusters[0], matched_clusters[i]);
                        for (auto merge_edge_id : clusters_[matched_clusters[i]]->edges_) {
                            for (auto &edge_cluster : edges_[merge_edge_id.first]) {
                                if (edge_cluster == clusters_[matched_clusters[i]]) {
                                    edge_cluster = clusters_[matched_clusters[0]];
                                    break;
                                }
                            }
                        }
//                        std::cout << clusters_[matched_clusters[0]]->size() << " + " << clusters_[matched_clusters[i]]->size();
//                        for (auto edge_data : clusters_[matched_clusters[0]]->edges_) {
//                            std::cout << edge_data.second->edge_.transform_.matrix() << std::endl;
//                        }
//                        for (auto edge_data : clusters_[matched_clusters[i]]->edges_) {
//                            std::cout << edge_data.second->edge_.transform_.matrix() << std::endl;
//                        }
                        clusters_[matched_clusters[0]]->merge(*clusters_[matched_clusters[i]]);
                        clusters_.erase(clusters_.begin() + matched_clusters[i]);
//                        std::cout << " = " << clusters_[matched_clusters[0]]->size() << std::endl;
//                        for (auto edge_data : clusters_[matched_clusters[0]]->edges_) {
//                            std::cout << edge_data.second->edge_.transform_.matrix() << std::endl;
//                        }
                    }
                }
            }
        }
    }
}

void TransformationFilter::remove(std::string id)
{
    if (edges_.find(id) != edges_.end()) {
        for (auto cluster : edges_[id]) {
            cluster->removeEdge(id);
            if (cluster->size() == 0) {
                clusters_.erase(std::remove(clusters_.begin(), clusters_.end(), cluster), clusters_.end());
            }
        }
        edges_.erase(id);
    }
}

void TransformationFilter::calcValidEdges()
{
    int count = 0;
    for (auto &cluster : clusters_) {
//        if (cluster->size() >= min_size_) {
//            std::cout << fabs((cluster->cluster_from_start_ - cluster->cluster_from_end_).toSec()) << std::endl;
//            std::cout << fabs((cluster->cluster_to_start_ - cluster->cluster_to_end_).toSec()) << std::endl;
//        }
        ROS_DEBUG_NAMED("transformation_filter", "cluster %d, size %d", count, cluster->size());
        count++;

        if (cluster->size() < min_size_) {
            continue;
        }
        if (!cluster->changed_ /*&& cluster->consensus_ > min_size_*/) {
            ROS_DEBUG_NAMED("transformation_filter", "cluster did not change");
            continue;
        }
        if (fabs((cluster->cluster_from_start_ - cluster->cluster_from_end_).toSec()) < 2. ||
                fabs((cluster->cluster_to_start_ - cluster->cluster_to_end_).toSec()) < 2.) {
            ROS_DEBUG_NAMED("transformation_filter", "cluster covers too small time");
            continue;
        }

        ROS_DEBUG_NAMED("transformation_filter", "calc valid edges for cluster %d of size %d", count, cluster->size());
        cluster->changed_ = false;

        std::vector<std::string> temp_ids;
        std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d> > poses_from;
        std::vector<Eigen::Isometry3d> poses_to;
        Eigen::MatrixXd P(3, cluster->size());
        Eigen::MatrixXd Q(3, cluster->size());

        int k = 0;
        for (auto data : cluster->edges_) {
            Eigen::Isometry3d from = data.second->pos_from_ *
                    data.second->edge_.displacement_from_ *
                    sensor_transforms_[data.second->edge_.sensor_from_] *
                    data.second->edge_.transform_ *
                    sensor_transforms_[data.second->edge_.sensor_to_].inverse();
            Eigen::Isometry3d to = data.second->pos_to_ * data.second->edge_.displacement_to_;
            P.col(k) = from.translation();

            Q.col(k) = to.translation();

            temp_ids.push_back(data.first);
            poses_from.push_back(from);
            poses_to.push_back(to);

            k++;
        }

        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        int consensus = 0;
        double mse = 0.;
        double max_error = 0.3;
        fte.estimateSVD(P, Q, T, consensus, mse, max_error, 200, 1.0, false);

        Eigen::Array<bool,1,Eigen::Dynamic> consensus_set;
        consensus = fte.consensus3D(P, Q, T, max_error, consensus_set);

        ROS_DEBUG_NAMED("transformation_filter", "old consensus: %d, new consensus: %d", cluster->consensus_, consensus);
        if (consensus >= min_size_ && consensus >= cluster->consensus_) {
            cluster->consensus_ = consensus;
            for (unsigned int i = 0; i < consensus_set.cols(); i++) {
                cluster->edges_[temp_ids[i]]->valid_ = consensus_set(i);
            }
        }
    }
}

std::vector<SlamEdge> TransformationFilter::validEdges(int skip)
{
    std::vector<SlamEdge> res;
    std::set<std::string> valid_edge_ids;
    for (auto cluster : clusters_) {
        // Get valid edges for each cluster.
        std::vector<EdgeData> valid_cluster_edges;
        for (auto edge : cluster->edges_) {
            if (edge.second->valid_) {
                valid_cluster_edges.push_back(*edge.second);
            }
        }

        if (valid_cluster_edges.size() > 0) {
            ROS_DEBUG_NAMED("transformation_filter", "valid cluster edges %d", valid_cluster_edges.size());
        }

        int max_edges = 5;
        if (valid_cluster_edges.size() > 2 * max_edges) {
            // Sort according to matching score.
            std::sort(valid_cluster_edges.begin(), valid_cluster_edges.end(), EdgeScoreSort);

            // Take best k edges.
            for (int i = 0; i < max_edges; i++) {
                valid_edge_ids.insert(valid_cluster_edges[i].edge_.id_);
            }

            // Sort according to timestamp.
            std::sort(valid_cluster_edges.begin(), valid_cluster_edges.end(), EdgeScoreSort);

            // Take equally spread edges.
            double increment = (double) valid_cluster_edges.size() / (double) max_edges;
            for (int i = 0; i < max_edges - 1; i++) {
                valid_edge_ids.insert(valid_cluster_edges[std::floor(increment * i)].edge_.id_);
            }
            valid_edge_ids.insert(valid_cluster_edges.back().edge_.id_);
        } else {
            // Take all edges.
            for (size_t i = 0; i < valid_cluster_edges.size(); i++) {
                valid_edge_ids.insert(valid_cluster_edges[i].edge_.id_);
            }
        }
    }

    for (auto edge_id : valid_edge_ids) {
        res.push_back(edges_[edge_id].front()->edges_[edge_id]->edge_);
    }

//    int count = 0;
//    for (auto edge : edges_) {
//        for (auto cluster : edge.second) {
//            if (cluster->edges_[edge.first]->valid_) {
//                if (count % skip == 0) {
//                    res.push_back(cluster->edges_[edge.first]->edge_);
//                }
//                count++;
//                break;
//            }
//        }
//    }
    ROS_DEBUG_NAMED("transformation_filter", "# valid edges: %d", (int)res.size());

    return res;
}

std::unordered_set<std::string> TransformationFilter::allEdges()
{
    std::unordered_set<std::string> all_edges;
    for (auto edge_it : edges_) {
        all_edges.insert(edge_it.first);
    }
    return all_edges;
}
