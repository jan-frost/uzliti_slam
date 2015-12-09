#ifndef TRANSFORMATION_FILTER_H
#define TRANSFORMATION_FILTER_H

#include <graph_slam_tools/graph/slam_node.h>
#include <graph_slam_tools/transformation/feature_transformation_estimator.h>
#include <unordered_set>

class EdgeData {
public:
    EdgeData(SlamEdge edge, Eigen::Isometry3d from, ros::Time time_from, Eigen::Isometry3d to, ros::Time time_to, bool valid) {
        edge_ = edge;
        pos_from_ = from;
        pos_to_ = to;
        valid_ = valid;
        time_from_ = time_from;
        time_to_ = time_to;
    }

    Eigen::Isometry3d pos_from_;
    ros::Time time_from_;
    Eigen::Isometry3d pos_to_;
    ros::Time time_to_;
    SlamEdge edge_;
    bool valid_;
};

class EdgeCluster {
public:
    EdgeCluster(const SlamEdge &edge, ros::Time stamp_from, ros::Time stamp_to, const SlamNode &from, const SlamNode &to);

    void addEdge(const SlamEdge &edge, ros::Time stamp_from, ros::Time stamp_to, const SlamNode &from, const SlamNode &to);

    void updateEdge(const SlamEdge &edge, const SlamNode &from, const SlamNode &to);

    void removeEdge(std::string id);

    void merge(EdgeCluster cluster);

    bool isPartOfCluster(ros::Time stamp_from, ros::Time stamp_to, double max_dt);

    int size();

    bool operator==(const EdgeCluster& other);

    ros::Time last_added_;
    ros::Time cluster_from_start_;
    ros::Time cluster_from_end_;
    ros::Time cluster_to_start_;
    ros::Time cluster_to_end_;
    bool cluster_closed_;
    bool changed_;
    Eigen::Isometry3d cluster_transformation_;
    int consensus_;

    std::unordered_map<std::string, boost::shared_ptr<EdgeData> > edges_;
};

class TransformationFilter {
public:
    TransformationFilter(double max_dt = 5., int min_size = 10, int max_cluster_size = 100);

    void add(const SlamEdge &edge, const SlamNode &from, const SlamNode &to);

    void remove(std::string id);

    void calcValidEdges();

    std::vector<SlamEdge> validEdges(int skip = 1);

    std::unordered_set<std::string> allEdges();

    std::map<std::string, Eigen::Isometry3d, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string, Eigen::Isometry3d> > > sensor_transforms_;

private:
    void dummyEdgeCallback(SlamEdge &edge) {}

private:
    double max_dt_;
    double min_size_;
    int max_cluster_size_;
    std::vector<boost::shared_ptr<EdgeCluster> > clusters_;
    std::unordered_map<std::string, std::vector<boost::shared_ptr<EdgeCluster> > > edges_;
    FeatureTransformationEstimator fte;
};

#endif

