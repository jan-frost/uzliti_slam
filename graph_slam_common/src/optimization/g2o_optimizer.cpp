#include <graph_slam_tools/optimization/g2o_optimizer.h>
#include <g2o/core/hyper_dijkstra.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/sclam2d/types_sclam2d.h>
#include <chrono>

using std::chrono::duration_cast;
using std::chrono::microseconds;
using std::chrono::milliseconds;
using std::chrono::minutes;
using std::chrono::system_clock;

G2oOptimizer::G2oOptimizer() : GraphOptimizer()
{
    linear_solver_ = new SlamLinearSolver();
    block_solver_ = new SlamBlockSolver(linear_solver_);
    solver_ = new g2o::OptimizationAlgorithmLevenberg(block_solver_);
    optimizer_.setAlgorithm(solver_);
    optimizer_.setVerbose(false);

    ros::NodeHandle nh_private("~");
    double cluster_size;
    nh_private.param<double>("cluster_size", cluster_size, 8);
    ROS_INFO("init transformation filter with cluster size %f", cluster_size);
    edge_filter_.reset(new TransformationFilter(5., cluster_size));

    clear();
}

G2oOptimizer::~G2oOptimizer()
{
}

void G2oOptimizer::addGraphImpl(SlamGraph &graph)
{
    clear();

    g2o::ParameterSE3Offset* gps_offset = new g2o::ParameterSE3Offset();
    gps_offset->setId(0);
    gps_offset->setOffset();
    optimizer_.addParameter(gps_offset);

    for (auto node_it = graph.nodeIterator(); node_it.first != node_it.second; ++node_it.first) {
        addVertex(node_it.first->second);
    }

    for (auto sensor_it = graph.sensorIterator(); sensor_it.first != sensor_it.second; ++sensor_it.first) {
        sensor_transforms_[sensor_it.first->first] = sensor_it.first->second;
    }
    edge_filter_->sensor_transforms_ = sensor_transforms_;

    auto start = system_clock::now();
    std::unordered_set<std::string> edges_to_remove = edge_filter_->allEdges();
    for (auto edge_it = graph.edgeIterator(); edge_it.first != edge_it.second; ++edge_it.first) {
        SlamEdge &edge = edge_it.first->second;
        if (graph.existsNode(edge.id_from_) && graph.existsNode(edge.id_to_)) {
            if (edge.type_ == graph_slam_msgs::Edge::TYPE_2D_WHEEL_ODOMETRY) {
                addOdometryEdge(edge.id_, graph);
            } /*else if (edge.type_ == graph_slam_msgs::Edge::TYPE_2D_LASER) {
                addFeatureEdge(edge);
            }*/ else {
                edge_filter_->add(edge, graph.node(edge.id_from_), graph.node(edge.id_to_));
                edges_to_remove.erase(edge.id_);
            }
        }
    }

    // Remove deleted edges from the filter.
    for (auto edge_id : edges_to_remove) {
        edge_filter_->remove(edge_id);
    }
    ROS_DEBUG_NAMED("edge_filter", "edge add %f ms", 0.001 * duration_cast<microseconds>(system_clock::now() - start).count());

    // Add edges from filter.
    start = system_clock::now();
    edge_filter_->calcValidEdges();
    std::vector<SlamEdge> filtered_edges = edge_filter_->validEdges();
    ROS_DEBUG_NAMED("edge_filter", "edge filter valid %f ms", 0.001 * duration_cast<microseconds>(system_clock::now() - start).count());
    for (auto edge : filtered_edges) {
        graph.edge(edge.id_).valid_ = true;
        addFeatureEdge(edge);
    }
}

void G2oOptimizer::storeImpl(SlamGraph &graph)
{
    g2o::HyperGraph::VertexIDMap vertices = optimizer_.vertices();
    g2o::HyperGraph::VertexIDMap::iterator vert_it;
    for (vert_it = vertices.begin() ; vert_it != vertices.end(); vert_it++) {
        g2o::VertexSE3* node_vertex = static_cast<g2o::VertexSE3*>(vert_it->second);
        if (node_id_to_g2o_map_.right.find(node_vertex) != node_id_to_g2o_map_.right.end() &&
                graph.existsNode(node_id_to_g2o_map_.right.at(node_vertex))) {
            graph.node(node_id_to_g2o_map_.right.at(node_vertex)).pose_ = node_vertex->estimate();
            graph.node(node_id_to_g2o_map_.right.at(node_vertex)).optimized_ = true;
        }
    }

    // Update edge error.
    g2o::HyperGraph::EdgeSet edges = optimizer_.edges();
    for (auto edge_it = edges.begin(); edge_it != edges.end(); edge_it++) {
        g2o::EdgeSE3* edge_vertex = static_cast<g2o::EdgeSE3*>(*edge_it);
        if (edge_vertex) {
            edge_vertex->computeError();
            g2o::Vector6d error = edge_vertex->error();
            double error_norm = error.norm();
            if (edge_id_to_g2o_map_.right.find(edge_vertex) != edge_id_to_g2o_map_.right.end() &&
                    graph.existsEdge(edge_id_to_g2o_map_.right.at(edge_vertex))) {
                SlamEdge &edge = graph.edge(edge_id_to_g2o_map_.right.at(edge_vertex));
                edge.age_++;
                edge.error_ = error_norm;
            }
        }
    }
}

void G2oOptimizer::optimizeImpl()
{
    if (!optimizer_.initializeOptimization()) {
        ROS_ERROR("g2o initialization failed.");
        return;
    }

    if (!has_gps_measurement_) {
        setFixedNodes();
    }

    optimizer_.optimize(config_.iterations, false);
}

void G2oOptimizer::clear()
{
    optimizer_.clear();
    node_id_to_g2o_map_.clear();
    edge_id_to_g2o_map_.clear();
    has_gps_measurement_ = false;
    optimizer_count_ = 0;
}

void G2oOptimizer::addVertex(const SlamNode &node)
{
    Eigen::Isometry3d node_pose = node.pose_;

    if (config_.optimize_xy_only) {
        Eigen::Vector3d rpy = g2o::internal::toEuler(node_pose.linear());
        rpy(0) = 0;
        rpy(1) = 0;
        node_pose.linear() = g2o::internal::fromEuler(rpy);
        node_pose.translation()(2) = 0;
    }

    if (node_id_to_g2o_map_.left.find(node.id_) != node_id_to_g2o_map_.left.end()) {
        ROS_ERROR("node already exists: %s", node.id_.c_str());

        g2o::VertexSE3* pose_vertex = static_cast<g2o::VertexSE3*>(node_id_to_g2o_map_.left.at(node.id_));
        pose_vertex->setEstimate(node_pose);
        pose_vertex->setFixed(node.fixed_);        
    } else {
        g2o::VertexSE3 *pose_vertex =  new g2o::VertexSE3();
        pose_vertex->setId(optimizer_count_);
        pose_vertex->setEstimate(node_pose);
        pose_vertex->setFixed(node.fixed_);
        optimizer_.addVertex(pose_vertex);
        optimizer_count_++;

        node_id_to_g2o_map_.insert(g2o_vertex_bimap::value_type(node.id_, dynamic_cast<g2o::HyperGraph::Vertex*>(pose_vertex)));
    }
}

void G2oOptimizer::addOdometryEdge(std::string id, SlamGraph &graph)
{
    SlamEdge edge = graph.edge(id);

    if (node_id_to_g2o_map_.left.find(edge.id_from_) == node_id_to_g2o_map_.left.end()) {
        ROS_DEBUG("did not find edge.id_from_");
        return;
    }
    if (node_id_to_g2o_map_.left.find(edge.id_to_) == node_id_to_g2o_map_.left.end()) {
        ROS_DEBUG("did not find edge.id_to_");
        return;
    }

    if (graph.node(edge.id_from_).fixed_ && !graph.node(edge.id_to_).fixed_) {
        ROS_DEBUG("omit odometry edge from last fixed node");
        return;
    }

    Eigen::Isometry3d odom = edge.transform_;
    if (config_.use_odometry_parameters) {
        Eigen::Vector3d rpy = g2o::internal::toEuler(odom.linear());
        double diff_time_ = fabs(edge.diff_time_.toSec());
        g2o::MotionMeasurement mma(odom.translation()(0), odom.translation()(1), rpy(2), diff_time_);
        g2o::VelocityMeasurement vel = g2o::OdomConvert::convertToVelocity(mma);
        g2o::MotionMeasurement mmb = g2o::OdomConvert::convertToMotion(vel, 1);
        odom.translation()(0) = mmb.x();
        odom.translation()(1) = mmb.y();
        rpy(2) = mmb.theta();
        odom.linear() = g2o::internal::fromEuler(rpy);

//        Eigen::Vector3d rpy = g2o::internal::toEuler(odom.linear());
//        double drift_theta = graph.odom()(1) * fabs(rpy(2));
//        double drift_trans = graph.odom()(2) * odom.translation().norm();
//        Eigen::AngleAxisd drift_rot(drift_theta + drift_trans, Eigen::Vector3d::UnitZ());
//        odom.translation() = graph.odom()(0) * (drift_rot * odom.translation());
//        rpy(2) += drift_theta + drift_trans;
//        odom.linear() = g2o::internal::fromEuler(rpy);
    }

    odom = edge.displacement_from_ * odom * edge.displacement_to_.inverse();

    if (config_.optimize_xy_only) {
        Eigen::Vector3d rpy = g2o::internal::toEuler(odom.linear());
        rpy(0) = 0;
        rpy(1) = 0;
        odom.linear() = g2o::internal::fromEuler(rpy);
        odom.translation()(2) = 0;
    }

    g2o::EdgeSE3* edge_odom = new g2o::EdgeSE3();
    edge_odom->setId(optimizer_count_);
    edge_odom->vertices()[0] = node_id_to_g2o_map_.left.at(edge.id_from_);
    edge_odom->vertices()[1] = node_id_to_g2o_map_.left.at(edge.id_to_);
    edge_odom->setMeasurement(odom);
    edge_odom->information() = edge.information_;
    optimizer_.addEdge(edge_odom);
    edge_id_to_g2o_map_.insert(g2o_edge_bimap::value_type(edge.id_, dynamic_cast<g2o::HyperGraph::Edge*>(edge_odom)));

//        g2o::EdgeSE3Prior *edge_gps = new g2o::EdgeSE3Prior();
//        edge_gps->setId(optimizer_count_);
//        edge_gps->vertices()[0] = node_id_to_g2o_map_.left.at(edge.id_from_);
//        edge_gps->setMeasurement(edge.transform_);
//        edge_gps->information() = edge.information_;
//        edge_gps->setParameterId(0, 0);
//        optimizer_.addEdge(edge_gps);
//        edge_id_to_g2o_map_.insert(g2o_edge_bimap::value_type(edge.id_, dynamic_cast<g2o::HyperGraph::Edge*>(edge_gps)));
//        has_gps_measurement_ = true;

    optimizer_count_++;
}

void G2oOptimizer::addFeatureEdge(const SlamEdge &edge)
{
    if (node_id_to_g2o_map_.left.find(edge.id_from_) == node_id_to_g2o_map_.left.end() ||
            node_id_to_g2o_map_.left.find(edge.id_to_) == node_id_to_g2o_map_.left.end()) {
        ROS_DEBUG("did not find a node of edge %s", edge.id_.c_str());
        edge_filter_->remove(edge.id_);
        return;
    }

    if (static_cast<g2o::VertexSE3*>(node_id_to_g2o_map_.left.at(edge.id_from_))->fixed() &&
            static_cast<g2o::VertexSE3*>(node_id_to_g2o_map_.left.at(edge.id_to_))->fixed()) {
        edge_id_to_g2o_map_.insert(g2o_edge_bimap::value_type(edge.id_, NULL));
        return;
    }

    g2o::EdgeSE3* edge_sensor = new g2o::EdgeSE3();
    edge_sensor->setId(optimizer_count_);
    edge_sensor->vertices()[0] = node_id_to_g2o_map_.left.at(edge.id_from_);
    edge_sensor->vertices()[1] = node_id_to_g2o_map_.left.at(edge.id_to_);

    Eigen::Isometry3d transform = edge.displacement_from_ * sensor_transforms_[edge.sensor_from_] * edge.transform_ * sensor_transforms_[edge.sensor_to_].inverse() * edge.displacement_to_.inverse();
    if (config_.optimize_xy_only) {
        Eigen::Vector3d rpy = g2o::internal::toEuler(transform.linear());
        rpy(0) = 0;
        rpy(1) = 0;
        transform.linear() = g2o::internal::fromEuler(rpy);
        transform.translation()(2) = 0;
    }
    edge_sensor->setMeasurement(transform);

    edge_sensor->information() = edge.information_;
    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber();
    rk->setDelta(1.);
    edge_sensor->setRobustKernel(rk);
    optimizer_.addEdge(edge_sensor);
    edge_id_to_g2o_map_.insert(g2o_edge_bimap::value_type(edge.id_, dynamic_cast<g2o::HyperGraph::Edge*>(edge_sensor)));

    optimizer_count_++;
}

void G2oOptimizer::setFixedNodes()
{
    g2o::OptimizableGraph::VertexSet sources;

    //TODO Sollte immer am ältesten Knoten fixen!

    // Set fixed vertices
    g2o::HyperGraph::VertexIDMap vertices = optimizer_.vertices();
    g2o::HyperGraph::VertexIDMap::iterator vert_it;
    for (vert_it = vertices.begin() ; vert_it != vertices.end(); vert_it++) {
        g2o::VertexSE3* node_vertex = dynamic_cast<g2o::VertexSE3*>(vert_it->second);
        if (node_vertex && node_vertex->fixed()) {
            sources.insert(node_vertex);
        }
    }

    bool all_fixed = false;
    g2o::UniformCostFunction f;
    g2o::HyperDijkstra d(&optimizer_);

    std::map<int, g2o::HyperGraph::Vertex *> ordered_vertices(optimizer_.vertices().begin(), optimizer_.vertices().end());
    while (!all_fixed) {
        all_fixed = true;
        d.shortestPaths(sources, &f);

        // Fix first unvisited node.
        if (d.visited().size() != optimizer_.vertices().size()) {
            all_fixed = false;
            std::string oldest_node = "";
            for (auto it = ordered_vertices.begin(); it != ordered_vertices.end(); ++it) {
                g2o::OptimizableGraph::Vertex* v = static_cast<g2o::OptimizableGraph::Vertex*>(it->second);

                if (d.visited().count(v) == 0) {
//                    all_fixed = false;
//                    v->setFixed(true);
//                    sources.insert(v);
//                    break;
                    if (oldest_node == "" || node_id_to_g2o_map_.right.at(v) < oldest_node) {
                        oldest_node = node_id_to_g2o_map_.right.at(v);
                    }
                }
            }

            g2o::OptimizableGraph::Vertex* v = static_cast<g2o::OptimizableGraph::Vertex*>(node_id_to_g2o_map_.left.at(oldest_node));
            v->setFixed(true);
            sources.insert(v);
        }
    }
}
