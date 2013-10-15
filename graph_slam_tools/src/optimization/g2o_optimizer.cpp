#include <graph_slam_tools/optimization/g2o_optimizer.h>
#include <g2o/core/hyper_dijkstra.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/sclam2d/types_sclam2d.h>

G2oOptimizer::G2oOptimizer() : GraphOptimizer()
{
    linear_solver_ = new SlamLinearSolver();
    block_solver_ = new SlamBlockSolver(linear_solver_);
    solver_ = new g2o::OptimizationAlgorithmLevenberg(block_solver_);
    optimizer_.setAlgorithm(solver_);
    optimizer_.setVerbose(false);

    clear();
}

G2oOptimizer::~G2oOptimizer()
{
}

void G2oOptimizer::addGraphImpl(SlamGraph &graph)
{
    clear();

    for (auto node_it = graph.nodeIterator(); node_it.first != node_it.second; ++node_it.first) {
        addVertex(node_it.first->second);
    }

    for (auto edge_it = graph.edgeIterator(); edge_it.first != edge_it.second; ++edge_it.first) {
        SlamEdge &edge = edge_it.first->second;
        if (graph.existsNode(edge.id_from_) &&graph.existsNode((edge.id_to_))) {
            addEdge(edge.id_, graph);
        }
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
            graph.node(node_id_to_g2o_map_.right.at(node_vertex)).pose_.linear() = node_vertex->estimate().rotation();
            graph.node(node_id_to_g2o_map_.right.at(node_vertex)).pose_.translation() = node_vertex->estimate().translation();
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

    setFixedNodes();

    optimizer_.optimize(config_.iterations, false);
}

void G2oOptimizer::clear()
{
    optimizer_.clear();
    node_id_to_g2o_map_.clear();
    edge_id_to_g2o_map_.clear();
    optimizer_count_ = 0;
}

void G2oOptimizer::addVertex(const SlamNode &node)
{
    Eigen::Isometry3d node_pose = node.pose_;
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

void G2oOptimizer::addEdge(std::string id, SlamGraph &graph)
{
    SlamEdge edge = graph.edge(id);

    if (node_id_to_g2o_map_.left.find(edge.id_from_) == node_id_to_g2o_map_.left.end()) {
        ROS_ERROR("did not find edge.id_from_");
        return;
    }
    if (node_id_to_g2o_map_.left.find(edge.id_to_) == node_id_to_g2o_map_.left.end()) {
        ROS_ERROR("did not find edge.id_to_");
        return;
    }

    if (edge.type_ ==  graph_slam_msgs::Edge::TYPE_2D_WHEEL_ODOMETRY) {
        Eigen::Isometry3d odom = edge.transform_;
        if (config_.use_odometry_parameters) {
            Eigen::Vector3d rpy = g2o::internal::toEuler(odom.linear());
            double diff_time_ = fabs((graph.node(edge.id_to_).stamp_ - graph.node(edge.id_from_).stamp_).toSec());
            g2o::MotionMeasurement mma(odom.translation()(0), odom.translation()(1), rpy(2), diff_time_);
            g2o::VelocityMeasurement vel = g2o::OdomConvert::convertToVelocity(mma);
            vel.setVl(graph.odom()(0) * vel.vl());
            vel.setVr(graph.odom()(1) * vel.vr());
            g2o::MotionMeasurement mmb = g2o::OdomConvert::convertToMotion(vel, graph.odom()(2));
            odom.translation()(0) = mmb.x();
            odom.translation()(1) = mmb.y();
            rpy(2) = mmb.theta();
            odom.linear() = g2o::internal::fromEuler(rpy);
        }

        g2o::EdgeSE3* edge_odom = new g2o::EdgeSE3();
        edge_odom->setId(optimizer_count_);
        edge_odom->vertices()[0] = node_id_to_g2o_map_.left.at(edge.id_from_);
        edge_odom->vertices()[1] = node_id_to_g2o_map_.left.at(edge.id_to_);
        edge_odom->setMeasurement(edge.displacement_from_ * odom * edge.displacement_to_.inverse());
        edge_odom->information() = edge.information_;
        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber();
        rk->setDelta(1.);
        edge_odom->setRobustKernel(rk);
        optimizer_.addEdge(edge_odom);
        edge_id_to_g2o_map_.insert(g2o_edge_bimap::value_type(edge.id_, dynamic_cast<g2o::HyperGraph::Edge*>(edge_odom)));
    } else if (edge.type_ ==  graph_slam_msgs::Edge::TYPE_3D_FULL) {
        g2o::EdgeSE3* edge_sensor = new g2o::EdgeSE3();
        edge_sensor->setId(optimizer_count_);
        edge_sensor->vertices()[0] = node_id_to_g2o_map_.left.at(edge.id_from_);
        edge_sensor->vertices()[1] = node_id_to_g2o_map_.left.at(edge.id_to_);
        edge_sensor->setMeasurement(edge.displacement_from_ * graph.sensor(edge.sensor_from_) * edge.transform_ * graph.sensor(edge.sensor_to_).inverse() * edge.displacement_to_.inverse());
        edge_sensor->information() = edge.information_;
        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber();
        rk->setDelta(1.);
        edge_sensor->setRobustKernel(rk);
        optimizer_.addEdge(edge_sensor);
        edge_id_to_g2o_map_.insert(g2o_edge_bimap::value_type(edge.id_, dynamic_cast<g2o::HyperGraph::Edge*>(edge_sensor)));
    }

    optimizer_count_++;
}

void G2oOptimizer::setFixedNodes()
{
    g2o::OptimizableGraph::VertexSet sources;

    //TODO Should always fix the oldest node.

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
            for (auto it = ordered_vertices.begin(); it != ordered_vertices.end(); ++it) {
                g2o::OptimizableGraph::Vertex* v = static_cast<g2o::OptimizableGraph::Vertex*>(it->second);

                if (d.visited().count(v) == 0) {
                    all_fixed = false;
                    v->setFixed(true);
                    sources.insert(v);
                    break;
                }
            }
        }
    }
}
