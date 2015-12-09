#include <graph_slam_tools/optimization/sensor_transform_optimizer.h>

#include <g2o/core/hyper_dijkstra.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <graph_slam_tools/optimization/edge_rot_trans_sensor_calib.h>
#include <graph_slam_tools/optimization/edge_se3_sensor_calib.h>
#include <g2o/types/slam3d/isometry3d_mappings.h>

SensorTransformOptimizer::SensorTransformOptimizer() : G2oOptimizer()
{
}

SensorTransformOptimizer::~SensorTransformOptimizer()
{
}

void SensorTransformOptimizer::addGraphImpl(SlamGraph &graph)
{
    clear();

    for (auto node_it = graph.nodeIterator(); node_it.first != node_it.second; ++node_it.first) {
        addVertex(node_it.first->second);
    }

    for (auto sensor_it = graph.sensorIterator(); sensor_it.first != sensor_it.second; ++sensor_it.first) {
        g2o::VertexSE3 *sensor_vertex = new g2o::VertexSE3();
        sensor_vertex->setId(optimizer_count_);
        sensor_vertex->setEstimate(sensor_it.first->second);
        sensor_vertex->setFixed(true);                          // Sensor transform optimizeation still buggy.
        optimizer_.addVertex(sensor_vertex);
        sensor_id_to_g2o_map_trans_.insert(g2o_vertex_bimap::value_type(sensor_it.first->first, dynamic_cast<g2o::HyperGraph::Vertex*>(sensor_vertex)));

        g2o::ParameterSE3Offset* sensor_offset = new g2o::ParameterSE3Offset();
        sensor_offset->setId(optimizer_count_);
        sensor_offset->setOffset();
        optimizer_.addParameter(sensor_offset);

        optimizer_count_++;
        g2o::EdgeSE3Prior *edge_prior = new g2o::EdgeSE3Prior();
        edge_prior->setId(optimizer_count_);
        edge_prior->vertices()[0] = sensor_vertex;
        edge_prior->setMeasurement(graph.sensorInitial(sensor_it.first->first));
        edge_prior->information() = 100. * Eigen::MatrixXd::Identity(6,6);
        edge_prior->information().block<3,3>(0,0) = 100. * Eigen::MatrixXd::Identity(3,3);
        edge_prior->setParameterId(0, optimizer_count_ - 1);
        optimizer_.addEdge(edge_prior);

        optimizer_count_++;
    }

    odom_params_vertex_ = new g2o::VertexOdomParams();
    odom_params_vertex_->setEstimate(graph.odom());
    odom_params_vertex_->setId(optimizer_count_++);
    optimizer_.addVertex(odom_params_vertex_);

    double sum_t_1 = 0.;
    double sum_t_2 = 0.;
    for (auto edge_it = graph.edgeIterator(); edge_it.first != edge_it.second; ++edge_it.first) {
        SlamEdge &edge = edge_it.first->second;
        if (graph.existsNode(edge.id_from_) && graph.existsNode(edge.id_to_)) {
            if (edge.type_ == graph_slam_msgs::Edge::TYPE_2D_WHEEL_ODOMETRY) {
                addOdometryEdge(edge.id_, graph);
                sum_t_1 += (graph.node(edge.id_from_).pose_.inverse() * graph.node(edge.id_to_).pose_).translation().norm();
                sum_t_2 += edge.transform_.translation().norm();
            } else if (edge.valid_) {
                addOdometryEdge(edge.id_, graph);
            }
        }
    }
    std::cout << "graph: " << sum_t_1 << std::endl;
    std::cout << "odom:  " << sum_t_2 << std::endl;
    std::cout << (sum_t_1 / sum_t_2) << std::endl;
}

void SensorTransformOptimizer::storeImpl(SlamGraph &graph)
{
    for (auto sensor_it = graph.sensorIterator(); sensor_it.first != sensor_it.second; ++sensor_it.first) {
        g2o::VertexSE3* sensor_vertex = static_cast<g2o::VertexSE3*>(sensor_id_to_g2o_map_trans_.left.at(sensor_it.first->first));
//        std::cout << sensor_it.first->first << std::endl << sensor_vertex->estimate().matrix() << std::endl << graph.sensorInitial(sensor_it.first->first).matrix() << std::endl;
        Eigen::Vector3d rpy = g2o::internal::toEuler(sensor_vertex->estimate().linear());
//        std::cout << rpy << std::endl;
        graph.sensor(sensor_it.first->first) = sensor_vertex->estimate();
    }

    std::cout << "odom: " << odom_params_vertex_->estimate().transpose() << std::endl;
    graph.odom() = odom_params_vertex_->estimate();
}

void SensorTransformOptimizer::optimizeImpl()
{
    if (!optimizer_.initializeOptimization()) {
        ROS_ERROR("g2o initialization failed.");
        return;
    }

    optimizer_.optimize(100, false);
}

void SensorTransformOptimizer::clear()
{
    optimizer_.clear();
    node_id_to_g2o_map_.clear();
    edge_id_to_g2o_map_.clear();
    sensor_id_to_g2o_map_trans_.clear();
    sensor_id_to_g2o_map_rot_.clear();
    has_gps_measurement_ = false;
    optimizer_count_ = 0;
}

void SensorTransformOptimizer::addVertex(const SlamNode &node)
{
    Eigen::Isometry3d node_pose = node.pose_;
    if (node_id_to_g2o_map_.left.find(node.id_) != node_id_to_g2o_map_.left.end()) {
        ROS_ERROR("node already exists: %s", node.id_.c_str());

        g2o::VertexSE3* pose_vertex = static_cast<g2o::VertexSE3*>(node_id_to_g2o_map_.left.at(node.id_));
        pose_vertex->setEstimate(node_pose);
        pose_vertex->setFixed(true);
    } else {
        g2o::VertexSE3 *pose_vertex =  new g2o::VertexSE3();
        pose_vertex->setId(optimizer_count_);
        pose_vertex->setEstimate(node_pose);
        pose_vertex->setFixed(true);
        optimizer_.addVertex(pose_vertex);
        optimizer_count_++;

        node_id_to_g2o_map_.insert(g2o_vertex_bimap::value_type(node.id_, dynamic_cast<g2o::HyperGraph::Vertex*>(pose_vertex)));
    }
}

void SensorTransformOptimizer::addOdometryEdge(std::string id, SlamGraph &graph)
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

    if (edge.type_ ==  graph_slam_msgs::Edge::TYPE_3D_FULL) {
        g2o::EdgeSE3SensorCalib* edge_sensor = new g2o::EdgeSE3SensorCalib();
        edge_sensor->setId(optimizer_count_);
        edge_sensor->vertices()[0] = node_id_to_g2o_map_.left.at(edge.id_from_);
        edge_sensor->vertices()[1] = node_id_to_g2o_map_.left.at(edge.id_to_);
        edge_sensor->vertices()[2] = sensor_id_to_g2o_map_trans_.left.at(edge.sensor_from_);
        edge_sensor->vertices()[3] = sensor_id_to_g2o_map_trans_.left.at(edge.sensor_to_);
        edge_sensor->setMeasurement(edge.transform_);
        edge_sensor->information() = edge.information_;
        optimizer_.addEdge(edge_sensor);
        edge_id_to_g2o_map_.insert(g2o_edge_bimap::value_type(edge.id_, dynamic_cast<g2o::HyperGraph::Edge*>(edge_sensor)));
    } else if (edge.type_ == graph_slam_msgs::Edge::TYPE_2D_WHEEL_ODOMETRY) {
        g2o::Vector6d meas = g2o::internal::toVectorMQT(edge.transform_);
        g2o::EdgeSE3OdomDifferentialCalib* edge_odom = new g2o::EdgeSE3OdomDifferentialCalib();
        edge_odom->vertices()[0] = node_id_to_g2o_map_.left.at(edge.id_from_);
        edge_odom->vertices()[1] = node_id_to_g2o_map_.left.at(edge.id_to_);
        edge_odom->vertices()[2] = odom_params_vertex_;
        edge_odom->diff_time_ = fabs(edge.diff_time_.toSec());

        edge_odom->setMeasurement(meas);
        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber();
        rk->setDelta(1.);
        edge_odom->setRobustKernel(rk);
        edge_odom->information() = edge.information_;
        optimizer_.addEdge(edge_odom);
    }

    optimizer_count_++;
}
