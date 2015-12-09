#ifndef SENSOR_TRANSFORM_OPTIMIZER_H
#define SENSOR_TRANSFORM_OPTIMIZER_H

#include <graph_slam_tools/optimization/g2o_optimizer.h>
#include <graph_slam_tools/optimization/edge_se3_odom_diffrential_calib.h>

class SensorTransformOptimizer : public G2oOptimizer
{
public:
    SensorTransformOptimizer();
    ~SensorTransformOptimizer();

protected:
    void clear();
    void addGraphImpl(SlamGraph &graph);
    void storeImpl(SlamGraph &graph);
    void optimizeImpl();

    void addVertex(const SlamNode &node);
    void addOdometryEdge(std::string id, SlamGraph &graph);

    g2o_vertex_bimap sensor_id_to_g2o_map_trans_;
    g2o_vertex_bimap sensor_id_to_g2o_map_rot_;
    g2o::VertexOdomParams *odom_params_vertex_;
};

#endif
