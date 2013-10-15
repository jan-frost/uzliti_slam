#ifndef OCCUPANCY_GRID_PROJECTOR_H
#define OCCUPANCY_GRID_PROJECTOR_H

#include <graph_slam_tools/projection/map_projector.h>

#include <dynamic_reconfigure/server.h>
#include <graph_slam_tools/OccupancyGridProjectorConfig.h>
#include <graph_slam_tools/projection/graph_grid_mapper.h>
#include <ros/ros.h>

class OccupancyGridProjector : public MapProjector
{
public:
    OccupancyGridProjector(ros::NodeHandle nh);
    ~OccupancyGridProjector();
    void setConfig(graph_slam_tools::OccupancyGridProjectorConfig config);

protected:
    void projectImpl(SlamGraph &graph);

    ros::Publisher map_pub_;    /**< Publisher for the occupancy grid map. */
    GraphGridMapper mapper;     /**< An auxiliary object to handle the generation of an occupancy grid map */
    graph_slam_tools::OccupancyGridProjectorConfig config_;
};

#endif
