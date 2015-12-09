#ifndef FASTFUSION_PROJECTOR_H
#define FASTFUSION_PROJECTOR_H

#include <graph_slam_tools/projection/map_projector.h>

#include <ros/ros.h>

class FastFusionProjector : public MapProjector
{
public:
    FastFusionProjector(ros::NodeHandle nh);
    ~FastFusionProjector();

protected:
    void projectImpl(SlamGraph &graph);
};

#endif
