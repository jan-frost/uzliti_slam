// Copyright (c) 2014, Institute of Computer Engineering (ITI), Universität zu Lübeck
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

#ifndef G2O_EDGE_SE3_SENSOR_CALIB_H
#define G2O_EDGE_SE3_SENSOR_CALIB_H

#include <g2o/core/base_multi_edge.h>
#include <g2o/types/slam3d/types_slam3d.h>

namespace g2o {
using namespace Eigen;

/**
   * \brief scanmatch measurement that also calibrates an offset for the laser
   */
class EdgeSE3SensorCalib : public BaseMultiEdge<6, Isometry3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeSE3SensorCalib();

    void computeError()
    {
        const VertexSE3* v1 = static_cast<const VertexSE3*>(_vertices[0]);
        const VertexSE3* v2 = static_cast<const VertexSE3*>(_vertices[1]);
        const Isometry3d& x1 = v1->estimate();
        const Isometry3d& x2 = v2->estimate();

        const VertexSE3* v3 = static_cast<const VertexSE3*>(_vertices[2]);
        Isometry3d l1 = v3->estimate();

        const VertexSE3* v4 = static_cast<const VertexSE3*>(_vertices[3]);
        Isometry3d l2 = v4->estimate();

        Isometry3d delta_sensor = _inverseMeasurement * ((x1 * l1).inverse() * x2 * l2);
        _error = internal::toVectorMQT(delta_sensor);
    }

    void setMeasurement(const Isometry3d& m){
        _measurement = m;
        _inverseMeasurement = m.inverse();
    }

    virtual double initialEstimatePossible(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to)
    {
        if (   from.count(_vertices[2]) == 1 && from.count(_vertices[3]) == 1 // need the laser offset
               && ((from.count(_vertices[0]) == 1 && to == _vertices[1]) || ((from.count(_vertices[1]) == 1 && to == _vertices[0])))) {
            return 1.0;
        }
        return -1.0;
    }
    virtual void initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to);

    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

protected:
    Isometry3d _inverseMeasurement;
};

} // end namespace

#endif
