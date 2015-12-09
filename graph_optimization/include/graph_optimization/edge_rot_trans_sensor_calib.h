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

#ifndef G2O_EDGE_ROT_TRANS_SENSOR_CALIB_H
#define G2O_EDGE_ROT_TRANS_SENSOR_CALIB_H

#include <g2o/core/base_multi_edge.h>
#include <g2o/types/slam3d/types_slam3d.h>

namespace g2o {
using namespace Eigen;

  /**
   * \brief Vertex for a quaternion
   */
  class VertexQuaternion : public BaseVertex<4, Eigen::Quaterniond>
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      VertexQuaternion() {}
      virtual bool read(std::istream& is) { return false; }
      virtual bool write(std::ostream& os) const { return false;}

      virtual void setToOriginImpl() { _estimate = Eigen::Quaterniond::Identity(); }

      virtual void oplusImpl(const double* update_) {
        Map<const Eigen::Quaterniond> update(update_);
        Eigen::Quaterniond update_quat = update;
        _estimate.x() += update_quat.x();
        _estimate.y() += update_quat.y();
        _estimate.z() += update_quat.z();
        _estimate.w() += update_quat.w();
        _estimate.normalize();
      }

      virtual bool setEstimateDataImpl(const double* est){
        Map<const Eigen::Quaterniond> _est(est);
        _estimate = _est;
        return true;
      }

      virtual bool getEstimateData(double* est) const{
        Map<Eigen::Quaterniond> _est(est);
        _est = _estimate;
        return true;
      }

      virtual int estimateDimension() const {
        return 4;
      }

      virtual bool setMinimalEstimateDataImpl(const double* est){
        _estimate = Map<const Eigen::Quaterniond>(est);
        return true;
      }

      virtual bool getMinimalEstimateData(double* est) const{
        Map<Eigen::Quaterniond> v(est);
        v = _estimate;
        return true;
      }

      virtual int minimalEstimateDimension() const {
        return 4;
      }

  };

/**
   * \brief scanmatch measurement that also calibrates an offset for the laser
   */
class EdgeRotTransSensorCalib : public BaseMultiEdge<6, Isometry3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeRotTransSensorCalib();

    void computeError()
    {
        const VertexSE3* v1 = static_cast<const VertexSE3*>(_vertices[0]);
        const VertexSE3* v2 = static_cast<const VertexSE3*>(_vertices[1]);
        const Isometry3d& x1 = v1->estimate();
        const Isometry3d& x2 = v2->estimate();

        const VertexPointXYZ* v3 = static_cast<const VertexPointXYZ*>(_vertices[2]);
        const VertexQuaternion* v4 = static_cast<const VertexQuaternion*>(_vertices[3]);
        Isometry3d l1 = Eigen::Isometry3d::Identity();
        l1.translation() = v3->estimate();
        l1.linear() = v4->estimate().toRotationMatrix();

        const VertexPointXYZ* v5 = static_cast<const VertexPointXYZ*>(_vertices[4]);
        const VertexQuaternion* v6 = static_cast<const VertexQuaternion*>(_vertices[5]);
        Isometry3d l2 = Eigen::Isometry3d::Identity();
        l2.translation() = v5->estimate();
        l2.linear() = v6->estimate().toRotationMatrix();

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
