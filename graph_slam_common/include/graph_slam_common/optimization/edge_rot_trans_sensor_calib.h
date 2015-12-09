// Copyright (c) 2014, Institute of Computer Engineering (ITI), Universität zu Lübeck
// Jan Frost, Jan Helge Klüssendorff
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice, this
//    list of conditions and the following disclaimer in the documentation and/or
//    other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its contributors may
//    be used to endorse or promote products derived from this software without
//    specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
// IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
// NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

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
