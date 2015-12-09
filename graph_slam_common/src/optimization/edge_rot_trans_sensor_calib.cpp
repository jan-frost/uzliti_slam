// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <graph_slam_tools/optimization/edge_rot_trans_sensor_calib.h>

namespace g2o {

  EdgeRotTransSensorCalib::EdgeRotTransSensorCalib() :
    BaseMultiEdge<6, Isometry3d>()
  {
      resize(6);
  }

  void EdgeRotTransSensorCalib::initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to)
  {
    (void) to;
    VertexSE3* vi = static_cast<VertexSE3*>(_vertices[0]);
    VertexSE3* vj = static_cast<VertexSE3*>(_vertices[1]);
    VertexSE3* li  = static_cast<VertexSE3*>(_vertices[2]);
    VertexSE3* lj  = static_cast<VertexSE3*>(_vertices[3]);
    if (from.count(li) == 0)
      return;
    if (from.count(vi) == 1) {
      vj->setEstimate(vi->estimate() * li->estimate() * measurement() * lj->estimate().inverse());
    } else {
      vi->setEstimate(vj->estimate() * lj->estimate() * _inverseMeasurement * lj->estimate().inverse());
    }
  }

  bool EdgeRotTransSensorCalib::read(std::istream& is)
  {
    Vector6d p;
    is >> p(0) >> p(1) >> p(2) >> p(3) >> p(4) >> p(5);
    _measurement = internal::fromVectorMQT(p);
    _inverseMeasurement=measurement().inverse();
    for (int i = 0; i < information().rows(); ++i)
      for (int j = i; j < information().cols(); ++j) {
        is >> information()(i, j);
        if (i != j)
          information()(j, i) = information()(i, j);
      }
    return true;
  }

  bool EdgeRotTransSensorCalib::write(std::ostream& os) const
  {
    Vector6d p = internal::toVectorMQT(measurement());
    os << p(0) << " " << p(1) << " " << p(2) << " " << p(3) << " " << p(4) << " " << p(5);
    for (int i = 0; i < information().rows(); ++i)
      for (int j = i; j < information().cols(); ++j)
        os << " " << information()(i, j);
    return os.good();
  }

} // end namespace
