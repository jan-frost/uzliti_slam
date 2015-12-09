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

#include <graph_optimization/edge_rot_trans_sensor_calib.h>

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
