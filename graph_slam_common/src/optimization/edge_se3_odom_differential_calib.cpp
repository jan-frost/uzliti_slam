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

#include <graph_slam_tools/optimization/edge_se3_odom_diffrential_calib.h>

namespace g2o {

  EdgeSE3OdomDifferentialCalib::EdgeSE3OdomDifferentialCalib() :
    BaseMultiEdge<6, Vector6d>()
  {
    resize(3);
    diff_time_ = 1.;
  }

  void EdgeSE3OdomDifferentialCalib::computeError()
  {
      const VertexSE3* v1                        = dynamic_cast<const VertexSE3*>(_vertices[0]);
      const VertexSE3* v2                        = dynamic_cast<const VertexSE3*>(_vertices[1]);
      const VertexOdomParams* params = dynamic_cast<const VertexOdomParams*>(_vertices[2]);
      const Isometry3d& x1                              = v1->estimate();
      const Isometry3d& x2                              = v2->estimate();

      Isometry3d odom = internal::fromVectorMQT(measurement());
      Eigen::Vector3d rpy = g2o::internal::toEuler(odom.linear());

      //        g2o::MotionMeasurement mma(odom.translation()(0), odom.translation()(1), rpy(2), diff_time_);
      //        g2o::VelocityMeasurement vel = g2o::OdomConvert::convertToVelocity(mma);
      //        vel.setVl(params->estimate()(0) * vel.vl());
      //        vel.setVr(params->estimate()(1) * vel.vr());
      //        g2o::MotionMeasurement mmb = g2o::OdomConvert::convertToMotion(vel, params->estimate()(2));
      //        odom.translation()(0) = mmb.x();
      //        odom.translation()(1) = mmb.y();
      //        rpy(2) = mmb.theta();
      //        odom.linear() = g2o::internal::fromEuler(rpy);

      // move to cpp file
      // implement translation scale
      // implement rotation scale, which affects odometry translation

      double drift_theta = params->estimate()(1) * fabs(rpy(2));
      double drift_trans = params->estimate()(2) * odom.translation().norm();
      Eigen::AngleAxisd drift_rot(drift_theta + drift_trans, Eigen::Vector3d::UnitZ());
      odom.translation() = params->estimate()(0) * (drift_rot * odom.translation());
      rpy(2) += drift_theta + drift_trans;
      odom.linear() = g2o::internal::fromEuler(rpy);

      Isometry3d delta = x2.inverse() * x1 * odom;
      _error = internal::toVectorMQT(delta);
  }

  bool EdgeSE3OdomDifferentialCalib::read(std::istream& is)
  {
    return true;
  }

  bool EdgeSE3OdomDifferentialCalib::write(std::ostream& os) const
  {
      return true;
  }

} // end namespace
