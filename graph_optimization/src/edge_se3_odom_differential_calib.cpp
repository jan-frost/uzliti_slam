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

#include <graph_optimization/edge_se3_odom_diffrential_calib.h>

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
