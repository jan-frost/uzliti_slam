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

#ifndef G2O_EDGE_SE3_ODOM_CALIB_DIFFERENTIAL_H
#define G2O_EDGE_SE3_ODOM_CALIB_DIFFERENTIAL_H

#include <g2o/types/sclam2d/g2o_types_sclam2d_api.h>
#include <g2o/types/sclam2d/odometry_measurement.h>
#include <g2o/types/sclam2d/vertex_odom_differential_params.h>
#include <g2o/types/sclam2d/edge_se2_odom_differential_calib.h>

#include <g2o/types/slam2d/vertex_se2.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/core/base_multi_edge.h>

using namespace Eigen;
namespace g2o {


class G2O_TYPES_SCLAM2D_API VertexOdomParams: public BaseVertex <6, Vector6d> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    VertexOdomParams() {

    }

    virtual void setToOriginImpl() {
      _estimate << 1., 1., 1., 0., 0., 0.;
    }

    virtual void oplusImpl(const double* v) {
      for (int i=0; i<6; i++)
        _estimate(i) += v[i];
    }

    virtual bool read(std::istream& is) {
        return true;
    }

    virtual bool write(std::ostream& os) const {
        return true;
    }
};

class G2O_TYPES_SCLAM2D_API EdgeSE3OdomDifferentialCalib : public BaseMultiEdge<6, Vector6d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeSE3OdomDifferentialCalib();

    void computeError();

    static Eigen::Quaterniond extrapolate(Eigen::Quaterniond a, Eigen::Quaterniond b, double t)
    {
        Eigen::Quaterniond res = a;
        while (t >= 1) {
            res = b * res;
            t -= 1;
        }
        res = a.slerp(t, b) * res;
        return res;
    }

    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    double diff_time_;
};

} // end namespace

#endif
