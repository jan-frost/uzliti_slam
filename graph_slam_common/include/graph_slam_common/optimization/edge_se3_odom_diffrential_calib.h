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
