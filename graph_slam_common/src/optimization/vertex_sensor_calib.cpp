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

#include <graph_slam_tools/g2o_types/vertex_sensor_calib.h>

namespace g2o {

VertexSensorCalib::VertexSensorCalib() :
    BaseVertex<5, Eigen::Isometry3d>(),
    _numOplusCalls(0)
{
    setToOriginImpl();
    updateCache();
}

Vector6d VertexSensorCalib::from5d(Vector5d v5)
{
    Vector6d v6;
    v6(0) = v5(0);
    v6(1) = v5(1);
    v6(3) = v5(2);
    v6(4) = v5(3);
    v6(5) = v5(4);
    v6(2) = 0;
    return v6;
}

void VertexSensorCalib::to5d(Vector6d v6, Vector5d &v5)
{
    v5(0) = v6(0);
    v5(1) = v6(1);
    v5(2) = v6(3);
    v5(3) = v6(4);
    v5(4) = v6(5);
}

bool VertexSensorCalib::read(std::istream& is)
{
    Vector6d est;
    for (int i = 0; i < 6; i++)
        is  >> est[i];
    setEstimate(internal::fromVectorMQT(est));
    return true;
}

bool VertexSensorCalib::write(std::ostream& os) const
{
    Vector6d est=internal::toVectorMQT(_estimate);
    for (int i = 0; i < 6; i++)
        os << est[i] << " ";
    return os.good();
}
}
