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

#ifndef SLAM_EDGE_H
#define SLAM_EDGE_H

#include <vector>
#include <eigen3/Eigen/Dense>
#include <graph_slam_msgs/Edge.h>

class SlamEdge
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SlamEdge();

    SlamEdge(std::string id,
             std::string from_id,
             std::string to_id,
             Eigen::Isometry3d transform,
             Eigen::MatrixXd information,
             unsigned char type,
             std::string sensor_from,
             std::string sensor_to,
             double error = 0.,
             double age = 0.);

    void init(std::string id,
              std::string from_id,
              std::string to_id,
              Eigen::Isometry3d transform,
              Eigen::MatrixXd information,
              unsigned char type,
              std::string sensor_from,
              std::string sensor_to,
              double error = 0.,
              double age = 0.);

    std::string id_;
    std::string id_from_;
    std::string id_to_;
    Eigen::Isometry3d transform_;
    Eigen::Isometry3d displacement_from_;
    Eigen::Isometry3d displacement_to_;
    Eigen::MatrixXd information_;
    unsigned char type_;
    std::string sensor_from_;
    std::string sensor_to_;
    double age_;
    double error_;
    double matching_score_;
};

#endif // GRAPHLINK_H
