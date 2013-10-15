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

#include <graph_slam_tools/graph/slam_edge.h>

SlamEdge::SlamEdge()
{
    init("", "", "", Eigen::Isometry3d::Identity(), Eigen::MatrixXd::Identity(6,6), 0, "", "");
}

SlamEdge::SlamEdge(std::string id, std::string from_id, std::string to_id, Eigen::Isometry3d transform, Eigen::MatrixXd information, unsigned char type, std::string sensor_from, std::string sensor_to, double error, double age)
{
    init(id, from_id, to_id, transform, information, type, sensor_from, sensor_to, error, age);
}

void SlamEdge::init(std::string id, std::string from_id, std::string to_id, Eigen::Isometry3d transform, Eigen::MatrixXd information, unsigned char type, std::string sensor_from, std::string sensor_to, double error, double age)
{
    this->id_ = id;
    this->id_from_ = from_id;
    this->id_to_ = to_id;
    this->transform_ = transform;
    this->information_ = information;
    this->type_ = type;
    this->sensor_from_ = sensor_from;
    this->sensor_to_ = sensor_to;
    this->error_ = error;
    this->age_ = age;
    displacement_from_ = Eigen::Isometry3d::Identity();
    displacement_to_ = Eigen::Isometry3d::Identity();
}
