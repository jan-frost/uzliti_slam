// Copyright (c) 2015, Institute of Computer Engineering (ITI), Universität zu Lübeck
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

#include <graph_slam_common/slam_edge.h>

SlamEdge::SlamEdge()
{
    init("", "", "", Eigen::Isometry3d::Identity(), Eigen::MatrixXd::Identity(6,6), 0, "", "");
}

SlamEdge::SlamEdge(std::string id, std::string from_id, std::string to_id, Eigen::Isometry3d transform, Eigen::MatrixXd information, unsigned char type, std::string sensor_from, std::string sensor_to, ros::Duration diff_time, double error, double age)
{
    init(id, from_id, to_id, transform, information, type, sensor_from, sensor_to, diff_time, error, age);
}

void SlamEdge::init(std::string id, std::string from_id, std::string to_id, Eigen::Isometry3d transform, Eigen::MatrixXd information, unsigned char type, std::string sensor_from, std::string sensor_to, ros::Duration diff_time, double error, double age)
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
    this->diff_time_ = diff_time;
    displacement_from_ = Eigen::Isometry3d::Identity();
    displacement_to_ = Eigen::Isometry3d::Identity();
    valid_ = false;
}
