// Copyright (c) 2015, Institute of Computer Engineering (ITI), Universität zu Lübeck
// Jan Frost, Jan Helge Klüssendorff
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

#ifndef GLOBAL_FEATURE_REPOSITORY_RECOGNIZER_H
#define GLOBAL_FEATURE_REPOSITORY_RECOGNIZER_H

#include <place_recognition/place_recognizer.h>

#include <place_recognition/global_feature_repository.h>

class GlobalFeatureRepositoryRecognizer : public PlaceRecognizer
{
public:
    GlobalFeatureRepositoryRecognizer();
    ~GlobalFeatureRepositoryRecognizer();

protected:
    void clearImpl();
    std::vector<int> searchAndAddPlaceImpl(std::string id, std::vector<SensorDataPtr> data);
    void addPlaceImpl(std::string id, std::vector<SensorDataPtr> data);
    std::vector<int> searchImpl(std::string id, std::vector<SensorDataPtr> data);
    void removePlaceImpl(std::string id, std::vector<SensorDataPtr> data);

    GlobalFeatureRepository gfr_;
    Eigen::VectorXi lmnDescInRep;
};

#endif
