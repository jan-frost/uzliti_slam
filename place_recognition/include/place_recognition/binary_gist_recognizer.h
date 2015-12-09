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

#ifndef BINARY_GIST_RECOGNIZER
#define BINARY_GIST_RECOGNIZER

#include <place_recognition/place_recognizer.h>
#include <flann/flann.h>

typedef flann::Hamming<unsigned char> FlannDistance;
typedef FlannDistance::ElementType FlannElement;
typedef FlannDistance::ResultType FlannDistanceType;

class BinaryGistRecognizer : public PlaceRecognizer
{
public:
    BinaryGistRecognizer();
    ~BinaryGistRecognizer();

protected:
    void clearImpl();
    std::vector<int> searchAndAddPlaceImpl(std::string id, std::vector<SensorDataPtr> data);
    void addPlaceImpl(std::string id, std::vector<SensorDataPtr> data);
    std::vector<int> searchImpl(std::string id, std::vector<SensorDataPtr> data);
    void removePlaceImpl(std::string id, std::vector<SensorDataPtr> data);

    flann::LshIndexParams flann_params_;
    flann::Index<FlannDistance> flann_matcher_;
    std::map<int,int> multiple_desc_map_;
    int local_place_count_;
    std::vector<cv::Mat> descriptors_;
};

#endif
