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

#include <place_recognition/global_feature_repository.h>

#include <chrono>
#include <bitset>

using std::chrono::duration_cast;
using std::chrono::microseconds;
using std::chrono::system_clock;

GlobalFeatureRepository::GlobalFeatureRepository() :
    flann_params_(12, 20, 0),
    flann_matcher_(flann_params_)
{
    this->descriptor_ = -1;
}

GlobalFeatureRepository::~GlobalFeatureRepository() {
}

void GlobalFeatureRepository::clear()
{
    links.clear();
    descriptors.clear();
    flann_matcher_ = flann::Index<FlannDistance>(flann_params_);
}

bool GlobalFeatureRepository::match(cv::Mat currDesc, Eigen::VectorXi &matches, std::vector<int> &matchingNodes, int type, int min_matches)
{
    std::cout << "GRF size: " << descriptors.size() << std::endl;
    if (descriptor_ != type) {
        descriptor_ = type;
        clear();
    }

    std::vector<cv::DMatch> dmatch;
    this->knnFeatureSearch(currDesc,dmatch);

    //find nodes which contain features (currDesc)
    for (auto match : dmatch) {
        //Retrieve index of feature in feature rep (descriptors, links)
        for (auto id : links[match.imgIdx]) { //increase count for node if it contains current feature
            matchingNodes[id]++;
        }
    }

    //matches vector correspond to currDesc matrix
    //values are -1 for a unknown feature
    //or the index of the feature in feature rep
    matches = -1 * Eigen::VectorXi::Ones(currDesc.rows);
    for (unsigned int i = 0; i < dmatch.size(); i++)
    {
        //index in currDesc eg the col of bestMatches
        int qInd = dmatch.at(i).queryIdx;
        //index of the feature number stored in featureRep (descriptors, links)
        int iInd = dmatch.at(i).imgIdx;

        matches(qInd) = iInd;
    }
    return true;
}

void GlobalFeatureRepository::knnFeatureSearch(cv::Mat desc, std::vector< cv::DMatch > &dmatch)
{
    int nn = 2;
    flann::Matrix<FlannElement> query(desc.data, desc.rows, desc.cols);
    Matrix<int> indices(new int[query.rows*nn], query.rows, nn);
    Matrix<FlannDistanceType> dists(new FlannDistanceType[query.rows*nn], query.rows, nn);
    flann_matcher_.knnSearch(query, indices, dists, nn, flann::SearchParams(64));
    for (size_t i = 0; i < query.rows; i++) {
        if (indices[i][0] >= 0 && indices[i][0] < (int)descriptors.size() && dists[i][0] < 40 /*&& indices[i][1] < (int)descriptors.size() && dists[i][0] < 0.9 * dists[i][1]*/) {
            cv::DMatch m(i, indices[i][0], indices[i][0], dists[i][0]);
            dmatch.push_back(m);
        }
    }
}

void GlobalFeatureRepository::addDescriptors(cv::Mat newDesc, int nodeNumber, std::vector<int> newDescPosition)
{
        for(unsigned int i = 0; i < newDescPosition.size(); i++)
	{
		addDescriptor(newDesc.row(i).clone(),nodeNumber,newDescPosition.at(i));
	}
}

void GlobalFeatureRepository::addDescriptors(cv::Mat newDesc, int nodeNumber)
{
	for(int i = 0; i < newDesc.rows; i++)
	{
		addDescriptor(newDesc.row(i).clone(),nodeNumber,i);
	}
}

void GlobalFeatureRepository::addDescriptor(cv::Mat desc, int nodeNumber, int descPosition)
{
    //add descriptor to feature rep with desc the descriptor, nodeNumber number of the node containing the descriptor
	//and descPosition the position of the desc in the DESC mat of the node

    int bit_count = 0;
    for (int i = 0; i < desc.cols; i++) {
        std::bitset<8> bits(desc.at<unsigned char>(0,i));
        bit_count += bits.count();
    }

    if (bit_count > 3*desc.cols) {
        descriptors.push_back(desc);
        std::vector<int> vec;
        vec.reserve(8);
        vec.push_back(nodeNumber);
        links.push_back(vec);

        flann::Matrix<FlannElement> flann_mat(desc.data, desc.rows, desc.cols);
        if (flann_matcher_.veclen() == 0) {
            flann_matcher_ = flann::Index<FlannDistance>(flann_mat, flann_params_);
        } else {
            flann_matcher_.addPoints(flann_mat);
        }
    }
}

void GlobalFeatureRepository::addLinks(std::vector<int> featIndex, int nodeNumber, std::vector<int> descPosition)
{
    for(unsigned int i = 0; i < featIndex.size(); i++) {
        addLink(featIndex.at(i),nodeNumber,descPosition.at(i));
    }
}

void GlobalFeatureRepository::addLink(int featIndex, int nodeNumber, int descPosition)
{
    links.at(featIndex).push_back(nodeNumber);
}


