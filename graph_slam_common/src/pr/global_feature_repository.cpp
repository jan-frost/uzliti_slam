#include <graph_slam_tools/pr/global_feature_repository.h>

#include <chrono>

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

//    auto start = system_clock::now();
    std::vector<cv::DMatch> dmatch;
    this->knnFeatureSearch(currDesc,dmatch);
//    std::cout << "search: " << (duration_cast<microseconds>(system_clock::now() - start).count() / 1000.) << std::endl;
//    start = system_clock::now();

//    std::vector<int> matchScore;

//    std::vector<int> all_matches;
//    //find nodes which contain features (currDesc)
//    for(unsigned int i = 0; i < dmatch.size(); i++) {
//        //Retrieve index of feature in feature rep (descriptors, links)
//        int ind = dmatch.at(i).imgIdx;
//        all_matches.insert(all_matches.end(), links[ind].begin(), links[ind].end());
//    }
//    std::sort(all_matches.begin(), all_matches.end());
//    int count = 0;
//    int current_val = -1;
//    for (auto val : all_matches) {
//        if (val == current_val) {
//            count++;
//        } else {
//            if (count > min_matches) {
//                //TODO better use lower_bound
//                bool added = false;
//                for (int i = 0; i < matchScore.size(); i++) {
//                    if (matchScore[i] < count) {
//                        matchScore.insert(matchScore.begin() + i, count);
//                        matchingNodes.insert(matchingNodes.begin() + i, current_val);
//                        added = true;
//                        break;
//                    }
//                }
//                if (!added) {
//                    matchScore.push_back(count);
//                    matchingNodes.push_back(current_val);
//                }
//            }

//            current_val = val;
//            count = 1;
//        }
//    }
//    std::cout << "similarity score_new: " << (duration_cast<microseconds>(system_clock::now() - start).count() / 1000.) << std::endl;
//    start = system_clock::now();

    //find nodes which contain features (currDesc)
    for (auto match : dmatch) {
        //Retrieve index of feature in feature rep (descriptors, links)
        for (auto id : links[match.imgIdx]) { //increase count for node if it contains current feature
            matchingNodes[id]++;
        }
    }
//    std::cout << "similarity score: " << (duration_cast<microseconds>(system_clock::now() - start).count() / 1000.) << std::endl;
//    start = system_clock::now();

//    int relevantNodes = 0;
//    for (auto it=matchCount.begin() ; it != matchCount.end(); it++ )
//    { //count nodes which match at least MIN_MATCHES features
//        //this nodes are taken into consideration for position estimation
//        if(it->second > min_matches)
//        { //relevant mapping maps from node number to the index in bestMatches matrix
//            //resp. matchingNodes.at(i) contains the nodeNumber corresponding to
//            //row i in bestMatches. Meaning the local feature desc matching.
//            bool added = false;
//            for (int i = 0; i < matchScore.size(); i++) {
//                if (matchScore[i] < it->second) {
//                    matchScore.insert(matchScore.begin() + i, it->second);
//                    matchingNodes.insert(matchingNodes.begin() + i, it->first);
//                    added = true;
//                    break;
//                }
//            }
//            if (!added) {
//                matchScore.push_back(it->second);
//                matchingNodes.push_back(it->first);
//            }
//            relevantNodes++;
//        }
//    }
//    std::cout << "sort: " << (duration_cast<microseconds>(system_clock::now() - start).count() / 1000.) << std::endl;
//    start = system_clock::now();

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
{//add descriptor to feature rep with desc the descriptor, nodeNumber number of the node containing the descriptor
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

void GlobalFeatureRepository::rebuildMatcher()
{ //add pending descriptors to the matcher and train it.
//    matcher->add(descriptors);
//    descriptors.clear();
//    matcher->train();
}

void GlobalFeatureRepository::addLinks(std::vector<int> featIndex, int nodeNumber, std::vector<int> descPosition)
{
    for(unsigned int i = 0; i < featIndex.size(); i++)
    {
        addLink(featIndex.at(i),nodeNumber,descPosition.at(i));
    }
}

void GlobalFeatureRepository::addLink(int featIndex, int nodeNumber, int descPosition)
{
    links.at(featIndex).push_back(nodeNumber);
}


