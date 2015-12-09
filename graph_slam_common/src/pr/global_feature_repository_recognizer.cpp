#include <graph_slam_tools/pr/global_feature_repository_recognizer.h>

struct CorrespondenceGreater
{
    bool operator()(const std::pair<int,float> &left, const std::pair<int,float> &right) const
    {
        return left.second > right.second;
    }
};

GlobalFeatureRepositoryRecognizer::GlobalFeatureRepositoryRecognizer() : PlaceRecognizer()
{
}

GlobalFeatureRepositoryRecognizer::~GlobalFeatureRepositoryRecognizer() { }

void GlobalFeatureRepositoryRecognizer::clearImpl()
{
    gfr_.clear();
}

std::vector<int> GlobalFeatureRepositoryRecognizer::searchAndAddPlaceImpl(std::string id, std::vector<SensorDataPtr> data)
{
    std::vector<int> res;

    for (auto d : data) {
        if (d->type_ == graph_slam_msgs::SensorData::SENSOR_TYPE_FEATURE) {
            FeatureDataPtr feature_data = boost::dynamic_pointer_cast<FeatureData>(d);
            cv::Mat descriptors = feature_data->features_;

            std::vector<int> all_matches(place_count_ + 1, 0);
            gfr_.match(descriptors, lmnDescInRep, all_matches, feature_data->feature_type_, config_.T);

            // Get all matches that have more than T similarity.
            std::vector<std::pair<int, float> > matches;
            for (int i = 0; i < (int)all_matches.size(); i++) {
                if (all_matches[i] > 0) {
                    if (all_matches[i] >= config_.T) {
                        matches.push_back(std::make_pair(i, (float)all_matches[i]));
                    }
                }
            }
            ROS_DEBUG_NAMED("place recognition", "%d matches", (int) matches.size());

            if (matches.size() > 0) {
                // Sort matches.
                res.reserve(matches.size());
                std::sort(matches.begin(), matches.end(), CorrespondenceGreater());
                for (auto match : matches) {
                    std::cout << "(" << match.first << "," << match.second << ") ";
                    res.push_back(match.first);
                }
            }
            std::cout << std::endl;

            //integrate node into GFR
            for (unsigned int i = 0; i < lmnDescInRep.size(); i++) {
                if (lmnDescInRep(i) == -1) { //add new desc to feature rep);
                    gfr_.addDescriptor(descriptors.row(i), place_count_, i);
                } else { //update link to existing desc in feat rep
                    gfr_.addLink(lmnDescInRep(i), place_count_, i);
                }
            }
        }
    }

    return res;
}

void GlobalFeatureRepositoryRecognizer::addPlaceImpl(std::string id, std::vector<SensorDataPtr> data)
{
    //TODO ignore, if place is already there.

    // Scan for features.
    for (auto d : data) {
        if (d->type_ == graph_slam_msgs::SensorData::SENSOR_TYPE_FEATURE) {
            ROS_DEBUG("add place for node %s", id.c_str());
            FeatureDataPtr feature_data = boost::dynamic_pointer_cast<FeatureData>(d);
            cv::Mat descriptors = feature_data->features_;

            //search for matching in GFR
            std::vector<int> potNeighbors;
            gfr_.match(descriptors, lmnDescInRep, potNeighbors, feature_data->feature_type_);

            //integrate node into GFR
            for (unsigned int i = 0; i < lmnDescInRep.size(); i++) {
                if (lmnDescInRep(i) == -1)
                { //add new desc to feature rep);
                    gfr_.addDescriptor(descriptors.row(i), place_count_, i);
                } else { //update link to existing desc in feat rep
                    gfr_.addLink(lmnDescInRep(i), place_count_, i);
                }
            }
//            gfr_.rebuildMatcher();
        }
    }
}

std::vector<int> GlobalFeatureRepositoryRecognizer::searchImpl(std::string id, std::vector<SensorDataPtr> data)
{
    std::vector<int> res;

    for (auto d : data) {
        if (d->type_ == graph_slam_msgs::SensorData::SENSOR_TYPE_FEATURE) {
            FeatureDataPtr feature_data = boost::dynamic_pointer_cast<FeatureData>(d);
            cv::Mat descriptors = feature_data->features_;

            std::vector<int> all_matches(place_count_ + 1, 0);
            gfr_.match(descriptors, lmnDescInRep, all_matches, feature_data->feature_type_, config_.T);

            // Get all matches that have more than T similarity.
            std::vector<std::pair<int, float> > matches;
            for (int i = 0; i < (int)all_matches.size(); i++) {
                if (all_matches[i] > 0) {
                    if (all_matches[i] >= config_.T) {
                        matches.push_back(std::make_pair(i, (float)all_matches[i]));
                    }
                }
            }
            ROS_DEBUG_NAMED("place recognition", "%d matches", (int) matches.size());

            if (matches.size() > 0) {
                // Sort matches.
                res.reserve(matches.size());
                std::sort(matches.begin(), matches.end(), CorrespondenceGreater());
                for (auto match : matches) {
                    res.push_back(match.first);
                }
            }
        }
    }

    return res;
}

void GlobalFeatureRepositoryRecognizer::removePlaceImpl(std::string id, std::vector<SensorDataPtr> data)
{
    //TODO remove from GFR.
}
