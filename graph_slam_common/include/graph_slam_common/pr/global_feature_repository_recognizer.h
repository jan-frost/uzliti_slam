#ifndef GLOBAL_FEATURE_REPOSITORY_RECOGNIZER_H
#define GLOBAL_FEATURE_REPOSITORY_RECOGNIZER_H

#include <graph_slam_tools/pr/place_recognizer.h>
#include <graph_slam_tools/pr/global_feature_repository.h>

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

//    std::map<int, std::string> place_id_map_;
    GlobalFeatureRepository gfr_;
    Eigen::VectorXi lmnDescInRep;
//    int place_count_;
};

#endif
