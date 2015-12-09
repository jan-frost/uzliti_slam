#ifndef BINARY_GIST_RECOGNIZER
#define BINARY_GIST_RECOGNIZER

#include <graph_slam_tools/pr/place_recognizer.h>
#include <graph_slam_tools/lsh.h>
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

    my_lsh::LshMatcher lsh_matcher_;
    flann::LshIndexParams flann_params_;
    flann::Index<FlannDistance> flann_matcher_;
    std::map<int,int> multiple_desc_map_;
    int local_place_count_;
    std::vector<cv::Mat> descriptors_;
};

#endif
