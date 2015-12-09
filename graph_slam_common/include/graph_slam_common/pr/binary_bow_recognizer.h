#ifndef BINARY_BOW_RECOGNIZER
#define BINARY_BOW_RECOGNIZER

#include <graph_slam_tools/pr/place_recognizer.h>
#include <flann/flann.h>
#include "DBoW2.h"

class BinaryBowRecognizer : public PlaceRecognizer
{
public:
    BinaryBowRecognizer();
    ~BinaryBowRecognizer();

protected:
    void convert(const cv::Mat &descriptor, boost::dynamic_bitset<> &desc_conv);
    void clearImpl();
    std::vector<int> searchAndAddPlaceImpl(std::string id, std::vector<SensorDataPtr> data);
    void addPlaceImpl(std::string id, std::vector<SensorDataPtr> data);
    std::vector<int> searchImpl(std::string id, std::vector<SensorDataPtr> data);
    void removePlaceImpl(std::string id, std::vector<SensorDataPtr> data);


    BriefVocabulary voc_;
    BriefDatabase db_;
    int local_place_count_;
    std::vector<cv::Mat> descriptors_;
};

#endif
