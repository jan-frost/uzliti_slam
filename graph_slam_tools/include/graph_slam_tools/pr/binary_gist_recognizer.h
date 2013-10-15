#ifndef BINARY_GIST_RECOGNIZER
#define BINARY_GIST_RECOGNIZER

#include <graph_slam_tools/pr/place_recognizer.h>
#include <graph_slam_tools/lsh.h>

class BinaryGistRecognizer : public PlaceRecognizer
{
public:
    BinaryGistRecognizer();
    ~BinaryGistRecognizer();

protected:
    void clearImpl();
    void addPlaceImpl(std::string id, std::vector<SensorDataPtr> data);
    std::vector<std::string> searchImpl(std::string id, std::vector<SensorDataPtr> data);

    boost::shared_ptr<lsh::LshMatcher> matcher_;
    int place_count_;
    std::map<int, std::string> place_id_map_;
};

#endif
