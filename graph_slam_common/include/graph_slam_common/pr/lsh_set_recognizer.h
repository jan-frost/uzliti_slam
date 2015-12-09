#ifndef LSH_SET_RECOGNIZER_H
#define LSH_SET_RECOGNIZER_H

#include <graph_slam_tools/pr/place_recognizer.h>
#include <graph_slam_tools/pr/lsh_set.h>
#include <chrono>
#include <unordered_map>
#include <google/dense_hash_map>

using std::chrono::duration_cast;
using std::chrono::microseconds;
using std::chrono::system_clock;

union long_long_array_u
{
  uint8_t b[8];
  uint64_t ll;
};

class FastLshTable
{
public:
    FastLshTable (int start_byte, int key_width);

    void add(const uchar *descriptor, int id);

    void match(const uchar *descriptor, std::vector<int> &matches);

    void matchAndAdd(const uchar *descriptor, int id, std::vector<int> &matches);

    void remove(const uchar *descriptor, int id);

    std::unordered_map<u_int64_t, std::vector<int> > table_;
    int start_byte_;
    int key_width_;
};

class FastLshSet
{
public:
    FastLshSet(int key_width = 8);

    void clear();

    int getNumTables();

    void add(cv::Mat &descriptors, int id);

    void match(cv::Mat &descriptors, std::vector<int> &matches);

    void matchAndAdd(cv::Mat &descriptors, int id, std::vector<int> &matches);

    void remove(cv::Mat &descriptors, int id);

    std::vector<boost::shared_ptr<FastLshTable> > tables_;
    int key_width_;
};

class LshSetRecognizer : public PlaceRecognizer
{
public:
    LshSetRecognizer();
    ~LshSetRecognizer();

protected:
    void clearImpl();
    std::vector<int> searchAndAddPlaceImpl(std::string id, std::vector<SensorDataPtr> data);
    void addPlaceImpl(std::string id, std::vector<SensorDataPtr> data);
    std::vector<int> searchImpl(std::string id, std::vector<SensorDataPtr> data);
    void removePlaceImpl(std::string id, std::vector<SensorDataPtr> data);

    bool init_;
    lsh_set::LshSetMatcher matcher_;         /**< Modified LSH structure, which stores node references rather than descriptor references. */
    FastLshSet fast_matcher_;
//    int place_count_;
//    std::map<int, std::string> place_id_map_;
};

#endif
