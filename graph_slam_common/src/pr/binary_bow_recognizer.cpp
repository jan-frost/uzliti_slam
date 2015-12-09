#include <graph_slam_tools/pr/binary_bow_recognizer.h>

BinaryBowRecognizer::BinaryBowRecognizer() :
    PlaceRecognizer(),
    voc_("/home/jan/Data/voc/voc.yml.gz"),
    db_(voc_, false, 0)
{
    local_place_count_ = 0;
}

BinaryBowRecognizer::~BinaryBowRecognizer() { }

void BinaryBowRecognizer::convert(const cv::Mat &descriptor, boost::dynamic_bitset<> &desc_conv)
{
    desc_conv.resize(8 * descriptor.cols);
    for (int i = 0; i < descriptor.cols; i++) {
        unsigned char byte = descriptor.at<unsigned char>(0,i);
        for (int j = 0; j < 7; j++) {
            if (byte & (1 << j)) {
                desc_conv.set(8 * i + j);
            }
        }
    }
}

void BinaryBowRecognizer::clearImpl()
{
}

std::vector<int> BinaryBowRecognizer::searchAndAddPlaceImpl(string id, std::vector<SensorDataPtr> data)
{
    std::vector<int> res;

    for (auto d : data) {
        if (d->type_ == graph_slam_msgs::SensorData::SENSOR_TYPE_FEATURE) {
            ROS_DEBUG("add place for node %s", id.c_str());

            // Convert to DBoW descriptor.
            FeatureDataPtr feature_data = boost::dynamic_pointer_cast<FeatureData>(d);
            std::vector<boost::dynamic_bitset<> > features;
            for (int i = 0; i < feature_data->features_.rows; i++) {
                boost::dynamic_bitset<> feature;
                convert(feature_data->features_.row(i), feature);
                features.push_back(feature);
            }

            // Compute bag-of-words vector.
            DBoW2::BowVector bow;
            voc_.transform(features, bow);

            // Query for descriptor
            DBoW2::QueryResults ret;
            db_.query(bow, ret, config_.k_nearest_neighbors);

            for (auto result: ret) {
                if (result.Score >= config_.T && place_id_map_.left.find(result.Id) != place_id_map_.left.end()) {
                    res.push_back(result.Id);
                }
            }

            // Add descriptor.
            db_.add(bow);
        }
    }

    return res;
}

void BinaryBowRecognizer::addPlaceImpl(std::string id, std::vector<SensorDataPtr> data)
{
    // Scan for gist descriptor.
    for (auto d : data) {
        if (d->type_ == graph_slam_msgs::SensorData::SENSOR_TYPE_FEATURE) {
            ROS_DEBUG("add place for node %s", id.c_str());

            // Convert to DBoW descriptor.
            FeatureDataPtr feature_data = boost::dynamic_pointer_cast<FeatureData>(d);
            std::vector<boost::dynamic_bitset<> > features;
            for (int i = 0; i < feature_data->features_.rows; i++) {
                boost::dynamic_bitset<> feature;
                convert(feature_data->features_.row(i), feature);
                features.push_back(feature);
            }

            // Add descriptor to matcher.
            DBoW2::BowVector bow;
            voc_.transform(features, bow);
            db_.add(bow);
        }
    }
}

std::vector<int> BinaryBowRecognizer::searchImpl(std::string id, std::vector<SensorDataPtr> data)
{
    std::vector<int> res;

    // Scan for gist descriptor.
    for (auto d : data) {
        if (d->type_ == graph_slam_msgs::SensorData::SENSOR_TYPE_FEATURE) {
            // Convert to DBoW descriptor.
            FeatureDataPtr feature_data = boost::dynamic_pointer_cast<FeatureData>(d);
            std::vector<boost::dynamic_bitset<> > features;
            for (int i = 0; i < feature_data->features_.rows; i++) {
                boost::dynamic_bitset<> feature;
                convert(feature_data->features_.row(i), feature);
                features.push_back(feature);
            }

            // Query for descriptor
            DBoW2::QueryResults ret;
            db_.query(features, ret, config_.k_nearest_neighbors);

            for (auto result: ret) {
                if (result.Score >= config_.T && place_id_map_.left.find(result.Id) != place_id_map_.left.end()) {
                    res.push_back(result.Id);
                }
            }
        }
    }

    return res;
}

void BinaryBowRecognizer::removePlaceImpl(std::string id, std::vector<SensorDataPtr> data)
{
    //TODO implement removal.
}
