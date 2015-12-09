
#include "DBoW2.h"

#include "DUtils.h"
#include "DUtilsCV.h" // defines macros CVXX
#include "DVision.h"
#include <boost/algorithm/string/case_conv.hpp>
#include <boost/filesystem.hpp>

void convert(const cv::Mat &descriptor, boost::dynamic_bitset<> &desc_conv)
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

int main(int argc, char** argv)
{
    vector<vector<boost::dynamic_bitset<> > > features;

    // Loop through dataset.
    std::vector<std::string> files;
    std::vector<std::string> base_dirs;
    base_dirs.push_back("/home/jan/Data/PR/CityCenter/Images/");
    base_dirs.push_back("/home/jan/Data/PR/KITTI/Images/");
    base_dirs.push_back("/home/jan/Data/PR/ITI/Images/");
    base_dirs.push_back("/home/jan/Data/PR/MIT/Images/");
    base_dirs.push_back("/home/jan/Data/PR/NewCollege/Images/");
    for (auto base_dir : base_dirs) {
        for ( boost::filesystem::directory_iterator it( base_dir );
              it != boost::filesystem::directory_iterator(); ++it ) {
            if ( boost::filesystem::is_regular_file( it->status() ) &&
                 boost::algorithm::to_lower_copy( it->path().extension().string() ) == ".jpg" ||
                 boost::algorithm::to_lower_copy( it->path().extension().string() ) == ".png" ) {
                files.push_back( it->path().string() );
            }
        }
    }
    std::sort(files.begin(), files.end());

//    features.clear();
//    features.reserve(files.size());

    cv::ORB orb(300);
    DVision::BRIEF brief;

    cout << "Extracting ORB features..." << endl;
    for (size_t i = 0; i < files.size(); i += 20) {
        std::cout << i << " ";
        cv::Mat image = cv::imread(files[i], 0);
        vector<cv::KeyPoint> keypoints;
        cv::Mat cv_descriptors;
        vector<boost::dynamic_bitset<> > descriptors;

        orb.detect(image, keypoints);
        orb.compute(image, keypoints, cv_descriptors);
        for (int j = 0; j < cv_descriptors.rows; j++) {
            boost::dynamic_bitset<> feature;
            convert(cv_descriptors.row(j), feature);
            descriptors.push_back(feature);
        }
//        brief.compute(image, keypoints, descriptors);

//        features.push_back(vector<vector<boost::dynamic_bitset<> > >());
//        vector<vector<boost::dynamic_bitset<> > > new_features;
//        changeStructure(descriptors, new_features, brief.getDescriptorLengthInBits() / 8);
        std::cout << descriptors.size() << std::endl;
//        features.insert(features.end(), new_features.begin(), new_features.end());
        features.push_back(descriptors);
    }
    std::cout << std::endl;
    cout << "features: " << features.size() << endl;

    // branching factor and depth levels
    const int k = 10;
    const int L = 6;
    const DBoW2::WeightingType weight = DBoW2::TF_IDF;
    const DBoW2::ScoringType score = DBoW2::L1_NORM;

    cout << "Creating a small " << k << "^" << L << " vocabulary..." << endl;
    BriefVocabulary voc(k, L, weight, score);
    voc.create(features);
    std::cout << "... done!" << std::endl;

    std::cout << "Vocabulary information: " << std::endl
    << voc << std::endl << std::endl;

    voc.save("/home/jan/Data/voc/voc.yml.gz");

    return 0;
}
