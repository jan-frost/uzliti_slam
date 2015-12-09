#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <boost/algorithm/string/case_conv.hpp>
#include <boost/filesystem.hpp>
#include <sensor_msgs/image_encodings.h>

using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "generate_fabmap_vocabulary");

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
    std::random_shuffle(files.begin(), files.end());

    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::Image>("/stereo/left/image_raw", 1);

    ros::Rate r(10);
    cout << "Publishing image..." << endl;
    for (size_t i = 0; i < files.size(); i += 1) {
        std::cout << "publish image: " << files[i] << std::endl;
        cv::Mat image = cv::imread(files[i], 0);
        cv_bridge::CvImage cv_ptr;
        cv_ptr.encoding = sensor_msgs::image_encodings::MONO8;
        cv_ptr.header.seq = i;
        cv_ptr.image = image;
        sensor_msgs::Image img_msg = *cv_ptr.toImageMsg();

        pub.publish(img_msg);

        r.sleep();
        ros::spinOnce();
    }

    return 0;
}
