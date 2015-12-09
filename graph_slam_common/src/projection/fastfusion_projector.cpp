#include <graph_slam_tools/projection/fastfusion_projector.h>
#include <nav_msgs/OccupancyGrid.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <fstream>
#include <graph_slam_tools/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

FastFusionProjector::FastFusionProjector(ros::NodeHandle nh)
{
}

FastFusionProjector::~FastFusionProjector()
{
}

void FastFusionProjector::projectImpl(SlamGraph &graph)
{
    int i = 0;
    std::string base = "/home/jan/Data/temp/";
    // Save keyframes to disc.
    for (auto node_it = graph.nodeIterator(); node_it.first != node_it.second; ++node_it.first) {
        const SlamNode &node = node_it.first->second;

        std::vector<SensorDataPtr> sensor_data = node.sensor_data_;

        for (auto data : sensor_data) {
            if (data->type_ == graph_slam_msgs::SensorData::SENSOR_TYPE_DEPTH_IMAGE) {
                DepthImageDataPtr depth_data = boost::dynamic_pointer_cast<DepthImageData>(data);
                cv::Mat depth_img;
                cv::Mat color_img;

                cv_bridge::CvImagePtr depth_bridge;
                cv_bridge::CvImagePtr rgb_bridge;
                try {
                    depth_bridge = cv_bridge::toCvCopy(depth_data->depth_image_, sensor_msgs::image_encodings::TYPE_32FC1);
                    depth_img = depth_bridge->image;
                    rgb_bridge = cv_bridge::toCvCopy(depth_data->color_image_, sensor_msgs::image_encodings::BGR8);
                    color_img = rgb_bridge->image;

                    cv::Mat depth_img_scaled(depth_img.rows, depth_img.cols, CV_16UC1);
                    for(int v = 0; v < depth_img.rows; ++v) {
                        for(int u = 0; u < depth_img.cols; ++u) {
                            depth_img_scaled.at<short>(v,u) = 5000. * depth_img.at<float>(v,u);
                        }
                    }

                    // Store color image
                    std::stringstream color_file;
                    color_file << "rgb" << std::setfill('0') << std::setw(6) << i << ".png";
                    cv::imwrite(base + color_file.str(), color_img);
                    std::cout << color_file.str() << std::endl;

                    // Store depth image.
                    std::stringstream depth_file;
                    depth_file << "depth" << std::setfill('0') << std::setw(6) << i << ".png";
                    cv::imwrite(base + depth_file.str(), depth_img_scaled);

                    // Append to text file
                    Eigen::Isometry3d T = node.pose_ * depth_data->displacement_ * graph.sensor(depth_data->sensor_frame_);
                    Eigen::Quaterniond quat(T.rotation());
                    Eigen::Vector3d pos(T.translation());

                    std::ofstream of;
                    of.open(base + "acc.txt", std::ios_base::app);
                    of << std::setprecision(16) << node.stamps_.front().toSec() << " ";
                    of << pos(0) << " " << pos(1) << " " << pos(2) << " ";
                    of << quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w() << " ";
                    of << depth_data->depth_image_.header.stamp.toSec() << " " << depth_file.str() << " ";
                    of << depth_data->color_image_.header.stamp.toSec() << " " << color_file.str() << std::endl;

                    // Store PCD file for SSA.
                    std::stringstream pcd_file;
                    pcd_file << "cloud" << std::setfill('0') << std::setw(5) << i << ".pcd";
                    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud = Conversions::toPointCloudColor(T, depth_data);
                    pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
                    sor.setInputCloud(cloud);
                    sor.setLeafSize(0.05f, 0.05f, 0.05f);
                    sor.setFilterFieldName("y");
                    sor.setFilterLimits(-1.0, 2.0);
                    sor.setFilterLimitsNegative(false);
                    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>());
                    sor.filter(*cloud_filtered);
                    pcl::io::savePCDFileBinary(base + pcd_file.str(), *cloud_filtered);

                    i++;
                } catch (cv_bridge::Exception& e) {
                    ROS_ERROR("cv_bridge exception: %s",e.what());
                }

            }
        }
    }
}
