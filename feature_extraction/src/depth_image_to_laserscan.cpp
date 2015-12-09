// Copyright (c) 2014, Institute of Computer Engineering (ITI), Universität zu Lübeck
// Jan Frost
// All rights reserved.
//
// This file is part of the uzliti_slam ROS package.
//
// uzliti_slam is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// uzliti_slam is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with uzliti_slam.  If not, see <http://www.gnu.org/licenses/>.

#include <feature_extraction/feature_extraction_service_node.h>

#include <graph_slam_common/conversions.h>
#include <map_projection/graph_grid_mapper.h>

GraphGridMapper mapper;
map_projection::OccupancyGridProjectorConfig conf_;

void occupancyGridConfigCallback(map_projection::OccupancyGridProjectorConfig &config, uint32_t level)
{
    ROS_ERROR("test %f", config.min_height);
    mapper.setConfig(config);
}

int main(int argc, char** argv)
{    
    ros::init(argc, argv, "depth_image_to_laserscan");
    ros::NodeHandle nh("~");

    std::vector<RGBDCameraListernerPtr> rgbd_listeners;
    std::vector<std::string> cams;
    std::vector<std::string> cams_default;
    cams_default.push_back("camera");
    nh.param<std::vector<std::string>>("cams", cams, cams_default);
    ROS_INFO("Using the following cameras:");
    for (auto cam : cams) {
        ROS_INFO("Added camera '%s'", cam.c_str());
        rgbd_listeners.push_back(RGBDCameraListernerPtr(new RGBDCameraListerner(nh, "/" + cam)));
    }

    double depth_scale = 1.0;
    nh.param<double>("depth_scale", depth_scale, 1.0);

    std::string odom_frame = "/odom";
    nh.param<std::string>("odom_frame", odom_frame, "/odom");

    ros::Publisher pub = nh.advertise<sensor_msgs::LaserScan>("/scan", 10);
    tf::TransformListener tfl;

    dynamic_reconfigure::Server<map_projection::OccupancyGridProjectorConfig> server_og(nh);
    dynamic_reconfigure::Server<map_projection::OccupancyGridProjectorConfig>::CallbackType f_og;
    f_og = boost::bind(&occupancyGridConfigCallback, _1, _2);
    server_og.setCallback(f_og);

    ros::Rate rate(10);
    while (ros::ok()) {
        bool first = true;
        bool all_new = true;

        if (rgbd_listeners.size() == 1) {
            sensor_msgs::Image depth_img_msg;
            sensor_msgs::Image color_img_msg;
            sensor_msgs::CameraInfo camera_model_msg;
            if (rgbd_listeners[0]->getRgbdImage(color_img_msg, depth_img_msg, camera_model_msg)) {
                image_geometry::PinholeCameraModel camera_model;
                camera_model.fromCameraInfo(camera_model_msg);

                cv_bridge::CvImagePtr depth_bridge;
                cv::Mat depth_img;
                try {
                    depth_bridge = cv_bridge::toCvCopy(depth_img_msg, sensor_msgs::image_encodings::TYPE_32FC1);
                    depth_img = depth_bridge->image;
                    if (depth_img_msg.encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
                        depth_img = 0.001 * depth_img;
                    }
                    if (depth_scale != 1.) {
                        depth_img *= depth_scale;
                    }

                    Eigen::Isometry3d odom_transform = Eigen::Isometry3d::Identity();
                    tfl.waitForTransform("/base_footprint", odom_frame, depth_img_msg.header.stamp, ros::Duration(0.5));
                    if (Conversions::getTransform(tfl, odom_transform, "/base_footprint", odom_frame, depth_img_msg.header.stamp)) {
                        Eigen::Vector3d rpy = g2o::internal::toEuler(odom_transform.linear());
                        rpy(2) = 0;
                        odom_transform.linear() = g2o::internal::fromEuler(rpy);
                        odom_transform.translation() = Eigen::Vector3d::Zero();
                    } else {
                        ROS_WARN("failed to get odom transform");
                    }

                    // Get camera transform from TF (latest transform).
                    Eigen::Isometry3d camera_transform;
                    tfl.waitForTransform("/base_footprint", depth_img_msg.header.frame_id, depth_img_msg.header.stamp, ros::Duration(0.5));
                    if (!Conversions::getTransform(tfl, camera_transform, "/base_footprint", depth_img_msg.header.frame_id, depth_img_msg.header.stamp)) {
                        ROS_WARN("feature_extaction: could not get camera transform");

                        rate.sleep();
                        ros::spinOnce();
                        continue;
                    }
                    camera_transform = odom_transform * camera_transform;

                    sensor_msgs::LaserScan scan;
                    scan.header = depth_img_msg.header;
                    mapper.extractImageLaserLine(depth_img, camera_transform, camera_model, scan);
                    pub.publish(scan);
                } catch (cv_bridge::Exception& e) {
                    ROS_ERROR_ONCE("depth cv_bridge exception (encoding %s):\n %s", depth_img_msg.encoding.c_str(), e.what());
                }
            }
        } else {
            sensor_msgs::LaserScan combined_scan;
            Eigen::Isometry3d first_transform;
            for (auto listener : rgbd_listeners) {
                sensor_msgs::Image depth_img_msg;
                sensor_msgs::Image color_img_msg;
                sensor_msgs::CameraInfo camera_model_msg;
                if (!listener->getRgbdImage(color_img_msg, depth_img_msg, camera_model_msg)) {
                    all_new = false;
                    break;
                }
                image_geometry::PinholeCameraModel camera_model;
                camera_model.fromCameraInfo(camera_model_msg);

                cv_bridge::CvImagePtr depth_bridge;
                cv::Mat depth_img;
                try {
                    depth_bridge = cv_bridge::toCvCopy(depth_img_msg, sensor_msgs::image_encodings::TYPE_32FC1);
                    depth_img = depth_bridge->image;
                    if (depth_img_msg.encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
                        depth_img = 0.001 * depth_img;
                    }
                    if (depth_scale != 1.) {
                        depth_img *= depth_scale;
                    }

                    // Get camera transform from TF (latest transform).
                    Eigen::Isometry3d camera_transform;
                    ros::Time t = ros::Time(0);
                    Conversions::getTransform(tfl, camera_transform, "/base_footprint", depth_img_msg.header.frame_id, t);

                    sensor_msgs::LaserScan scan;
                    scan.header = depth_img_msg.header;
                    mapper.extractImageLaserLine(depth_img, camera_transform, camera_model, scan);

                    if (first) {
                        tfl.waitForTransform("/odom", "/base_footprint", depth_img_msg.header.stamp, ros::Duration(0.1));
                        if (!Conversions::getTransform(tfl, first_transform, "/odom", "/base_footprint", depth_img_msg.header.stamp)) {
                            ROS_ERROR("error 1");
                        }
                        combined_scan = scan;
                    } else {
                        first = false;
                        Eigen::Isometry3d current_transform;
                        tfl.waitForTransform("/odom", "/base_footprint", depth_img_msg.header.stamp, ros::Duration(0.1));
                        if (!Conversions::getTransform(tfl, current_transform, "/odom", "/base_footprint", depth_img_msg.header.stamp)) {
                            ROS_ERROR("error 2");
                        }
                        mapper.mergeLaserScans(combined_scan, scan, first_transform.inverse() * current_transform);
                    }
                } catch (cv_bridge::Exception& e) {
                    ROS_ERROR_ONCE("depth cv_bridge exception (encoding %s):\n %s", depth_img_msg.encoding.c_str(), e.what());
                }
            }

            if (all_new) {
                pub.publish(combined_scan);
            }
        }

        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
