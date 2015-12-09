// Copyright (c) 2014, Institute of Computer Engineering (ITI), Universität zu Lübeck
// Jan Frost, Jan Helge Klüssendorff
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice, this
//    list of conditions and the following disclaimer in the documentation and/or
//    other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its contributors may
//    be used to endorse or promote products derived from this software without
//    specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
// IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
// NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <graph_slam_tools/projection/graph_grid_mapper.h>
#include <cv_bridge/cv_bridge.h>
#include <occupancy_grid_utils/OverlayClouds.h>
#include <occupancy_grid_utils/combine_grids.h>
#include <laser_geometry/laser_geometry.h>
#include <graph_slam_tools/conversions.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <chrono>

using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::microseconds;
using std::chrono::system_clock;

GraphGridMapper::GraphGridMapper(){
    init = false;
    last_node_id_ = "";
}

GraphGridMapper::~GraphGridMapper(){}

void GraphGridMapper::mergeLaserScans(const LaserscanDataPtr &a, const LaserscanDataPtr &b)
{
    std::vector<cv::Point2f> poi_a = scanToPoints(a);
    std::vector<cv::Point2f> int_a = scanToIntensities(a);
    std::vector<cv::Point2f> poi_b = scanToPoints(b);
    std::vector<cv::Point2f> int_b = scanToIntensities(b);

    a->displacement_ = Eigen::Isometry3d::Identity();
    for (size_t i = 0; i < a->scan_.ranges.size(); i++) {
        a->scan_.ranges[i] = a->scan_.range_max + 1.0f;
        a->scan_.intensities[i] = 0.0f;
    }

    for (size_t i = 0; i < poi_a.size(); i++) {
        float x = poi_a.at(i).x;
        float y = poi_a.at(i).y;

        auto angle = -std::atan2(-y, x);
        if (angle < a->scan_.angle_min || angle > a->scan_.angle_max) {
            continue;
        }

        int index = (angle - a->scan_.angle_min) / a->scan_.angle_increment;
        auto range = std::sqrt(x*x + y*y);
        a->scan_.ranges[index] = range;
    }

    for (size_t i = 0; i < int_a.size(); i++) {
        float x = int_a.at(i).x;
        float y = int_a.at(i).y;

        auto angle = -std::atan2(-y, x);
        if (angle < a->scan_.angle_min || angle > a->scan_.angle_max) {
            continue;
        }

        int index = (angle - a->scan_.angle_min) / a->scan_.angle_increment;
        auto range = std::sqrt(x*x + y*y);
        a->scan_.intensities[index] = range;
    }

    for (size_t i = 0; i < poi_b.size(); i++) {
        float x = poi_b.at(i).x;
        float y = poi_b.at(i).y;

        auto angle = -std::atan2(-y, x);
        if (angle < a->scan_.angle_min || angle > a->scan_.angle_max) {
            continue;
        }
        int index = (angle - a->scan_.angle_min) / a->scan_.angle_increment;
        auto range = std::sqrt(x*x + y*y);
        if (isnan(a->scan_.ranges[index]) || a->scan_.ranges[index] < a->scan_.range_min || a->scan_.ranges[index] > a->scan_.range_max) {
            a->scan_.ranges[index] = range;
        } else {
            // Use mean, if ranges are close.
            if (std::fabs(a->scan_.ranges[index] - range) < 0.1f) {
                a->scan_.ranges[index] = 0.5f * (a->scan_.ranges[index] + range);
            // Else take newer measurement.
            } else if (b->scan_.header.stamp > a->scan_.header.stamp) {
                a->scan_.ranges[index] = range;
            }
        }
    }

    for (size_t i = 0; i < int_b.size(); i++) {
        float x = int_b.at(i).x;
        float y = int_b.at(i).y;

        auto angle = -std::atan2(-y, x);
        if (angle < a->scan_.angle_min || angle > a->scan_.angle_max) {
            continue;
        }
        int index = (angle - a->scan_.angle_min) / a->scan_.angle_increment;
        auto range = std::sqrt(x*x + y*y);
        if (isnan(a->scan_.intensities[index]) || a->scan_.intensities[index] < a->scan_.range_min) {
            a->scan_.intensities[index] = range;
        } else {
            // Use mean, if ranges are close.
            if (std::fabs(a->scan_.intensities[index] - range) < 0.1f) {
                a->scan_.intensities[index] = 0.5f * (a->scan_.intensities[index] + range);
            // Else take newer measurement.
            } else if (b->scan_.header.stamp > a->scan_.header.stamp) {
                a->scan_.intensities[index] = range;
            }
        }
    }

    a->scan_center_ = scanMean(a->scan_).cast<double>();
}

sensor_msgs::LaserScan GraphGridMapper::mergeLaserScans(const sensor_msgs::LaserScan &a, const sensor_msgs::LaserScan &b, Eigen::Isometry3d displacement)
{
    sensor_msgs::LaserScan scan = a;

    // Make points out of second scans.
    std::vector<cv::Point2d> poi_b;
    std::vector<cv::Point2d> int_b;
    auto deg = b.angle_min;
    for (size_t i = 0; i < b.ranges.size(); i++) {
        auto r = b.ranges[i];
        if (!isnan(r) && r >= scan.range_min && r <= scan.range_max) {
            Eigen::Vector3d point = displacement * Eigen::Vector3d(std::cos(deg) * r, std::sin(deg) * r, 0).homogeneous();
            poi_b.push_back(cv::Point2d(point(0), point(1)));
        }
        r = b.intensities[i];
        if (!isnan(r) && r >= scan.range_min) {
            Eigen::Vector3d point = displacement * Eigen::Vector3d(std::cos(deg) * r, std::sin(deg) * r, 0).homogeneous();
            int_b.push_back(cv::Point2d(point(0), point(1)));
        }
        deg +=scan.angle_increment;
    }

    for (unsigned int i = 0; i < poi_b.size(); i++) {
        float x = poi_b.at(i).x;
        float y = poi_b.at(i).y;

        auto angle = -std::atan2(-y, x);
        if (angle < scan.angle_min || angle > scan.angle_max)
        {
            ROS_DEBUG("rejected for angle %f not in range (%f, %f)\n", angle, scan.angle_min, scan.angle_max);
            continue;
        }
        int index = (angle - scan.angle_min) / scan.angle_increment;
        auto range = std::sqrt(x*x + y*y);
        if (isnan(scan.ranges[index]) ||
                scan.ranges[index] == 0 ||
                scan.ranges[index] > scan.range_max) {
            scan.ranges[index] = range;
        } else {
            // Use mean, if ranges are close.
            if (fabs(scan.ranges[index] - range) < 0.1f) {
                scan.ranges[index] = 0.5 * (scan.ranges[index] + range);
            // Else use
            } else {
                scan.ranges[index] = 0.0f;
            }
        }
    }

    for (unsigned int i = 0; i < int_b.size(); i++) {
        float x = int_b.at(i).x;
        float y = int_b.at(i).y;

        auto angle = -std::atan2(-y, x);
        if (angle < scan.angle_min || angle > scan.angle_max)
        {
            ROS_DEBUG("rejected for angle %f not in range (%f, %f)\n", angle, scan.angle_min, scan.angle_max);
            continue;
        }
        int index = (angle - scan.angle_min) / scan.angle_increment;
        auto range = std::sqrt(x*x + y*y);
        if (isnan(scan.intensities[index]) ||
                scan.intensities[index] == 0 ||
                scan.intensities[index] > scan.range_max) {
            scan.intensities[index] = range;
        } else {
            // Use mean, if ranges are close.
            if (fabs(scan.intensities[index] - range) < 0.1f) {
                scan.intensities[index] = 0.5 * (scan.intensities[index] + range);
            // Else use
            } else if (scan.intensities[index] > range) {
                scan.intensities[index] = 0.f;
            }
        }
    }

    return scan;
}

bool GraphGridMapper::convertDepthImages2Map(SlamGraph &graph, nav_msgs::OccupancyGrid &map, ros::Publisher &mapPub)
{
    std::string map_frame = "/map";

    //setup map dimensions
    nav_msgs::MapMetaData info;
    cv::Point2d gridSize;
    info.origin = getMapOrigin(graph, gridSize);
    info.resolution = config_.resolution;
    info.width = gridSize.x / info.resolution;
    info.height = gridSize.y / info.resolution;

    nav_msgs::OccupancyGrid fake_grid;
    fake_grid.info = info;
    occupancy_grid_utils::OverlayClouds overlay = occupancy_grid_utils::createCloudOverlay(fake_grid, map_frame, 0.1, 10, 1);
    clouds_.clear();

    //compute laser scan like data from depth images and project them onto the map
    std::pair<NodeIterator, NodeIterator> node_it;
    for (node_it = graph.nodeIterator(); node_it.first != node_it.second; node_it.first++) {
        const SlamNode &node = node_it.first->second;

        // Search for depth image.
        int total_points = 0;
        for (auto data : node.sensor_data_) {
            if (data->type_ == graph_slam_msgs::SensorData::SENSOR_TYPE_DEPTH_IMAGE) {
                boost::shared_ptr<DepthImageData> depth_data = boost::dynamic_pointer_cast<DepthImageData>(data);
                sensor_msgs::Image depth_image = depth_data->depth_image_;

                try
                {
                    sensor_msgs::LaserScan laser;
                    Eigen::Isometry3d camera_transform = depth_data->displacement_ * graph.sensor(depth_data->sensor_frame_);
                    image_geometry::PinholeCameraModel camera_model = depth_data->camera_model_;
                    convertDepthImage2LaserScan(depth_image, camera_transform, camera_model, laser);
                    total_points += laser.ranges.size();

                    sensor_msgs::PointCloud fixed_frame_cloud;
                    laser_geometry::LaserProjection projector_;
                    projector_.projectLaser(laser,fixed_frame_cloud);

                    // Construct and save LocalizedCloud
                    CloudPtr loc_cloud(new occupancy_grid_utils::LocalizedCloud());
                    loc_cloud->cloud.points = fixed_frame_cloud.points;
                    loc_cloud->sensor_pose = affineTransformToPoseMsg(node.pose_);
                    loc_cloud->header.frame_id = map_frame;
                    loc_cloud->header.stamp = laser.header.stamp;
                    clouds_.push_back(loc_cloud);

                    if (clouds_.size() > 1 && total_points > 0) {
                        occupancy_grid_utils::addCloud(&overlay, clouds_.back());
                        nav_msgs::OccupancyGridPtr grid = occupancy_grid_utils::getGrid(overlay);
                        grid->header.stamp = ros::Time::now();
                        mapPub.publish(grid);
                    }
                }
                catch (tf::LookupException& e)
                {
                    ROS_WARN("lookup error");
                    return false;
                }
                catch (tf::ExtrapolationException)
                {
                    ROS_ERROR("extrapolation exception");
                    return false;
                }
                catch (tf::ConnectivityException)
                {
                    ROS_ERROR("connectivity exception");
                    return false;
                }
            }
        }
    }

    //retrieve grid
    nav_msgs::OccupancyGridConstPtr grid = occupancy_grid_utils::getGrid(overlay);
    map = *grid;
    return true;
}

bool GraphGridMapper::convertLaserScans2Map(SlamGraph &graph, nav_msgs::OccupancyGrid &map, ros::Publisher &mapPub, Eigen::Isometry3d diff_transform)
{
    std::string map_frame = "/map";

    double diff_translation = diff_transform.translation().norm();
    Eigen::AngleAxisd diff_angleaxis(diff_transform.linear());
    double diff_rotation = fabs(diff_angleaxis.angle()) * 180 / M_PI;
    ROS_DEBUG("map -> odom diff: %f m, %f deg", diff_translation, diff_rotation);

    std::vector<std::string> nodes_to_add;
    if (diff_translation <= 0.5 && diff_rotation < 5 && last_node_id_ != "") {
        nodes_to_add = graph.getNodesAfter(last_node_id_);
    } else {
        // Clear occupancy grid.
        clouds_.clear();

        //setup map dimensions
        nav_msgs::MapMetaData info;
        cv::Point2d gridSize;
        info.origin = getMapOrigin(graph, gridSize);
        info.resolution = config_.resolution;
        info.width = gridSize.x / info.resolution;
        info.height = gridSize.y / info.resolution;

        fake_grid_.info = info;
        overlay_ = occupancy_grid_utils::createCloudOverlay(fake_grid_, map_frame, 0.1, 10, 1);

        for (auto node_it = graph.nodeIterator(); node_it.first != node_it.second; node_it.first++) {
            nodes_to_add.push_back(node_it.first->first);
        }
    }
    if (nodes_to_add.size() > 0) {
        last_node_id_ = nodes_to_add.back();
    }

    for (auto node_id : nodes_to_add) {
        if (graph.existsNode(node_id)) {
            const SlamNode &node = graph.node(node_id);
            auto node_pose = affineTransformToPoseMsg(node.pose_);
            occupancy_grid_utils::addKnownFreePoint(&overlay_, node_pose.position, 0.5);

            // If there is a node off the grid, force clearing in the next iteration.
            if (node.pose_.translation()(0) < fake_grid_.info.origin.position.x + config_.range_max ||
                    node.pose_.translation()(1) < fake_grid_.info.origin.position.y + config_.range_max ||
                    node.pose_.translation()(0) > fake_grid_.info.origin.position.x + fake_grid_.info.width * fake_grid_.info.resolution - config_.range_max ||
                    node.pose_.translation()(1) > fake_grid_.info.origin.position.y + fake_grid_.info.height * fake_grid_.info.resolution - config_.range_max) {
                last_node_id_ = "";
            }
        }
    }

    for (auto node_id : nodes_to_add) {
        if (graph.existsNode(node_id)) {
            const SlamNode &node = graph.node(node_id);

            // Search for laser scan.
            int total_points = 0;
            for (auto data : node.sensor_data_) {
                if (data->type_ == graph_slam_msgs::SensorData::SENSOR_TYPE_LASERSCAN) {
                    boost::shared_ptr<LaserscanData> depth_data = boost::dynamic_pointer_cast<LaserscanData>(data);
                    sensor_msgs::LaserScan laser = depth_data->scan_;
                    laser.range_max = config_.range_max;
                    try
                    {
                        total_points += laser.ranges.size();

                        sensor_msgs::PointCloud fixed_frame_cloud;
                        laser_geometry::LaserProjection projector;
                        projector.projectLaser(laser,fixed_frame_cloud);

                        // Construct and save LocalizedCloud
                        CloudPtr loc_cloud(new occupancy_grid_utils::LocalizedCloud());
                        loc_cloud->cloud.points = fixed_frame_cloud.points;
                        loc_cloud->sensor_pose = affineTransformToPoseMsg(node.pose_ * depth_data->displacement_);
                        loc_cloud->header.frame_id = map_frame;
                        loc_cloud->header.stamp = laser.header.stamp;
                        clouds_.push_back(loc_cloud);

                        if (clouds_.size() > 1 && total_points > 0) {
                            occupancy_grid_utils::addCloud(&overlay_, clouds_.back());
                        }
                    }
                    catch (tf::LookupException& e)
                    {
                        ROS_WARN("lookup error");
                        return false;
                    }
                    catch (tf::ExtrapolationException)
                    {
                        ROS_ERROR("extrapolation exception");
                        return false;
                    }
                    catch (tf::ConnectivityException)
                    {
                        ROS_ERROR("connectivity exception");
                        return false;
                    }
                }
            }
        }
    }

    //retrieve grid
    nav_msgs::OccupancyGridConstPtr grid = occupancy_grid_utils::getGrid(overlay_);
    map = *grid;
    return true;
}

bool GraphGridMapper::convertDepthImage2LaserScan(const sensor_msgs::Image &image, const Eigen::Isometry3d &camera_transform, const image_geometry::PinholeCameraModel &camera_model, sensor_msgs::LaserScan &scan)
{
    cv::Mat depth_image;
    cv_bridge::CvImagePtr depth_bridge;
    try {
        depth_bridge = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::TYPE_32FC1);

        depth_image = depth_bridge->image;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge excepion: %s",e.what());
        return false;
    }

    extractImageLaserLine(depth_image, camera_transform, camera_model, scan);
    return true;
}

void GraphGridMapper::extractImageLaserLine(const cv::Mat &depth_image, const Eigen::Isometry3d &camera_transform, const image_geometry::PinholeCameraModel &camera_model, sensor_msgs::LaserScan &scan)
{
    scan.header.frame_id = "/base_footprint";
    scan.angle_min = -M_PI;
    scan.angle_max = M_PI;
    scan.angle_increment = config_.angle_increment;
    scan.time_increment = 0.0;
    scan.scan_time = 1. / 30.;
    scan.range_min = config_.range_min;
    scan.range_max = config_.range_max;

    uint32_t ranges_size = std::ceil((scan.angle_max - scan.angle_min) / scan.angle_increment);
    scan.ranges.assign(ranges_size, scan.range_max + 1.0f);
    scan.intensities.assign(ranges_size, 0);

    ROS_DEBUG_NAMED("time", "project image to scan");
    auto start = system_clock::now();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = Conversions::toPointCloud(Eigen::Isometry3d::Identity(), camera_model, depth_image);
    ROS_DEBUG_NAMED("time", "  to point cloud: %f ms", 0.001 * duration_cast<microseconds>(system_clock::now() - start).count());
    start = system_clock::now();
    transformPointCloudInPlace(camera_transform, cloud);
    ROS_DEBUG_NAMED("time", "  transform point cloud: %f ms", 0.001 * duration_cast<microseconds>(system_clock::now() - start).count());
    start = system_clock::now();

    for (unsigned int pit = 0; pit < cloud->points.size(); pit++) {
        pcl::PointXYZ &point = cloud->points[pit];

        if (std::isnan(point.z) || point.z < config_.min_height || point.z > config_.max_height) {
            ROS_DEBUG("rejected for nan in point");
            continue;
        }

        auto angle = -std::atan2(-point.y, point.x);
        if (angle < scan.angle_min || angle > scan.angle_max) {
            ROS_DEBUG("rejected for angle %f not in range (%f, %f)\n", angle, scan.angle_min, scan.angle_max);
            continue;
        }

        int index = (angle - scan.angle_min) / scan.angle_increment;
        auto range_sq = point.x*point.x + point.y*point.y;
        if (range_sq < scan.ranges[index] * scan.ranges[index]) {
            scan.ranges[index] = std::sqrt(range_sq);
        }
        if (range_sq > scan.intensities[index] * scan.intensities[index]) {
            scan.intensities[index] = std::sqrt(range_sq);
        }
    }
    ROS_DEBUG_NAMED("time", "  to laser scan: %f ms", 0.001 * duration_cast<microseconds>(system_clock::now() - start).count());
}

void GraphGridMapper::transformPointCloudInPlace(const Eigen::Isometry3d Tr, const pcl::PointCloud<pcl::PointXYZ>::Ptr &pc) {
    Eigen::Isometry3f T = Tr.cast<float>();
    for (unsigned int pit=0; pit < pc->points.size(); pit++) {
        if (!isnan(pc->points[pit].z)) {
            Eigen::Map<Eigen::Vector3f> pt((float*)&pc->points[pit], 3);
            pt = T * pt;
        }
    }
}

cv::Point3d GraphGridMapper::to3D(cv::Point3d &p, const Eigen::Isometry3d &camera_transform, const image_geometry::PinholeCameraModel &camera_model)
{
    int width = camera_model.cameraInfo().width;
    int height = camera_model.cameraInfo().height;

    int u = round(p.x);
    if(u < 0) {
        u = 0;
    } else if (u >= width) {
        u = width -1;
    }
    int v = round(p.y);
    if(v < 0) {
        v = 0;
    } else if (v >= height) {
        v = height - 1;
    }
    cv::Point3d p3D(-1.0,-1.0,std::numeric_limits<double>::infinity());

    if (p.z != 0 && !isnan(p.z))
    {
        p3D.x = (u - camera_model.cx()) * p.z / camera_model.fx();
        p3D.y = (v - camera_model.cy()) * p.z / camera_model.fy();
        p3D.z = p.z;

        Eigen::Vector3d vec(p3D.x, p3D.y, p3D.z);
        vec = camera_transform * vec.homogeneous();
        p3D.x = vec(0);
        p3D.y = vec(1);
        p3D.z = vec(2);
    }

    return p3D;
}

void GraphGridMapper::setConfig(graph_slam_tools::OccupancyGridProjectorConfig config)
{
    this->config_ = config;
}

geometry_msgs::Pose GraphGridMapper::affineTransformToPoseMsg(const Eigen::Isometry3d &pose)
{
    Eigen::Vector3d position = pose.translation();
    Eigen::Quaterniond orientation(pose.linear());
    geometry_msgs::Pose poseMsg;
    poseMsg.position.x = position(0);
    poseMsg.position.y = position(1);
    poseMsg.position.z = position(2);
    poseMsg.orientation.w = orientation.w();
    poseMsg.orientation.x = orientation.x();
    poseMsg.orientation.y = orientation.y();
    poseMsg.orientation.z = orientation.z();
    return poseMsg;
}

geometry_msgs::Pose GraphGridMapper::getMapOrigin(SlamGraph &graph, cv::Point2d &size)
{
    geometry_msgs::Pose origin;
    origin.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0,0.0,0.0);
    origin.position.x = origin.position.y = origin.position.z = 0.0;
    geometry_msgs::Pose boundingBox[4];
    for (unsigned int i = 0; i < 4; i++) {
        boundingBox[i] = affineTransformToPoseMsg(graph.nodeIterator().first->second.pose_);
    }

    std::pair<NodeIterator, NodeIterator> node_it;
    for (node_it = graph.nodeIterator(); node_it.first != node_it.second; node_it.first++) {
        //compute min and max position indices
        const SlamNode &node = node_it.first->second;

        geometry_msgs::Pose curr_pose = Conversions::toMsg(node.pose_);
        //left: min x
        if(curr_pose.position.x < boundingBox[0].position.x)
            boundingBox[0] = curr_pose;
        //right: max x
        if(curr_pose.position.x > boundingBox[1].position.x)
            boundingBox[1] = curr_pose;
        //top: max y
        if(curr_pose.position.y > boundingBox[2].position.y)
            boundingBox[2] = curr_pose;
        //bottom: min y
        if(curr_pose.position.y < boundingBox[3].position.y)
            boundingBox[3] = curr_pose;

    }
    //compute size of map
    size.x = abs(boundingBox[1].position.x - boundingBox[0].position.x) + 10 * config_.range_max;
    size.y = abs(boundingBox[2].position.y - boundingBox[3].position.y) + 10 * config_.range_max;

    origin.position.x = boundingBox[0].position.x - 5 * config_.range_max;
    origin.position.y = boundingBox[3].position.y - 5 * config_.range_max;

    return origin;
}

std::vector<cv::Point2f> GraphGridMapper::scanToPoints(const LaserscanDataPtr &data)
{
    std::vector<cv::Point2f> poi;
    auto deg = data->scan_.angle_min;
    for (auto r : data->scan_.ranges) {
        if (!isnan(r) && r >= data->scan_.range_min && r <= data->scan_.range_max) {
            Eigen::Vector3f point = data->displacement_.cast<float>() * Eigen::Vector3f(std::cos(deg) * r, std::sin(deg) * r, 0);
            poi.push_back(cv::Point2f(point(0), point(1)));
        }

        deg += data->scan_.angle_increment;
    }
    return poi;
}

std::vector<cv::Point2f> GraphGridMapper::scanToIntensities(const LaserscanDataPtr &data)
{
    std::vector<cv::Point2f> poi;
    auto deg = data->scan_.angle_min;
    for (auto r : data->scan_.intensities) {
        if (!isnan(r) && r >= data->scan_.range_min && r <= data->scan_.range_max) {
            Eigen::Vector3f point = data->displacement_.cast<float>() * Eigen::Vector3f(std::cos(deg) * r, std::sin(deg) * r, 0);
            poi.push_back(cv::Point2f(point(0), point(1)));
        }

        deg += data->scan_.angle_increment;
    }
    return poi;
}

Eigen::Vector3f GraphGridMapper::scanMean(const sensor_msgs::LaserScan &scan)
{
    Eigen::Vector3f scan_mean = Eigen::Vector3f::Zero();
    float scan_count = 0;
    auto deg = scan.angle_min;
    for (auto r : scan.ranges) {
        if (!isnan(r) && r > scan.range_min && r <= scan.range_max) {
            scan_mean += Eigen::Vector3f(std::cos(deg) * r, std::sin(deg) * r, 0);
            scan_count++;
        }
        deg += scan.angle_increment;
    }
    if (scan_count > 0) {
        scan_mean /= scan_count;
    }
    return scan_mean;
}
