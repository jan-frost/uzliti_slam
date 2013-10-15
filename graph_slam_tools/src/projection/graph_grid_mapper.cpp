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

GraphGridMapper::GraphGridMapper(){
    init = false;
}

GraphGridMapper::~GraphGridMapper(){}

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
    Eigen::Isometry3d last_pose = Eigen::Isometry3d::Identity();
    ros::Time last_stamp = ros::Time(0);
    for (node_it = graph.nodeIterator(); node_it.first != node_it.second; node_it.first++) {
        const SlamNode &node = node_it.first->second;

        Eigen::AngleAxisd diffAngleAxis(node.pose_.linear().transpose() * last_pose.linear());
        double diffRotation = fabs(diffAngleAxis.angle() / (node.stamp_ - last_stamp).toSec()) * 180 / M_PI;
        last_pose = node.pose_;
        last_stamp = node.stamp_;
        if (diffRotation < 90.) {
            // Search for depth image.
            int total_points = 0;
            for (auto data : node.sensor_data_) {
                if (data->type_ == graph_slam_msgs::SensorData::SENSOR_TYPE_DEPTH_IMAGE) {
                    boost::shared_ptr<DepthImageData> depth_data = boost::dynamic_pointer_cast<DepthImageData>(data);
                    sensor_msgs::Image depth_image = depth_data->depth_image_;

                    try
                    {
                        sensor_msgs::LaserScan laser;
                        Eigen::Isometry3d camera_transform = graph.sensor(depth_data->sensor_frame_);
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
    }

    //retrieve grid
    nav_msgs::OccupancyGridConstPtr grid = occupancy_grid_utils::getGrid(overlay);
    map = *grid;
    return true;
}

bool GraphGridMapper::convertLaserScans2Map(SlamGraph &graph, nav_msgs::OccupancyGrid &map, ros::Publisher &mapPub)
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
            if (data->type_ == graph_slam_msgs::SensorData::SENSOR_TYPE_LASERSCAN) {
                boost::shared_ptr<LaserscanData> depth_data = boost::dynamic_pointer_cast<LaserscanData>(data);
                sensor_msgs::LaserScan laser = depth_data->scan_;

                try
                {
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

    std::vector<cv::Point3d> poi;
    extractImageLaserLine(depth_image, camera_transform, camera_model, poi);
    toLaserMsg(poi, scan);
    return true;
}

void GraphGridMapper::extractImageLaserLine(const cv::Mat &depth_image, const Eigen::Isometry3d &camera_transform, const image_geometry::PinholeCameraModel &camera_model, std::vector<cv::Point3d> &poi)
{
    for (int i = 0; i < depth_image.cols; i++)
    { //we expect distance in m
        cv::Point3d min_poi(i, 0.0, std::numeric_limits<double>::infinity());
        double min_dist = config_.range_max;
        for (int j = 0; j < depth_image.rows; j++)
        {
            double cDist = (double)depth_image.at<float>(j,i);
            // Filter based on distance.
            if (!std::isinf(cDist) && !std::isnan(cDist) && cDist > config_.range_min && cDist < config_.range_max) {
                cv::Point3d curr_poi(i, j, cDist);
                cv::Point3d p3d = to3D(curr_poi, camera_transform, camera_model);

                // Fitler based on height
                if (cDist < min_dist && p3d.z <= config_.max_height && p3d.z >= config_.min_height) {
                    min_dist = cDist;
                    min_poi = p3d;
                }
            }
        }
        if (min_dist < config_.range_max) {
            poi.push_back(min_poi);
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

void GraphGridMapper::toLaserMsg(std::vector<cv::Point3d> &poi, sensor_msgs::LaserScan &scan)
{ //inspired by pointcloud_to_laserscan from WillowGarage
    scan.header.frame_id = "/base_link";
    scan.angle_min = -M_PI;
    scan.angle_max = M_PI;
    scan.angle_increment = config_.angle_increment;
    scan.time_increment = 0.0;
    scan.scan_time = 1. / 30.;
    scan.range_min = config_.range_min;
    scan.range_max = config_.range_max;
    float max_height_ = config_.max_height;
    float min_height_ = config_.min_height;

    uint32_t ranges_size = std::ceil((scan.angle_max - scan.angle_min) / scan.angle_increment);
    scan.ranges.assign(ranges_size, scan.range_max + 1.0);

    for (unsigned int i = 0; i < poi.size(); i++)
    {
        float x = poi.at(i).x;
        float y = poi.at(i).y;
        float z = poi.at(i).z;

        if ( std::isnan(x) || std::isnan(y) || std::isnan(z) )
        {
            ROS_DEBUG("rejected for nan in point(%f, %f, %f)\n", x, y, z);
            continue;
        }

        if (z > max_height_ || z < min_height_)
        {
            ROS_DEBUG("point (%f, %f, %f)\n", x, y, z);
            ROS_DEBUG("rejected for height %f not in range (%f, %f)\n", z, min_height_, max_height_);
            continue;
        }
        double angle = -atan2(-y, x);// + 15 * M_PI / 180;
        if (angle < scan.angle_min || angle > scan.angle_max)
        {
            ROS_DEBUG("rejected for angle %f not in range (%f, %f)\n", angle, scan.angle_min, scan.angle_max);
            continue;
        }
        int index = (angle - scan.angle_min) / scan.angle_increment;
        double range_sq = x*x+y*y;
        if (scan.ranges[index] * scan.ranges[index] > range_sq)
            scan.ranges[index] = sqrt(range_sq);
    }
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
    size.x = abs(boundingBox[1].position.x - boundingBox[0].position.x) + 2 * config_.range_max;
    size.y = abs(boundingBox[2].position.y - boundingBox[3].position.y) + 2 * config_.range_max;

    origin.position.x = boundingBox[0].position.x - config_.range_max;
    origin.position.y = boundingBox[3].position.y - config_.range_max;

    return origin;
}
//}
