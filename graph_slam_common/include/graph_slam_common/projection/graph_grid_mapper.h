#ifndef GRAPH_GRID_MAPPER_H
#define GRAPH_GRID_MAPPER_H

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

#include <ros/ros.h>
#include <opencv/cv.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <graph_slam_msgs/Graph.h>
#include <boost/circular_buffer.hpp>
#include <boost/optional.hpp>
#include <boost/foreach.hpp>
#include <occupancy_grid_utils/ray_tracer.h>
#include <graph_slam_tools/graph/slam_graph.h>
#include <graph_slam_tools/OccupancyGridProjectorConfig.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef boost::shared_ptr<occupancy_grid_utils::LocalizedCloud> CloudPtr;
typedef boost::shared_ptr<occupancy_grid_utils::LocalizedCloud const> CloudConstPtr;
typedef boost::circular_buffer<CloudConstPtr> CloudBuffer;

class GraphGridMapper {
public:
    GraphGridMapper();
    virtual ~GraphGridMapper();


    /**
      * @brief sets config params using dynamic reconfigure interface
      * @param config the config
      */
    void setConfig(graph_slam_tools::OccupancyGridProjectorConfig config);

    /**
      * @brief transform a point in image coordinates into 3d space
      * @param p point to transform
      * @return 3d transformation of p
      */
    cv::Point3d to3D(cv::Point3d &p, const Eigen::Isometry3d &camera_transform, const image_geometry::PinholeCameraModel &camera_model);

    static void mergeLaserScans(const LaserscanDataPtr &a, const LaserscanDataPtr &b);
    static sensor_msgs::LaserScan mergeLaserScans(const sensor_msgs::LaserScan &a, const sensor_msgs::LaserScan &b, Eigen::Isometry3d displacement);

    /**
      * @brief converts graph_slam_msgs/Graph into a 2D map as nav_msgs/OccupancyGrid
      * @param g the graph to convert
      * @param map the 2D occupancy grid map constructed from g
      * @return true if conversion was successfull
      */
    bool convertDepthImages2Map(SlamGraph &graph, nav_msgs::OccupancyGrid &map, ros::Publisher &mapPub);
    bool convertLaserScans2Map(SlamGraph &graph, nav_msgs::OccupancyGrid &map, ros::Publisher &mapPub, Eigen::Isometry3d diff_transform);
    /**
      * @brief converts sensor_msgs/Image into a sensor_msgs/LaserScan
      * @param image the depth image to extract the laserscan from
      * @param scan the extracted laserscan
      * @return if conversion was successfull
      */
    bool convertDepthImage2LaserScan(const sensor_msgs::Image &image, const Eigen::Isometry3d &camera_transform, const image_geometry::PinholeCameraModel &camera_model, sensor_msgs::LaserScan &scan);
    /**
      * @brief extracts a line containing points with closest distance to the camera from given image in camera coordinates
      * @param image the image to extract the laserscan from
      * @param poi a vector containing points of interest i.e. points with closest distance
      */
    void extractImageLaserLine(const cv::Mat &depth_image, const Eigen::Isometry3d &camera_transform, const image_geometry::PinholeCameraModel &camera_model, sensor_msgs::LaserScan &scan);
    void transformPointCloudInPlace(const Eigen::Isometry3d Tr, const pcl::PointCloud<pcl::PointXYZ>::Ptr &pc);

    /**
      * @brief transform pose from Eigen::Isometry3d to geometry_msgs::Pose
      * @param pose the pose to transform
      * @return the transformed pose
      */
    geometry_msgs::Pose affineTransformToPoseMsg(const Eigen::Isometry3d &pose);

    /**
      * @brief computes the origin of the map for constructing an occupancy grid
      * @param nodes the graph as map of nodes
      * @param size returns the size of the map (width,height)
      * @return the bottom left origin of the map
      */
    geometry_msgs::Pose getMapOrigin(SlamGraph &graph, cv::Point2d &size);

    static std::vector<cv::Point2f> scanToPoints(const LaserscanDataPtr &data);
    static std::vector<cv::Point2f> scanToIntensities(const LaserscanDataPtr &data);
    static Eigen::Vector3f scanMean(const sensor_msgs::LaserScan &scan);

    graph_slam_tools::OccupancyGridProjectorConfig config_;
    occupancy_grid_utils::OverlayClouds overlay_;
    nav_msgs::OccupancyGrid fake_grid_;
    std::vector<CloudConstPtr> clouds_;
    bool init;
    std::string last_node_id_;


};
//}

#endif
