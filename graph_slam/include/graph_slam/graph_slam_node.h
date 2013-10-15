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

#ifndef GRAPH_SLAM_NODE_H
#define GRAPH_SLAM_NODE_H

#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <graph_slam/GraphSlamConfig.h>
#include <graph_slam_msgs/Graph.h>
#include <graph_slam_msgs/EmptyService.h>
#include <graph_slam_msgs/MapRequestService.h>
#include <graph_slam_tools/graph/slam_graph.h>
#include <graph_slam_tools/optimization/g2o_optimizer.h>
#include <graph_slam_tools/pr/binary_gist_recognizer.h>
#include <graph_slam_tools/projection/occupancy_grid_projector.h>
#include <graph_slam_tools/transformation/feature_transformation_estimator.h>
#include <message_filters/subscriber.h>
#include <nav_msgs/Odometry.h>
#include <tf/message_filter.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <thread>

//* Graph SLAM class
/**
* The GraphSlamNode class forms the backend for the SLAM algorithm.
* It is derived from the general GraphCore class.
*/
class GraphSlamNode {
public:
    GraphSlamNode(ros::NodeHandle nh);

    // Sensor callbacks
    //---------------------------------------------------------------------------------------------------------

    void sensorDataCallback(const graph_slam_msgs::SensorDataArrayConstPtr &sensor_data_array);

    bool mapRequestCallback(graph_slam_msgs::MapRequestService::Request &req, graph_slam_msgs::MapRequestService::Response &res);

    bool clearCallback(graph_slam_msgs::EmptyService::Request &req, graph_slam_msgs::EmptyService::Response &res);


    // Dynamic reconfigure callbacks
    //---------------------------------------------------------------------------------------------------------

    void graphSlamConfigCallback(graph_slam::GraphSlamConfig &config, uint32_t level);

    void linkEstimationConfigCallback(graph_slam_tools::FeatureLinkEstimationConfig &config, uint32_t level);

    void occupancyGridConfigCallback(graph_slam_tools::OccupancyGridProjectorConfig &config, uint32_t level);

    void placeRecognizerConfigCallback(graph_slam_tools::PlaceRecognizerConfig &config, uint32_t level);

    void graphOptimizerConfigCallback(graph_slam_tools::GraphOptimizerConfig &config, uint32_t level);

private:
    void clear();

    void processThread();

    void finishedGraphOptimization();

    void finishedSensorOptimization();

    void newEdgeCallback(SlamEdge &edge);

    void odomTimerCallback(const ros::TimerEvent& e);

    void optimizationTimerCallback(const ros::TimerEvent& e);

    void parameterOptimizationTimerCallback(const ros::TimerEvent& e);

    bool getTransform(Eigen::Isometry3d &T, std::string from, std::string to, ros::Time stamp);

    ros::NodeHandle _nh;                /**< Main node handle. */

    /**< SLAM graph structure. The SLAM graph is stored as a STL map, where the node
    identifier acts as the key to the corresponding SlamNode object.*/
    SlamGraph graph;

    Eigen::Isometry3d map_transform_;

    boost::shared_ptr<BinaryGistRecognizer> place_recognizer_;
    boost::shared_ptr<G2oOptimizer> optimizer_;
    boost::shared_ptr<FeatureTransformationEstimator> transformation_estimator_;
    boost::shared_ptr<OccupancyGridProjector> projector_;

    graph_slam::GraphSlamConfig config_; /**< Configuration parameters as obtained from dynamic_reconfigure. */

    Eigen::MatrixXd odom_information_;                /**< Information matrix for the odometry links. */

    std::string current_node_;                   /**< Id of the current node. */
    std::string last_node_;                      /**< Id of the last node. Needed because only odometry links will be established between the current and the last node. */
    std::string most_recent_optimizer_node_;
    std::string most_recent_loop_closure_;

    Eigen::Isometry3d last_odometry_pose_;               /**< Pose of the last node in the odometry frame. */
    Eigen::Isometry3d current_odometry_pose_;            /**< Current pose in the odometry frame. */
    tf::StampedTransform odom_transform_;             /**< Holds the transform between the map and the odom frame for the transform publisher. */
    Eigen::MatrixXd odom_covariance_;

    std::mutex graph_mutex_;

    boost::uuids::random_generator uuid_generator;
    ros::Publisher pose_pub_;               /**< Publisher for the current robot pose. */
    ros::Publisher graph_display_pub_;      /**< Publisher for the current graph. This is typically used for visualization and evaluation. */
    ros::Publisher sensor_request_pub_;

    tf::TransformBroadcaster broadcaster;           /**< Broadcaster for the transform between /map and /odom. */
    tf::TransformListener tfl; /** @brief to lookup transform between camera_link and base_link */

    message_filters::Subscriber<graph_slam_msgs::SensorDataArray> sensor_sub_;
    tf::MessageFilter<graph_slam_msgs::SensorDataArray> *sensor_sub_filter_;

    message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
    tf::MessageFilter<nav_msgs::Odometry> *odom_sub_filter_;

    ros::Timer odom_timer_;
    ros::Timer optimization_timer_;

    std::string odom_frame_;
    std::string odom_childframe_;
};

#endif
