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

#ifndef GRAPH_SLAM_NODE_H
#define GRAPH_SLAM_NODE_H

#define BOOST_GEOMETRY_OVERLAY_NO_THROW
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <graph_slam/GraphSlamConfig.h>
#include <graph_slam_msgs/Graph.h>
#include <graph_slam_msgs/GraphReceived.h>
#include <graph_slam_msgs/EmptyService.h>
#include <graph_slam_msgs/MapRequestService.h>
#include <graph_slam_msgs/Scope.h>
#include <graph_slam_common/slam_graph.h>
#include <graph_optimization/g2o_optimizer.h>
#include <graph_optimization/sensor_transform_optimizer.h>
#include <place_recognition/binary_gist_recognizer.h>
#include <place_recognition/global_feature_repository_recognizer.h>
#include <place_recognition/lsh_set_recognizer.h>
#include <map_projection/occupancy_grid_projector.h>
#include <transformation_estimation/feature_transformation_estimator.h>
#include <nav_msgs/Odometry.h>
#include <tf/message_filter.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <thread>
#include <mongo/client/dbclientinterface.h>
#include <chrono>

//* Graph SLAM class
/**
* The GraphSlamNode class forms the backend for the SLAM algorithm.
* It is derived from the general GraphCore class.
*/
class GraphSlamNode {
public:
    GraphSlamNode(ros::NodeHandle nh, std::string name, bool sync_to_database);

    // Message callbacks
    //---------------------------------------------------------------------------------------------------------

    void sensorDataCallback(const graph_slam_msgs::SensorDataArrayConstPtr &sensor_data_array);

    void scopeCallback(const graph_slam_msgs::GraphConstPtr &graph_msg);

    void graphReceivedCallback(const graph_slam_msgs::GraphReceivedConstPtr &graph_msg);

    void subGraphCallback(const graph_slam_msgs::GraphConstPtr &graph_msg);

    void scopeRequestCallback(const graph_slam_msgs::ScopeConstPtr &scope_msg);


    // Dynamic reconfigure callbacks
    //---------------------------------------------------------------------------------------------------------

    void graphSlamConfigCallback(graph_slam::GraphSlamConfig &config, uint32_t level);

    void graphOptimizerConfigCallback(graph_optimization::GraphOptimizerConfig &config, uint32_t level);

    void linkEstimationConfigCallback(transformation_estimation::FeatureLinkEstimationConfig &config, uint32_t level);

    void occupancyGridConfigCallback(map_projection::OccupancyGridProjectorConfig &config, uint32_t level);

    void placeRecognizerConfigCallback(place_recognition::PlaceRecognizerConfig &config, uint32_t level);

    std::string odom_frame_;
    std::string odom_childframe_;

protected:
    void finishedGraphOptimization();

    void newEdgeCallback(SlamEdge &edge);

    void newCloudEdgeCallback(SlamEdge &edge);

    void odomTimerCallback(const ros::TimerEvent& e);

    void optimizationTimerCallback(const ros::TimerEvent& e);

    void cloudRegistrationTimerCallback(const ros::TimerEvent& e);

    void scopeRequestTimerCallback(const ros::TimerEvent &e);

    void mergeTimerCallback(const ros::TimerEvent &e);

    void estimateScanEdge(SlamNode node1);

    void load();

    bool mergeNodes(std::string id_first, std::string id_second);

    bool checkEdgeHeuristic(const SlamEdge &edge);

    ros::NodeHandle nh_;                /**< Main node handle. */

    /**< SLAM graph structure. The SLAM graph is stored as a STL map, where the node
    identifier acts as the key to the corresponding SlamNode object.*/
    SlamGraph graph;

    boost::shared_ptr<PlaceRecognizer> place_recognizer_;

    boost::shared_ptr<G2oOptimizer> optimizer_;
    boost::shared_ptr<SensorTransformOptimizer> sensor_optimizer_;

    boost::shared_ptr<FeatureTransformationEstimator> transformation_estimator_;
    boost::shared_ptr<TransformationEstimator> cloud_transformation_estimator_;

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
    tf::TransformListener tfl_; /** @brief to lookup transform between camera_link and base_link */

    ros::Timer optimization_timer_;
    ros::Timer cloud_registration_timer_;
    ros::Timer odom_timer_;

    std::string last_frame_id_;
    std::string last_scan_node_;

    bool sync_to_database_;

    std::set<std::string> nodes_to_send_;
    std::set<std::string> edges_to_send_;

    double min_accept_valid_;

    int new_nodes_since_last_optimization_;

    bool is_sub_graph_;
    bool is_super_graph_;
    bool construct_nodes_;
    bool publish_map_transform_;
    bool publish_occupancy_grid_;

    ros::Publisher sub_graph_pub_;
    ros::Publisher scope_request_pub_;
    ros::Publisher scope_vis_pub_;
    ros::Timer scope_request_timer_;
    double current_scope_;
    std::set<std::string> nodes_sent_success_;
    std::set<std::string> edges_sent_success_;

    ros::Timer merge_timer_;
    ros::Timer laser_registration_timer_;

    Eigen::Vector3d last_scope_center_;
    double last_scope_radius_;

    ros::Publisher scope_pub_;
    ros::Publisher graph_received_pub_;

    int current_merge_index_;
    int current_scan_index_;

    bool graph_initialized_;

};

#endif
