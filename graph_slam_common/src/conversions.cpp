// Copyright (c) 2015, Institute of Computer Engineering (ITI), Universität zu Lübeck
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

#include <graph_slam_common/conversions.h>

#include <boost/cast.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

Eigen::MatrixXd Conversions::featureMsgToFeaturePositions(const graph_slam_msgs::Features &featureMsg)
{
    if (featureMsg.features.size() < 1) {
        return Eigen::MatrixXd();
    }

    // Read all feature positions from the feature message.
    Eigen::MatrixXd positions(3, featureMsg.features.size());
    for (unsigned int i = 0; i < featureMsg.features.size(); i++) {
        geometry_msgs::Point point = featureMsg.features[i].keypoint_position;
        positions(0,i) = point.x;
        positions(1,i) = point.y;
        positions(2,i) = point.z;
    }
    return positions;
}

geometry_msgs::PoseWithCovariance Conversions::toMsg(const Eigen::Isometry3d &pose, const Eigen::MatrixXd &Sigma)
{
    // Store the covariance in a ROS pose message.
    geometry_msgs::PoseWithCovariance pwc;
    pwc.pose = toMsg(pose);
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            pwc.covariance[6*i + j] = Sigma(i, j);
        }
    }
    return pwc;
}

geometry_msgs::Pose Conversions::toMsg(const Eigen::Isometry3d &pose)
{
    // Store the pose in a ROS pose message.
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

std::vector<graph_slam_msgs::SensorData> Conversions::toMsg(const std::vector<SensorDataPtr> &sensor_data)
{
    std::vector<graph_slam_msgs::SensorData> sensor_data_msg;
    for (auto data : sensor_data) {
        sensor_data_msg.push_back(data->toMsg());
    }
    return sensor_data_msg;
}

std::vector<graph_slam_msgs::SensorData> Conversions::toMsg(const std::vector<SensorDataPtr> &sensor_data, int sensor_data_type)
{
    std::vector<graph_slam_msgs::SensorData> sensor_data_msg;
    for (auto data : sensor_data) {
        if (data->type_ == sensor_data_type) {
            sensor_data_msg.push_back(data->toMsg());
        }
    }
    return sensor_data_msg;
}

void Conversions::fromMsg(const std::vector<graph_slam_msgs::SensorData> &sensor_data, cv::Mat &features)
{
    // Count feature data points. Assumption: all have the same feature type.
    int feature_type = 0;
    int feature_count = 0;
    int descriptor_size = 0;
    for (auto data : sensor_data) {
        if (data.sensor_type == graph_slam_msgs::SensorData::SENSOR_TYPE_FEATURE) {
            feature_type = data.features.descriptor_type;
            if (data.features.features.size() > 0) {
                descriptor_size = data.features.features[0].descriptor.size();
                feature_count += data.features.features.size();
            }
        }
    }

    if (feature_count > 0) {
        int k = 0;
        switch (feature_type) {
        case graph_slam_msgs::Features::BRIEF:
        case graph_slam_msgs::Features::ORB:
        case graph_slam_msgs::Features::BRISK:
        case graph_slam_msgs::Features::FREAK:
            features.create(feature_count, descriptor_size, CV_8U);
            for (auto data : sensor_data) {
                if (data.sensor_type == graph_slam_msgs::SensorData::SENSOR_TYPE_FEATURE) {
                    for (unsigned int i = 0; i < data.features.features.size(); i++) {
                        for (int j = 0; j < descriptor_size; j++) {
                            float val = data.features.features[i].descriptor[j];
                            features.at<unsigned char>(k,j) = (unsigned char)val;
                        }
                        k++;
                    }
                }
            }
            break;
        case graph_slam_msgs::Features::SURF:
        case graph_slam_msgs::Features::SIFT:
            features.create(feature_count, descriptor_size, CV_32F);
            for (auto data : sensor_data) {
                if (data.sensor_type == graph_slam_msgs::SensorData::SENSOR_TYPE_FEATURE) {
                    for (unsigned int i = 0; i < data.features.features.size(); i++) {
                        for (int j = 0; j < descriptor_size; j++) {
                            features.at<float>(k,j) = data.features.features[i].descriptor[j];
                        }
                        k++;
                    }
                }
            }
            break;
        default:
            ROS_ERROR_NAMED("feature_link_estimation", "unknown feature type: %d (LE)", feature_type);
            break;
        }
    }
}

std::vector<SensorDataPtr> Conversions::fromMsg(const graph_slam_msgs::SensorDataArray &sensor_data_msg)
{
    std::vector<SensorDataPtr> sensor_data_array;
    for (auto data : sensor_data_msg.data) {
        sensor_data_array.push_back(fromMsg(data));
    }
    return sensor_data_array;
}

SensorDataPtr Conversions::fromMsg(const graph_slam_msgs::SensorData &sensor_data_msg)
{
    if (sensor_data_msg.sensor_type == graph_slam_msgs::SensorData::SENSOR_TYPE_FEATURE) {
        FeatureDataPtr feature_data(new FeatureData());
        feature_data->fromMsg(sensor_data_msg);
        return feature_data;
    } else if (sensor_data_msg.sensor_type == graph_slam_msgs::SensorData::SENSOR_TYPE_DEPTH_IMAGE) {
        DepthImageDataPtr depth_data(new DepthImageData());
        depth_data->fromMsg(sensor_data_msg);
        return depth_data;
    } else if (sensor_data_msg.sensor_type == graph_slam_msgs::SensorData::SENSOR_TYPE_BINARY_GIST) {
        BinaryGistDataPtr gist_data(new BinaryGistData());
        gist_data->fromMsg(sensor_data_msg);
        return gist_data;
    } else if (sensor_data_msg.sensor_type == graph_slam_msgs::SensorData::SENSOR_TYPE_LASERSCAN) {
        LaserscanDataPtr laser_data(new LaserscanData());
        laser_data->fromMsg(sensor_data_msg);
        return laser_data;
    } else {
        SensorDataPtr unknown_data(new SensorData());
        unknown_data->fromMsg(sensor_data_msg);
        return unknown_data;
    }
}

graph_slam_msgs::Graph Conversions::toMsg(SlamGraph &graph, bool add_sensor_data)
{
    graph_slam_msgs::Graph graph_msg;
    graph_msg.header.stamp = ros::Time::now();
    graph_msg.header.frame_id = graph.frame();
    graph_msg.gps_offset = Conversions::toMsg(graph.getGlobalOffset());
    graph_msg.gps_zone = graph.getGPSZone();

    // Add nodes.
    for (auto node_it = graph.nodeIterator(); node_it.first != node_it.second; ++node_it.first) {
        graph_msg.nodes.push_back(Conversions::toMsg(node_it.first->second, add_sensor_data));
    }

    // Add links.
    for (auto edge_it = graph.edgeIterator(); edge_it.first != edge_it.second; ++edge_it.first) {
        graph_msg.edges.push_back(Conversions::toMsg(edge_it.first->second));
    }

    // Add odometry parameters.
    for (int i = 0; i < 6; i++) {
        graph_msg.odometry_parameters[i] = graph.odom()(i);
    }

    // Add sensor transformations.
    for (auto sensor_it = graph.sensorIterator(); sensor_it.first != sensor_it.second; ++sensor_it.first) {
        graph_slam_msgs::SensorTransform sensor_tranform_msg;
        sensor_tranform_msg.sensor_name = sensor_it.first->first;
        sensor_tranform_msg.transform = Conversions::toMsg(sensor_it.first->second);
        graph_msg.sensor_tranforms.push_back(sensor_tranform_msg);
    }

    return graph_msg;
}

void Conversions::fromMsg(const geometry_msgs::PoseWithCovariance &poseMsg, Eigen::Isometry3d &pose, Eigen::MatrixXd &covariance)
{
    pose = fromMsg(poseMsg.pose);

    covariance.resize(6,6);
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            covariance(i,j) = poseMsg.covariance[6*i + j];
        }
    }
}

Eigen::Isometry3d Conversions::fromMsg(const geometry_msgs::Pose &poseMsg)
{
    g2o::Vector7d v;
    v << poseMsg.position.x,
            poseMsg.position.y,
            poseMsg.position.z,
            poseMsg.orientation.x,
            poseMsg.orientation.y,
            poseMsg.orientation.z,
            poseMsg.orientation.w;
    Eigen::Isometry3d T = g2o::internal::fromVectorQT(v);
    return T;
}

SlamEdge Conversions::fromMsg(const graph_slam_msgs::Edge &edge_msg)
{
    Eigen::Isometry3d T;
    Eigen::MatrixXd I;
    Conversions::fromMsg(edge_msg.transformation, T, I);

    SlamEdge edge = SlamEdge(edge_msg.id, edge_msg.id_from, edge_msg.id_to, T, I, edge_msg.type, edge_msg.sensor_from, edge_msg.sensor_to, edge_msg.diff_time);
    edge.matching_score_ = edge_msg.matching_score;
    edge.displacement_from_ = fromMsg(edge_msg.displacement_from);
    edge.displacement_to_ = fromMsg(edge_msg.displacement_to);
    edge.error_ = edge_msg.error;
    edge.age_ = edge_msg.age;
    edge.valid_ = edge_msg.valid;
    return edge;
}

graph_slam_msgs::Edge Conversions::toMsg(const SlamEdge &edge)
{
    graph_slam_msgs::Edge edge_msg;
    edge_msg.id = edge.id_;
    edge_msg.type = edge.type_;
    edge_msg.id_from = edge.id_from_;
    edge_msg.id_to = edge.id_to_;
    edge_msg.transformation = toMsg(edge.transform_, edge.information_);
    edge_msg.error = edge.error_;
    edge_msg.matching_score = edge.matching_score_;
    edge_msg.sensor_from = edge.sensor_from_;
    edge_msg.sensor_to = edge.sensor_to_;
    edge_msg.displacement_from = toMsg(edge.displacement_from_);
    edge_msg.displacement_to = toMsg(edge.displacement_to_);
    edge_msg.error = edge.error_;
    edge_msg.age = edge.age_;
    edge_msg.valid = edge.valid_;
    edge_msg.diff_time = edge.diff_time_;
    return edge_msg;
}

SlamNode Conversions::fromMsg(const graph_slam_msgs::Node &node_msg, bool copy_sensor_data)
{
    SlamNode node;
    for (auto stamp : node_msg.stamps) {
        node.stamps_.push_back(stamp);
    }
    node.id_ = node_msg.id;
    node.pose_ = Conversions::fromMsg(node_msg.pose);
    node.sub_pose_ = Conversions::fromMsg(node_msg.odom_pose);
    node.fixed_ = node_msg.fixed;
    node.uncertainty_ = node_msg.uncertainty;
    for (auto edge : node_msg.edge_ids) {
        node.edges_.insert(edge);
    }

    if (copy_sensor_data) {
        node.sensor_data_ = Conversions::fromMsg(node_msg.sensor_data);
    }

    return node;
}

graph_slam_msgs::Node Conversions::toMsg(const SlamNode &node, bool copy_sensor_data)
{
    graph_slam_msgs::Node node_msg;
    for (auto stamp : node.stamps_) {
        node_msg.stamps.push_back(stamp);
    }
    node_msg.id = node.id_;
    node_msg.pose = Conversions::toMsg(node.pose_);
    node_msg.odom_pose = Conversions::toMsg(node.sub_pose_);
    node_msg.fixed = node.fixed_;
    node_msg.uncertainty = node.uncertainty_;
    for (auto edge : node.edges_) {
        node_msg.edge_ids.push_back(edge);
    }

    if (copy_sensor_data) {
        node_msg.sensor_data.data = Conversions::toMsg(node.sensor_data_);
    }

    return node_msg;
}

Eigen::Vector3d Conversions::fromMsg(const geometry_msgs::Point &point_msg)
{
    return Eigen::Vector3d(point_msg.x, point_msg.y, point_msg.z);
}

geometry_msgs::Point Conversions::toMsg(const Eigen::Vector3d &point)
{
    geometry_msgs::Point point_msg;
    point_msg.x = point(0);
    point_msg.y = point(1);
    point_msg.z = point(2);
    return point_msg;
}

void Conversions::toMsg(const g2o::Vector6d &point, boost::array<double, 6> &point_msg)
{
    for (int i = 0; i < 6; i++) {
        point_msg[i] = point(i);
    }
}

g2o::Vector6d Conversions::fromMsg(const boost::array<double, 6> &point_msg)
{
    g2o::Vector6d res;
    for (int i = 0; i < 6; i++) {
        res(i) = point_msg[i];
    }
    return res;
}

graph_slam_msgs::SensorTransform Conversions::toMsg(const std::string &name, const Eigen::Isometry3d transform)
{
    graph_slam_msgs::SensorTransform sensor_transform_msg;
    sensor_transform_msg.sensor_name = name;
    sensor_transform_msg.transform = Conversions::toMsg(transform);
    return sensor_transform_msg;
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Conversions::toPointCloudColor(const Eigen::Isometry3d &T, DepthImageDataPtr depth_data)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
    cv::Mat depth_img;
    cv::Mat color_img;

    cv_bridge::CvImagePtr depth_bridge;
    cv_bridge::CvImagePtr rgb_bridge;
    try {
        depth_bridge = cv_bridge::toCvCopy(depth_data->depth_image_, sensor_msgs::image_encodings::TYPE_32FC1);
        depth_img = depth_bridge->image;
        rgb_bridge = cv_bridge::toCvCopy(depth_data->color_image_, sensor_msgs::image_encodings::BGR8);
        color_img = rgb_bridge->image;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s",e.what());
        return cloud;
    }

    // Get camera info for 3D point estimation.
    image_geometry::PinholeCameraModel cam = depth_data->camera_model_;

    cloud->header.frame_id = depth_data->sensor_frame_;
    cloud->is_dense = false;
    cloud->height = depth_img.rows;
    cloud->width = depth_img.cols;
    cloud->sensor_origin_ = Eigen::Vector4f( T.translation()(0), T.translation()(1), T.translation()(2), 1.f );
    cloud->sensor_orientation_ = Eigen::Quaternionf(T.rotation().cast<float>());
    cloud->points.resize( cloud->height * cloud->width );

    size_t idx = 0;
    const float* depthdata = reinterpret_cast<float*>( &depth_img.data[0] );
    const unsigned char* colordata = &color_img.data[0];
    for(int v = 0; v < depth_img.rows; ++v) {
        for(int u = 0; u < depth_img.cols; ++u) {
            pcl::PointXYZRGBA& p = cloud->points[ idx ]; ++idx;

            float d = (*depthdata++);

            if (d > 0 && !isnan(d)/* && p.z <= 7.5*/) {
                p.z = d;
                p.x = (u - cam.cx()) * d / cam.fx();
                p.y = (v - cam.cy()) * d / cam.fy();

                cv::Point3_<uchar>* rgb = color_img.ptr<cv::Point3_<uchar> >(v,u);
                p.b = rgb->x;
                p.g = rgb->y;
                p.r = rgb->z;
                p.a = 255;
            } else {
                p.x = std::numeric_limits<float>::quiet_NaN();;
                p.y = std::numeric_limits<float>::quiet_NaN();;
                p.z = std::numeric_limits<float>::quiet_NaN();;
                p.rgb = 0;
                colordata += 3;
            }
        }
    }

    return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Conversions::toPointCloud(const Eigen::Isometry3d &T, const image_geometry::PinholeCameraModel &cam, const cv::Mat &depth_img)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->header.frame_id = "cam";
    cloud->is_dense = false;
    cloud->height = depth_img.rows;
    cloud->width = depth_img.cols;
    cloud->sensor_origin_ = Eigen::Vector4f( T.translation()(0), T.translation()(1), T.translation()(2), 1.f );
    cloud->sensor_orientation_ = Eigen::Quaternionf(T.rotation().cast<float>());
    cloud->points.resize( cloud->height * cloud->width );

    size_t idx = 0;
    const float* depthdata = reinterpret_cast<float*>( &depth_img.data[0] );
    for(int v = 0; v < depth_img.rows; ++v) {
        for(int u = 0; u < depth_img.cols; ++u) {
            pcl::PointXYZ& p = cloud->points[ idx ]; ++idx;

            float d = (*depthdata++);

            if (d > 0 && !isnan(d)) {
                p.z = d;
                p.x = (u - cam.cx()) * d / cam.fx();
                p.y = (v - cam.cy()) * d / cam.fy();
            } else {
                p.x = std::numeric_limits<float>::quiet_NaN();
                p.y = std::numeric_limits<float>::quiet_NaN();
                p.z = std::numeric_limits<float>::quiet_NaN();
            }
        }
    }
    return cloud;
}

bool Conversions::getTransform(tf::TransformListener &tfl, Eigen::Isometry3d &T, std::string from, std::string to, ros::Time stamp)
{
    geometry_msgs::Pose transform_msg;
    try {
        tf::StampedTransform t;
        tfl.lookupTransform(from, to, stamp, t);
        transform_msg.orientation.w = t.getRotation().w();
        transform_msg.orientation.x = t.getRotation().x();
        transform_msg.orientation.y = t.getRotation().y();
        transform_msg.orientation.z = t.getRotation().z();
        transform_msg.position.x = t.getOrigin().x();
        transform_msg.position.y = t.getOrigin().y();
        transform_msg.position.z = t.getOrigin().z();
    } catch (tf::LookupException le) {
        ROS_ERROR("Couldn't find transformation between %s and %s (%s)", from.c_str(), to.c_str(), le.what());
        return false;
    } catch (tf::ExtrapolationException ee) {
        ROS_ERROR("Transformation lookup time is in the future (%s)", ee.what());
        return false;
    } catch (tf::InvalidArgument ie) {
        ROS_ERROR("TF invalid argument between %s and %s (%s)", from.c_str(), to.c_str(), ie.what());
        return false;
    } catch (tf::ConnectivityException ce) {
        ROS_ERROR("TF no connectivity between %s and %s (%s)", from.c_str(), to.c_str(), ce.what());
        return false;
    }

    T = Conversions::fromMsg(transform_msg);
    return true;
}

void Conversions::writeMat( cv::Mat const& mat, const char* filename, const char* varName, bool bgr2rgb )
{
   int textLen = 116;
   char* text;
   int subsysOffsetLen = 8;
   char* subsysOffset;
   int verLen = 2;
   char* ver;
   char flags;
   int bytes;
   int padBytes;
   int bytesPerElement;
   int i,j,k,k2;
   bool doBgrSwap;
   char mxClass;
   int32_t miClass;
   uchar const* rowPtr;
   uint32_t tmp32;
   float tmp;
   FILE* fp;

   // Matlab constants.
   const uint16_t MI = 0x4d49; // Contains "MI" in ascii.
   const int32_t miINT8 = 1;
   const int32_t miUINT8 = 2;
   const int32_t miINT16 = 3;
   const int32_t miUINT16 = 4;
   const int32_t miINT32 = 5;
   const int32_t miUINT32 = 6;
   const int32_t miSINGLE = 7;
   const int32_t miDOUBLE = 9;
   const int32_t miMATRIX = 14;
   const char mxDOUBLE_CLASS = 6;
   const char mxSINGLE_CLASS = 7;
   const char mxINT8_CLASS = 8;
   const char mxUINT8_CLASS = 9;
   const char mxINT16_CLASS = 10;
   const char mxUINT16_CLASS = 11;
   const char mxINT32_CLASS = 12;
   const char mxUINT32_CLASS = 13;
   const uint64_t zero = 0; // Used for padding.

   fp = fopen( filename, "wb" );

   if( fp == 0 )
      return;

   const int rows = mat.rows;
   const int cols = mat.cols;
   const int chans = mat.channels();

   doBgrSwap = (chans==3) && bgr2rgb;

   // I hope this mapping is right :-/
   switch( mat.depth() )
   {
   case CV_8U:
      mxClass = mxUINT8_CLASS;
      miClass = miUINT8;
      bytesPerElement = 1;
      break;
   case CV_8S:
      mxClass = mxINT8_CLASS;
      miClass = miINT8;
      bytesPerElement = 1;
      break;
   case CV_16U:
      mxClass = mxUINT16_CLASS;
      miClass = miUINT16;
      bytesPerElement = 2;
      break;
   case CV_16S:
      mxClass = mxINT16_CLASS;
      miClass = miINT16;
      bytesPerElement = 2;
      break;
   case CV_32S:
      mxClass = mxINT32_CLASS;
      miClass = miINT32;
      bytesPerElement = 4;
      break;
   case CV_32F:
      mxClass = mxSINGLE_CLASS;
      miClass = miSINGLE;
      bytesPerElement = 4;
      break;
   case CV_64F:
      mxClass = mxDOUBLE_CLASS;
      miClass = miDOUBLE;
      bytesPerElement = 8;
      break;
   default:
      return;
   }

   //==================Mat-file header (128 bytes, page 1-5)==================
   text = new char[textLen]; // Human-readable text.
   memset( text, ' ', textLen );
   text[textLen-1] = '\0';
   const char* t = "MATLAB 5.0 MAT-file, Platform: PCWIN";
   memcpy( text, t, strlen(t) );

   subsysOffset = new char[subsysOffsetLen]; // Zeros for us.
   memset( subsysOffset, 0x00, subsysOffsetLen );
   ver = new char[verLen];
   ver[0] = 0x00;
   ver[1] = 0x01;

   fwrite( text, 1, textLen, fp );
   fwrite( subsysOffset, 1, subsysOffsetLen, fp );
   fwrite( ver, 1, verLen, fp );
   // Endian indicator. MI will show up as "MI" on big-endian
   // systems and "IM" on little-endian systems.
   fwrite( &MI, 2, 1, fp );
   //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

   //===================Data element tag (8 bytes, page 1-8)==================
   bytes = 16 + 24 + (8 + strlen(varName) + (8-(strlen(varName)%8))%8)
      + (8 + rows*cols*chans*bytesPerElement);
   fwrite( &miMATRIX, 4, 1, fp ); // Data type.
   fwrite( &bytes, 4, 1, fp); // Data size in bytes.
   //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

   //====================Array flags (16 bytes, page 1-15)====================
   bytes = 8;
   fwrite( &miUINT32, 4, 1, fp );
   fwrite( &bytes, 4, 1, fp );
   flags = 0x00; // Complex, logical, and global flags all off.

   tmp32 = 0;
   tmp32 = (flags << 8 ) | (mxClass);
   fwrite( &tmp32, 4, 1, fp );

   fwrite( &zero, 4, 1, fp ); // Padding to 64-bit boundary.
   //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

   //===============Dimensions subelement (24 bytes, page 1-17)===============
   bytes = 12;
   fwrite( &miINT32, 4, 1, fp );
   fwrite( &bytes, 4, 1, fp );

   fwrite( &rows, 4, 1, fp );
   fwrite( &cols, 4, 1, fp );
   fwrite( &chans, 4, 1, fp );
   fwrite( &zero, 4, 1, fp ); // Padding to 64-bit boundary.
   //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

   //==Array name (8 + strlen(varName) + (8-(strlen(varName)%8))%8 bytes, page 1-17)==
   bytes = strlen(varName);

   fwrite( &miINT8, 4, 1, fp );
   fwrite( &bytes, 4, 1, fp );
   fwrite( varName, 1, bytes, fp );

   // Pad to nearest 64-bit boundary.
   padBytes = (8-(bytes%8))%8;
   fwrite( &zero, 1, padBytes, fp );
   //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

   //====Matrix data (rows*cols*chans*bytesPerElement+8 bytes, page 1-20)=====
   bytes = rows*cols*chans*bytesPerElement;
   fwrite( &miClass, 4, 1, fp );
   fwrite( &bytes, 4, 1, fp );

   for( k = 0; k < chans; ++k )
   {
      if( doBgrSwap )
      {
         k2 = (k==0)? 2 : ((k==2)? 0 : 1);
      }
      else
         k2 = k;

      for( j = 0; j < cols; ++j )
      {
         for( i = 0; i < rows; ++i )
         {
            rowPtr = mat.data + mat.step*i;
            fwrite( rowPtr + (chans*j + k2)*bytesPerElement, bytesPerElement, 1, fp );
         }
      }
   }

   // Pad to 64-bit boundary.
   padBytes = (8-(bytes%8))%8;
   fwrite( &zero, 1, padBytes, fp );
   //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

   fclose(fp);
   delete[] text;
   delete[] subsysOffset;
   delete[] ver;
}
