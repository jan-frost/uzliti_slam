#include <global_feature_repository/gfr_node.h>
#include <graph_slam_msgs/FeatureType.h>
#include <rosbag/bag.h>
#include <rosbag/query.h>
#include <rosbag/view.h>
#include <graph_slam_msgs/Graph.h>
#include <graph_slam_tools/conversions.h>

GFRNode::GFRNode (ros::NodeHandle nh) :
    _nh(nh)
{
    addNodeService = _nh.advertiseService("add_node", &GFRNode::addNodeCallback, this);
    getNeighborsService = _nh.advertiseService("get_neighbors", &GFRNode::getNeighborsServiceCallback, this);
    loadService = _nh.advertiseService("place_recognition_load", &GFRNode::loadCallback, this);
    neighbor_pub = _nh.advertise<graph_slam_msgs::NeighborArray>("/potential_neighbors", 1);

    init = false;
    int descriptor_type;
    ros::NodeHandle nhp("~");
    nhp.param<int>("descriptor_type", descriptor_type, graph_slam_msgs::FeatureType::BRISK);

    ROS_INFO("desc %d",descriptor_type);
    gfr = new FeatureRep(descriptor_type);
    lmnId = -1;
}

bool GFRNode::addNodeCallback(graph_slam_msgs::AddNode::Request &req, graph_slam_msgs::AddNode::Response &resp)
{
    //search for matching in GFR
    Conversions::fromMsg(req.node.sensor_data.data, lmnDesc);
    cv::Mat bestMatches;
    std::vector<int> potNeighbors;

    gfr->match(lmnDesc,lmnDescInRep,bestMatches,potNeighbors);
    ROS_DEBUG_NAMED("global_feature_repository", "Found %d pot Neighbors", (int)potNeighbors.size());
    lmnId = req.node.id;

    //return matching nodes
    resp.neighbors.header.stamp = ros::Time::now();
    for (int i = 0; i < potNeighbors.size(); i++)
    {
        graph_slam_msgs::Neighbor n;
        n.id_from = potNeighbors[i];
        n.id_to = req.node.id;
        n.score = 0.0;
        resp.neighbors.neighbors.push_back(n);
    }

    //integrate node into GFR
    ROS_DEBUG_NAMED("global_feature_repository", "Id %d - ADD %d", (int)lmnId, (int)lmnDescInRep.size());
    for (unsigned int i = 0; i < lmnDescInRep.size(); i++)
    {
        if(lmnDescInRep(i) == -1)
        { //add new desc to feature rep);
            gfr->addDescriptor(lmnDesc.row(i),lmnId,i);
        } else { //update link to existing desc in feat rep
            gfr->addLink(lmnDescInRep(i),lmnId,i);
        }
    }
    gfr->rebuildMatcher();

    return true;
}

bool GFRNode::getNeighborsServiceCallback(graph_slam_msgs::GetNeighbors::Request &req, graph_slam_msgs::GetNeighbors::Response &resp)
{
    Conversions::fromMsg(req.node.sensor_data.data, lmnDesc);

    cv::Mat bestMatches; //contains occurence of the nodes
    std::vector<int> potNeighbors;
    gfr->match(lmnDesc,lmnDescInRep,bestMatches,potNeighbors);

    for (int i = 0; i < potNeighbors.size(); i++)
    {
        graph_slam_msgs::Neighbor n;
        n.id_from = potNeighbors[i];
        n.id_to = req.node.id;
        n.score = 0.0;
        resp.neighbors.neighbors.push_back(n);
    }
    ROS_INFO("Found %d pot Neighbors", (int)potNeighbors.size());

    return true;
}

void GFRNode::getNeighborsCallback(const graph_slam_msgs::NodeConstPtr &node)
{
    graph_slam_msgs::NeighborArray neighbors;
    Conversions::fromMsg(node->sensor_data.data, lmnDesc);
    cv::Mat bestMatches;
    std::vector<int> potNeighbors;

    gfr->match(lmnDesc,lmnDescInRep,bestMatches,potNeighbors);
    ROS_DEBUG_NAMED("global_feature_repository", "Found %d pot Neighbors", (int)potNeighbors.size());
    lmnId = node->id;

    //return matching nodes
    neighbors.header.stamp = ros::Time::now();
    for (int i = 0; i < potNeighbors.size(); i++)
    {
        graph_slam_msgs::Neighbor n;
        n.id_from = potNeighbors[i];
        n.id_to = node->id;
        n.score = 0.0;
        neighbors.neighbors.push_back(n);
    }
    neighbor_pub.publish(neighbors);

    //integrate node into GFR
    ROS_DEBUG_NAMED("global_feature_repository", "Id %d - ADD %d", (int)lmnId, (int)lmnDescInRep.size());
    for (unsigned int i = 0; i < lmnDescInRep.size(); i++)
    {
        if(lmnDescInRep(i) == -1)
        { //add new desc to feature rep);
            gfr->addDescriptor(lmnDesc.row(i),lmnId,i);
        } else { //update link to existing desc in feat rep
            gfr->addLink(lmnDescInRep(i),lmnId,i);
        }
    }
    gfr->rebuildMatcher();
}

bool GFRNode::loadCallback(graph_slam_msgs::LoadGraph::Request &req, graph_slam_msgs::LoadGraph::Response &resp)
{
    ROS_INFO_NAMED("global_feature_repository", "Initializing place recognition from file: %s", req.file_name.c_str());
    gfr->clear();

    rosbag::Bag bag;
    bag.open(req.file_name, rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(std::string("map"));

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    graph_slam_msgs::Graph::ConstPtr graph_msg;
    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
        graph_msg = m.instantiate<graph_slam_msgs::Graph>();
        if (graph_msg != NULL) {
            break;
        }
    }
    bag.close();

    for (unsigned int i = 0; i < graph_msg->nodes.size(); i++) {
        graph_slam_msgs::Node node = graph_msg->nodes[i];

        // Extract feature descriptors of the node.
        Conversions::fromMsg(node.sensor_data.data, lmnDesc);
        cv::Mat bestMatches;
        std::vector<int> potNeighbors;

        gfr->match(lmnDesc,lmnDescInRep,bestMatches,potNeighbors);
        lmnId = node.id;

        //integrate node into GFR
        for (unsigned int j = 0; j < lmnDescInRep.size(); j++)
        {
            if(lmnDescInRep(j) == -1){      //add new desc to feature rep
                gfr->addDescriptor(lmnDesc.row(j),lmnId,j);
            } else {                        //update link to existing desc in feat rep
                gfr->addLink(lmnDescInRep(j),lmnId,j);
            }
        }
        gfr->rebuildMatcher();
    }
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "global_feature_repository_node");
    ros::NodeHandle nh;
    GFRNode pr(nh);

    ros::Subscriber request_sub =
            nh.subscribe<graph_slam_msgs::Node>("/neighbor_request", 1, boost::bind(&GFRNode::getNeighborsCallback, &pr, _1));

    ros::spin();
}
