#include <graph_slam_tools/graph/slam_graph.h>
#include <graph_slam_tools/graph/rosbag_storage.h>
#include <graph_slam_tools/graph/mongodb_storage.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "convert_database");
    ros::NodeHandle nh;

    SlamGraph graph("/map", "slam", true);
    graph.initializeStorage("db_live", true);

    MongodbStorage mdb_storage(nh);

    mdb_storage.loadGraph(graph);
    graph.storeToDatabase();

    return 0;
}

