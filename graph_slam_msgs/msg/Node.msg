Header header

#the unique number of the node
string id

#the pose of the node in the graph
geometry_msgs/Pose pose
geometry_msgs/Pose odom_pose

graph_slam_msgs/SensorDataArray sensor_data

string[] edge_ids

bool fixed
bool is_loop
bool is_border
bool is_in_scope
