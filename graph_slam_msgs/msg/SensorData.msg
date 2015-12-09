
int32 SENSOR_TYPE_UNKNOWN = 0
int32 SENSOR_TYPE_FEATURE = 1
int32 SENSOR_TYPE_DEPTH_IMAGE = 2
int32 SENSOR_TYPE_BINARY_GIST = 3
int32 SENSOR_TYPE_LASERSCAN = 4

Header header

int32 sensor_type

geometry_msgs/Pose displacement

string sensor_frame

graph_slam_msgs/Features features

graph_slam_msgs/DepthImage depth_image

float32[] gist_descriptor

sensor_msgs/LaserScan scan

geometry_msgs/Point scan_center
