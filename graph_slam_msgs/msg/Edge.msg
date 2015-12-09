uint8 TYPE_3D_FULL = 1
uint8 TYPE_3D_ROTATION = 2
uint8 TYPE_3D_TRANSLATION = 3
uint8 TYPE_3D_GPS = 4
uint8 TYPE_2D_FULL = 101
uint8 TYPE_2D_ROTATION = 102
uint8 TYPE_2D_TRANSLATION = 103
uint8 TYPE_2D_WHEEL_ODOMETRY = 104
uint8 TYPE_2D_LASER = 105

string id

uint8 type

# This edge connects nodes:
string id_from
string id_to

# Edge displacements
geometry_msgs/Pose displacement_from
geometry_msgs/Pose displacement_to

# Full 3D transformation between the nodes
geometry_msgs/PoseWithCovariance transformation

float64 error
float64 age
float64 matching_score

# Sensor TF frames
string sensor_from
string sensor_to

bool valid

duration diff_time
