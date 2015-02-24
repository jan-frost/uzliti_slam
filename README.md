# UZLITI_SLAM

uzliti_slam is a collection of ROS packages for Visual Simultaneous Localization and Mapping (VSLAM).

Video: 

When referencing this work, please cite
Hartmann, Jan; Klüssendorff, Jan Helge; Maehle, Erik: A Unified Visual Graph-Based Approach to Navigation for Wheeled Mobile Robots. In: Proc. of the IEEE Int. Conf. on Intelligent Robots and Systems (IROS), Tokyo 2013
https://www.iti.uni-luebeck.de/fileadmin/Rob/paper/HKM13.pdf


## Guide

Build using:

```
catkin_make -j1
```

To start the algorithm, run:

```
roslaunch iti_slam_launch graph_slam.launch
```

To play the ITI Dataset, run:

```
roslaunch iti_slam_launch dataset.launch
```

To extract an occupancy grid map, run:

```
rosservice call /graph_slam_node/map_request '{info: {type: 1}}'
```

More documentation will soon follow.

## Disclaimer

The software has been developed with Ubuntu 14.04 and ROS Indigo. As this is research code, we give NO WARRENTY and disclaim any fitness for a particular purpose.

## License

uzliti_slam is licensed under BSD License.

## Packages

### feature_extraction

Extracts keypoint and image descriptors as well as laserscans from RGBD images.

### graph_slam

Implements the VSLAM algorithm.

### graph_slam_msgs

Defines ROS messages and services, e.g. for the SLAM graph and sensor data.

### graph_slam_tools

Provides tools and algorithms for VSLAM. This includes place recognition and graph optimization.


## REQUIREMENTS

uzliti_slam requires several packages and libraries that are not properly distributed via the official ROS repositories:
* occupancy_grid_utils	https://github.com/clearpathrobotics/occupancy_grid_utils
* g2o			https://github.com/RainerKuemmerle/g2o

In case of g2o, we advise you to build it from source.

## REMARKS

Accuracy of the occupancy grid map depends on
* the accuracy of the wheel odometry. We wpuld advise you to add inertial measurements and use the robot_pose_ekf.
* the accuracy of your TF robot model. The transform base_link -> camera_link in particular must be accurate.

The Kinect has a small field of view. Consider using multiple cameras (e.g. front and backward facing cameras) for better performance. The algorithm supports an arbitrary number of cameras.

