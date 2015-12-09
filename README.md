# UZLITI_SLAM

uzliti_slam is a collection of ROS packages for Visual Simultaneous Localization and Mapping (VSLAM).

Video: http://youtu.be/6Eie_d_URKg

When referencing this work, please cite
Hartmann, Jan; Klüssendorff, Jan Helge; Maehle, Erik: A Unified Visual Graph-Based Approach to Navigation for Wheeled Mobile Robots. In: Proc. of the IEEE Int. Conf. on Intelligent Robots and Systems (IROS), Tokyo 2013
https://www.iti.uni-luebeck.de/fileadmin/Rob/paper/HKM13.pdf

## Guide

To download required packages, download the uzliti_slam package into a new workspace and use the provided rosinstall file:

```
wstool init src src/uzliti_slam/uzliti_slam.rosinstall
```

To run the algorithm live, run:

```
roslaunch iti_slam_launch live.launch
```

To start the algorithm offline, run:

```
roslaunch iti_slam_launch slam.launch
```

To play the ITI Dataset, run:

```
roslaunch iti_slam_launch dataset.launch
```

More documentation will soon follow.

## Disclaimer

The software has been developed with Ubuntu 14.04 and ROS Indigo. As this is research code, we give NO WARRENTY and disclaim any fitness for a particular purpose.

## License

uzliti_slam is licensed under GPLv3 License. In this case, please contact the author if you are interested in commercial use.

## Packages

### feature_extraction

Extracts keypoint and image descriptors as well as laserscans from RGBD images.

### graph_slam

Implements the VSLAM algorithm.

### graph_slam_msgs

Defines ROS messages and services, e.g. for the SLAM graph and sensor data.

### graph_slam_common

Provides the SLAM graph data structure.

### graph_optimization

Provides a library for graph optimization, e.g. wraps the g2o optimizer so that it can be used with the VSLAM data structures.

### map_projection

Implements the creation of an occupancy grid map from the SLAM graph.

### place_recognition

Implements several methods for place recognition.

### transformation_estimation

Implements several methods for estimation edges in the SLAM graph, including feature- and laser-based edges.

### iti_slam_launch

Contains lauch files and configurations for running the VSLAM.



## REQUIREMENTS

uzliti_slam requires several packages and libraries that are not properly distributed via the official ROS repositories:
* csm			https://github.com/jhfrost/csm.git				branch: csm_ros
* occupancy_grid_utils	https://github.com/jhfrost/occupancy_grid_utils.git		branch: indigo_devel
* g2o			https://github.com/RainerKuemmerle/g2o.git			branch: master

In case of g2o, we advise you to build it from source. We would further advise you to use our modified robot_pose_ekf package
* navigation		https://github.com/jhfrost/navigation.git			branch: jade_devel

## REMARKS

Accuracy of the occupancy grid map depends on
* the accuracy of the wheel odometry. We would advise you to add inertial measurements and use the robot_pose_ekf.
* the accuracy of your TF robot model. The transform base_link -> camera_link in particular must be accurate.

The Kinect has a small field of view. Consider using multiple cameras (e.g. front and backward facing cameras) for better performance. The algorithm supports an arbitrary number of cameras.

