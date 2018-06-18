# velo2rgbd_calibration [![Build Status](http://build.ros.org/job/Kdev__velo2rgbd_calibration__ubuntu_xenial_amd64/8/badge/icon)](http://build.ros.org/job/Kdev__velo2rgbd_calibration__ubuntu_xenial_amd64/8/)

The *velo2rgbd_calibration* software implements an Automatic Calibration algorithm for Lidar-rgbd camera setups \[1\]. This software is provided as a ROS package.

Package developed at [Intelligent Systems Laboratory](http://www.uc3m.es/islab), Universidad Carlos III de Madrid.

![gazebo screenshot](screenshots/velo2rgbd_calibration_setup.png)

# Nodes #
## rgbd_pattern ##
### Subscribed Topics ###
*cloud2* ([sensor_msgs/PointCloud2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html))

&nbsp;&nbsp;&nbsp;&nbsp;rgbd camera point cloud containing points belonging to edges in the left image

*cam_plane_coeffs* ([pcl_msgs::ModelCoefficients](http://docs.ros.org/api/pcl_msgs/html/msg/ModelCoefficients.html))

&nbsp;&nbsp;&nbsp;&nbsp;Coefficients of the calibration target plane model
### Published Topics ###
*/rgbd_pattern/centers_cloud* ([velo2rgbd_calibration::ClusterCentroids](http://docs.ros.org/kinetic/api/velo2rgbd_calibration/html/msg/ClusterCentroids.html))

&nbsp;&nbsp;&nbsp;&nbsp;Target circles centers obtained from rgbd camera data

<!-- ### Parameters ### -->
## laser_pattern ##
### Subscribed Topics ###
*cloud1* ([sensor_msgs/PointCloud2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html))

&nbsp;&nbsp;&nbsp;&nbsp;LIDAR pointcloud
### Published Topics ###
*/laser_pattern/centers_cloud* ([velo2rgbd_calibration::ClusterCentroids](http://docs.ros.org/kinetic/api/velo2rgbd_calibration/html/msg/ClusterCentroids.html))

&nbsp;&nbsp;&nbsp;&nbsp;Target circles centers obtained from LIDAR data
<!-- ### Parameters ### -->
## velo2rgbd_calibration ##
### Subscribed Topics ###
*~cloud1* ([velo2rgbd_calibration::ClusterCentroids](http://docs.ros.org/kinetic/api/velo2rgbd_calibration/html/msg/ClusterCentroids.html))

&nbsp;&nbsp;&nbsp;&nbsp;Target circles centers obtained from LIDAR data

*~cloud2* ([velo2rgbd_calibration::ClusterCentroids](http://docs.ros.org/kinetic/api/velo2rgbd_calibration/html/msg/ClusterCentroids.html))

&nbsp;&nbsp;&nbsp;&nbsp;Target circles centers obtained from rgbd camera data

*~cloud3* ([sensor_msgs/PointCloud2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html))

&nbsp;&nbsp;&nbsp;&nbsp;Original LIDAR pointcloud
### Published Topics ###
TF containing the transformation between both sensors ([see TF info in ROS Wiki](http://wiki.ros.org/tf))

The node broadcasts the TF transformation between *velodyne* and *rgbd* frames.
The fixed transformation between *stereo_camera* and *rgbd* is published by a static broadcaster in *rgbd_pattern.launch*.
The picture below shows the three coordinate frames:

![gazebo screenshot](screenshots/frames.png)

**Note**: Additionally, a .launch file called *calibrated_tf.launch* containing the proper TF broadcasters is created in the */launch* folder of the *velo2rgbd_calibration* package so
you can use the calibration just executing:

```roslaunch velo2rgbd_calibration calibrated_tf.launch```

### Parameters ###
As described in the paper, different parameters can be selected. The ones that you will usually need are set in the launch files. For convenience, two sets of launch files are provided, with different parameters intended for the Gazebo environment, on the one hand, or the real wood pattern, on the other hand. The latter have *real_* as a prefix.

# Usage #
Some sample .launch files are provided in this package. The simplest way to launch the algorithm is by running the three main ROS nodes as follows:

```roslaunch velo2rgbd_calibration laser_pattern.launch```

```roslaunch velo2rgbd_calibration rgbd_pattern.launch```

```roslaunch velo2rgbd_calibration velo2rgbd_calibration.launch```

We also provide a launch file containing the three launch files above:

```roslaunch velo2rgbd_calibration full_calibration.launch```

In order to test the algorithm, you can download an example .bag file [here](https://www.dropbox.com/s/fdvew31azdp9cbg/1_trans_0.bag?dl=1).

**Note**: The .bag file above was generated using a simulator environment in Gazebo which is provided [here](https://github.com/beltransen/velo2rgbd_gazebo). We use the simulator in order to test the algorithm with a proper ground truth.

# Calibration target details #
The following scheme shows the real size of the calibration target used by this algorithm. Measurements are given in centimeters (cm).

![gazebo screenshot](screenshots/calibration_target_scheme.png)

**Note:** Other size may be used for convenience. If so, please configure nodes parameters accordingly.

# Citation #
\[1\] Guindel, C., Beltrán, J., Martín, D. and García, F. (2017). Automatic Extrinsic Calibration for Lidar-rgbd Vehicle Sensor Setups. *IEEE International Conference on Intelligent Transportation Systems (ITSC), 674–679*.

Pre-print available [here](https://arxiv.org/abs/1705.04085).
