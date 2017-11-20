# PSES Kinect Filter

This ROS-package provides filter for the kinect depth image based on a median and a voxel grid filter. This kinect filter returns a "clean" and less dense point cloud.

### Prerequisites

This project was build with ROS Kinetic but should work on older ROS versions as well. In addition to that, this project depends on the ROS packages **[cv_bridge](http://wiki.ros.org/cv_bridge)** and **[pcl_ros](http://wiki.ros.org/pcl_ros)**. You will need to install them, if they are not already installed in your system.


### Installing

Clone the repo into your ROS src folder:

`cd ~/catkin_ws/src`

`git clone https://github.com/tud-pses/pses_kinect_filter.git`

`cd ..`

Build the package with catkin_make:

`catkin_make`

## Getting Started

Please feel free to browse our wiki, where you can find instructions on how to use and configure this package.

[Pses_Kinect_Filter Wiki](https://github.com/tud-pses/pses_kinect_filter/wiki)

If you're looking for a documentation of the code, please follow this link:

[C++ documentation](https://tud-pses.github.io/pses_kinect_filter/)

## Authors

* **Nicolas Acero**
* **Sebastian Ehmes**
