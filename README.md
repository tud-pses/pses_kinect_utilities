# PSES Kinect Utilities

This ROS-package provides some utilities to cope with the noisy and dense data of the kinect and to perform efficiently some useful conversions such as depth image to point cloud or point cloud to laserscan.

### Prerequisites

This project was build with ROS Kinetic but should work on older ROS versions as well. In addition to that, this project depends on the ROS packages **[cv_bridge](http://wiki.ros.org/cv_bridge)**, **[pcl_ros](http://wiki.ros.org/pcl_ros)** and **[tf2_sensor_msgs](http://wiki.ros.org/tf2_sensor_msgs)**. You will need to install them, if they are not already installed in your system. More about that in a moment.


### Installing

Clone the repo into your ROS src folder:

`cd ~/catkin_ws/src`

`git clone https://github.com/tud-pses/pses_kinect_utilities.git`

`cd ..`

Install all dependencies of the package:

`rosdep install pses_kinect_utilities`

Build the package with catkin_make:

`catkin_make`

## Getting Started

Please feel free to browse our wiki, where you can find instructions on how to use and configure this package.

[Pses_Kinect_Utilities Wiki](https://github.com/tud-pses/pses_kinect_utilities/wiki)

If you're looking for a documentation of the code, please follow this link:

[C++ documentation](https://tud-pses.github.io/pses_kinect_utilities/)


## Screenshots

Here are some screenshots from our toolkit:
![color image](http://gdurl.com/eVJv)
*Original color image delivered by the [kinect2_bridge](https://github.com/tud-pses/iai_kinect2)*

![depth image](http://gdurl.com/zjkm)
*Original depth image delivered by the [kinect2_bridge](https://github.com/tud-pses/iai_kinect2)*

![depth_filtered](http://gdurl.com/VgR6)
*Depth image after being filtered by a median filter with our package*

![point_cloud](http://gdurl.com/baz4)

*Point cloud computed and published by our package*

**Note: We implemented our own point cloud conversion, which is performed on the GPU using a opencl kernel. Therefore it is more efficient and faster than the pointcloud delivered by the [kinect2_bridge](https://github.com/tud-pses/iai_kinect2) or the package [depth_image_proc](http://wiki.ros.org/depth_image_proc)**

![point_cloud_filtered](http://gdurl.com/hVMu)
*Point cloud after being filtered by a voxel grid filter with our package. The **red points** conform the resulting **laserscan**, which is also provided by our package.*

## Authors

* **Nicolas Acero**
* **Sebastian Ehmes**
