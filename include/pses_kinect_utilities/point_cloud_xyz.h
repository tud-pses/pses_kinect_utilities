/**
 * @file "pses_utilities/point_cloud_xyz.h"
 * @brief Contains the PointCloudXYZNodelet class, its member variables and
 * callback functions.
 *
*/

#ifndef POINT_CLOUD_XYZ_H
#define POINT_CLOUD_XYZ_H

#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pses_kinect_utilities/depth_image_to_pcl.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_geometry/pinhole_camera_model.h>

/**
 * @typedef sensor_msgs::PointCloud2 PointCloudMsg
 * @brief Shortcut for a PointCloud2 ROS message.
*/
typedef sensor_msgs::PointCloud2 PointCloudMsg;

/**
 * @namespace pses_kinect_utilities
 * @brief This namespace is used by the nodelets inside our package
 *pses_kinect_utilities.
 *
*/
namespace pses_kinect_utilities
{

/**
 * @typedef std::shared_ptr<DepthImageToPCL> DepthConvPtr
 * @brief Shortcut for a Pointer to a DepthImageToPCL Object.
*/
typedef std::shared_ptr<DepthImageToPCL> DepthConvPtr;

/**
 * @class PointCloudXYZNodelet point_cloud_xyz.h
 * @brief Class of the PointCloudXYZNodelet nodelet, which converts a depth
 *image into a point cloud containing x, y and z coordinates.
 *
*/
class PointCloudXYZNodelet : public nodelet::Nodelet
{
private:
  int queue_size_; /**< Attribute to store the desired queue size for the depth image subscriber. */
  std::string tf_frame_; /**< Attribute to store the tf frame id that has to be assigned to the output point cloud. */
  std::string cl_file_path_; /**< Attribute to store the path of the opencl kernel. */
  PointCloudMsg::Ptr current_cloud_; /**< Attribute to store last computed point cloud. */
  DepthConvPtr pcl_conversion_; /**< Pointer to a DepthImageToPCL object, which makes the actual conversion using a opencl kernel. */
  image_geometry::PinholeCameraModel camera_model_; /**< Attribute to store the corresponding camera model. */
  bool kernel_ready_; /**< Boolean flag that is used to know if the opencl kernel was already compiled. */
  // Subscriptions
  boost::shared_ptr<image_transport::ImageTransport> it_; /**< Pointer to an ImageTransport Object, which is used to subscribe depth images. */
  image_transport::CameraSubscriber sub_depth_; /**< Subscriber of the depth image. */
  // Publications
  boost::mutex connect_mutex_;
  ros::Publisher pub_cloud_; /**< Publisher of the point cloud. */

  /**
   * @brief This function is called by the initialization of the nodelet and acts as a constructor.
  */
  virtual void onInit();

  /**
   * @brief Callback function that is called whenever anyone is subscribed to the output point cloud.
  */
  void connectCb();

  /**
   * @brief Callback function that is called whenever a new depth image is received.
   * @param[in] depth_msg Contains the received depth image.
   * @param[in] info_msg Contains the camera info and metadata of the received depth image.
  */
  void depthCb(const sensor_msgs::ImageConstPtr& depth_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg);
};
}

#endif // POINT_CLOUD_XYZ_H
