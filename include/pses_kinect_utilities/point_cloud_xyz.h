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

typedef sensor_msgs::PointCloud2 PointCloudMsg;

namespace pses_kinect_utilities
{

typedef std::shared_ptr<DepthImageToPCL> DepthConvPtr;

class PointCloudXYZNodelet : public nodelet::Nodelet
{
private:
  int queue_size_;
  std::string depth_image_topic_;
  std::string output_topic_;
  std::string camera_info_topic_;
  std::string tf_frame_;
  std::string cl_file_path_;
  PointCloudMsg::Ptr current_cloud_;
  DepthConvPtr pcl_conversion_;
  image_geometry::PinholeCameraModel camera_model_;
  bool kernel_ready_;
  // Subscriptions
  boost::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::CameraSubscriber sub_depth_;
  // Publications
  boost::mutex connect_mutex_;
  ros::Publisher pub_cloud_;

  virtual void onInit();

  void connectCb();

  void depthCb(const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);
};
}

#endif // POINT_CLOUD_XYZ_H
