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

typedef sensor_msgs::PointCloud2 PointCloudMsg;

namespace pses_kinect_utilities
{

typedef std::shared_ptr<DepthImageToPCL> DepthConvPtr;

class PointCloudXYZNodelet : public nodelet::Nodelet
{
private:
  int queue_size_;
  int kernel_size_;
  std::string depth_image_topic_;
  std::string output_topic_;
  std::string camera_info_topic_;
  std::string tf_frame_;
  PointCloudMsg::Ptr current_cloud_;
  DepthConvPtr pcl_conversion_;
  ros::Subscriber sub_depth_;
  ros::NodeHandle nh;
  // Publications
  boost::mutex connect_mutex_;
  ros::Publisher pub_cloud_;

  virtual void onInit();

  void connectCb();

  void depthCb(const sensor_msgs::ImageConstPtr& depth_msg);
};
}

#endif // POINT_CLOUD_XYZ_H
