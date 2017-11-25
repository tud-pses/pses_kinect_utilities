#ifndef MEDIAN_FILTER_NODELET_H
#define MEDIAN_FILTER_NODELET_H

#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <boost/thread.hpp>
#include <ros/ros.h>

namespace pses_kinect_utilities
{

class MedianFilterNodelet : public nodelet::Nodelet
{
//private:
  int queue_size_;
  int kernel_size_;
  std::string depth_image_topic_;
  std::string output_topic_;
  sensor_msgs::Image::Ptr current_frame_;
  // Subscriptions
  boost::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::CameraSubscriber sub_depth_;
  // Publications
  boost::mutex connect_mutex_;
  image_transport::CameraPublisher pub_filtered_image_;

  virtual void onInit();

  void connectCb();

  void depthCb(const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);
};
} // namespace pses_kinect_utilities

#endif // MEDIAN_FILTER_NODELET_H
