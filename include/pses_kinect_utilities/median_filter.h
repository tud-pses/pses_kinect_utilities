#ifndef MEDIAN_FILTER_NODELET_H
#define MEDIAN_FILTER_NODELET_H

#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <pses_kinect_utilities/MedianFilterConfig.h>
#include <dynamic_reconfigure/server.h>

namespace pses_kinect_utilities
{

class MedianFilterNodelet : public nodelet::Nodelet
{
private:
  int queue_size_;
  MedianFilterConfig config_;
  sensor_msgs::Image::Ptr current_frame_;
  boost::shared_ptr<dynamic_reconfigure::Server<MedianFilterConfig>> reconfigureServer;
  // Subscriptions
  boost::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::CameraSubscriber sub_depth_;
  // Publications
  boost::mutex connect_mutex_;
  image_transport::CameraPublisher pub_filtered_image_;

  virtual void onInit();

  void connectCb();

  void depthCb(const sensor_msgs::ImageConstPtr& depth_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg);

  void dynReconfCb(MedianFilterConfig& inputConfig, uint32_t level);
};
} // namespace pses_kinect_utilities

#endif // MEDIAN_FILTER_NODELET_H
