/**
 * @file "pses_utilities/median_filter.h"
 * @brief Contains the MedianFilterNodelet class, its member variables and callback functions.
 *
*/

#ifndef MEDIAN_FILTER_NODELET_H
#define MEDIAN_FILTER_NODELET_H

#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <pses_kinect_utilities/MedianFilterConfig.h>
#include <dynamic_reconfigure/server.h>

/**
 * @namespace pses_kinect_utilities
 * @brief This namespace is used by the nodelets inside our package pses_kinect_utilities.
 *
*/
namespace pses_kinect_utilities
{

/**
 * @class MedianFilterNodelet median_filter.h
 * @brief Class of the median filter nodelet
 *
*/
class MedianFilterNodelet : public nodelet::Nodelet
{
private:
  int queue_size_; /**< Attribute to store the desired queue size for the depth image subscriber. */
  MedianFilterConfig config_; /**< Configuration of the median filter that is updated and set up by dynamic reconfigure. */
  sensor_msgs::Image::Ptr current_frame_; /**< Attribute to store current frame of the input image. */
  boost::shared_ptr<dynamic_reconfigure::Server<MedianFilterConfig> > reconfigureServer; /**< Dynamic reconfigure server. */
  // Subscriptions
  boost::shared_ptr<image_transport::ImageTransport> it_; /**< Pointer to an ImageTransport Object, which is used to subscribe and publish images. */
  image_transport::CameraSubscriber sub_depth_; /**< Subscriber of the input image. */
  // Publications
  boost::mutex connect_mutex_;
  image_transport::CameraPublisher pub_filtered_image_; /**< Publisher of the filtered image. */

  /**
   * @brief This function is called by the initialization of the nodelet and acts as a constructor.
  */
  virtual void onInit();

  /**
   * @brief Callback function that is called whenever anyone is subscribed to the output image or to its camera info.
  */
  void connectCb();

  /**
   * @brief Callback function that is called whenever a new image is received.
   * @param[in] depth_msg Contains the received image.
   * @param[in] info_msg Contains the camera info and metadata of the received image.
  */
  void depthCb(const sensor_msgs::ImageConstPtr& depth_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg);

  /**
   * @brief Callback function that is called whenever the filter configuration is updated or set up through dynamic reconfigure.
   * @param[in] inputConfig New filter configuration.
   * @param[in] level This parameter is not used in the implementation of this callback. Hence it can be ignored.
  */
  void dynReconfCb(MedianFilterConfig& inputConfig, uint32_t level);
};
} // namespace pses_kinect_utilities

#endif // MEDIAN_FILTER_NODELET_H
