/**
 * @file "median_filter.cpp"
 * @brief median filter nodelet implementation, containing the callback functions.
 *
*/

#include <pses_kinect_utilities/median_filter.h>
#include <image_transport/image_transport.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/ocl.hpp>

// Register as nodelet
PLUGINLIB_EXPORT_CLASS(pses_kinect_utilities::MedianFilterNodelet,
                       nodelet::Nodelet);

namespace pses_kinect_utilities
{

void MedianFilterNodelet::onInit()
{
  NODELET_DEBUG("Initializing median filter nodelet...");

  ros::NodeHandle& private_nh = getPrivateNodeHandle();
  ros::NodeHandle& nh = getNodeHandle();

  // Init dynamic reconfigure
  reconfigureServer =
      boost::make_shared<dynamic_reconfigure::Server<MedianFilterConfig>>(
          private_nh);
  dynamic_reconfigure::Server<MedianFilterConfig>::CallbackType f;
  f = boost::bind(&MedianFilterNodelet::dynReconfCb, this, _1, _2);
  reconfigureServer->setCallback(f);

  current_frame_.reset(new sensor_msgs::Image);
  it_.reset(new image_transport::ImageTransport(nh));

  // Read parameters
  private_nh.param("queue_size", queue_size_, 1);

  // Monitor whether anyone is subscribed to the output image or to its camera
  // info
  image_transport::SubscriberStatusCallback connect_cb =
      boost::bind(&MedianFilterNodelet::connectCb, this);
  ros::SubscriberStatusCallback connect_info_cb =
      boost::bind(&MedianFilterNodelet::connectCb, this);
  // Make sure we don't enter connectCb() between advertising and assigning to
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  pub_filtered_image_ = it_->advertiseCamera(
      "image_out", 1, connect_cb, connect_cb, connect_info_cb, connect_info_cb);
}

void MedianFilterNodelet::connectCb()
{
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  if (pub_filtered_image_.getNumSubscribers() == 0)
  {
    NODELET_INFO("Stopping median filter...");
    sub_depth_.shutdown();
  }
  else if (!sub_depth_)
  {
    NODELET_INFO("Running median filter...");
    image_transport::TransportHints hints("raw", ros::TransportHints(),
                                          getPrivateNodeHandle());
    sub_depth_ = it_->subscribeCamera(
        "image_in", queue_size_, &MedianFilterNodelet::depthCb, this, hints);
  }
}

void MedianFilterNodelet::depthCb(
    const sensor_msgs::ImageConstPtr& depth_msg,
    const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  sensor_msgs::CameraInfo cam_info = *info_msg;
  uint8_t kernel_size = config_.kernel_size;

  try
  {
    cv_ptr = cv_bridge::toCvCopy(*depth_msg, depth_msg->encoding);
  }
  catch (cv_bridge::Exception& e)
  {
    NODELET_ERROR("cv_bridge exception: %s", e.what());
  }

  if (cv_ptr->image.depth() > CV_8U && config_.kernel_size > 5)
  {
    kernel_size = 5;
    NODELET_WARN("INVALID KERNEL SIZE: You can only use a maximum kernel size "
                 "of 5 for the median filter when using images encoded with "
                 "more than 8 bits pro pixel. Using kernel size 5 instead...");
  }

  // Check if opencl is available
  if (cv::ocl::haveOpenCL())
  {
    // Apply a median filter using the OpenCL libraries of OpenCV
    cv::medianBlur(cv_ptr->image.getUMat(cv::ACCESS_READ),
                   cv_ptr->image.getUMat(cv::ACCESS_WRITE), kernel_size);
  }

  else // CPU variant
  {
    // Apply a median filter using OpenCV
    cv::medianBlur(cv_ptr->image, cv_ptr->image, kernel_size);
  }

  // Convert the data back to a ROS Image and store it in procImg
  cv_bridge::CvImage cvi;
  cvi.header = depth_msg->header;
  cvi.encoding = depth_msg->encoding;
  cvi.image = cv_ptr->image;
  cvi.toImageMsg(*current_frame_);

  pub_filtered_image_.publish(*current_frame_, cam_info, ros::Time::now());
}

void MedianFilterNodelet::dynReconfCb(MedianFilterConfig& inputConfig,
                                      uint32_t level)
{
  config_ = inputConfig;
  // Check if the kernel size is even
  if (config_.kernel_size % 2 == 0)
    config_.kernel_size += 1; // Enforce an odd kernel size.
  NODELET_INFO_STREAM("Reconfigured median filter params");
}

} // namespace pses_kinect_utilities
