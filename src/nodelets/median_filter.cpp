#include <pses_kinect_utilities/median_filter.h>
#include <image_transport/image_transport.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/ocl.hpp>

// Register as nodelet
PLUGINLIB_EXPORT_CLASS(pses_kinect_utilities::MedianFilterNodelet,
                       nodelet::Nodelet);

namespace enc = sensor_msgs::image_encodings;

namespace pses_kinect_utilities
{

void MedianFilterNodelet::onInit()
{
  NODELET_DEBUG("Initializing nodelet...");

  ros::NodeHandle& private_nh = getPrivateNodeHandle();
  current_frame_.reset(new sensor_msgs::Image);

  // Read parameters
  private_nh.param("queue_size", queue_size_, 1);
  private_nh.param("kernel_size", kernel_size_, 5);
  private_nh.param<std::string>("depth_image_topic", depth_image_topic_,
                                "/kinect2/sd/image_depth_rect");
  private_nh.param<std::string>("output_topic", output_topic_,
                                "/kinect_utilities/depth_image");

  // Monitor whether anyone is subscribed to the output
  ros::SubscriberStatusCallback connect_cb =
      boost::bind(&MedianFilterNodelet::connectCb, this);
  // Make sure we don't enter connectCb() between advertising and assigning to
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  pub_filtered_image_ = nh_.advertise<sensor_msgs::Image>(
      output_topic_, 1, connect_cb, connect_cb);
}

void MedianFilterNodelet::connectCb()
{
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  if (pub_filtered_image_.getNumSubscribers() == 0)
  {
    ROS_INFO("unsuscribed!");
    sub_depth_.shutdown();
  }
  else if (!sub_depth_)
  {
    ROS_INFO("Suscribed!");
    sub_depth_ = nh_.subscribe<sensor_msgs::Image>(
        depth_image_topic_, 1,
        boost::bind(&MedianFilterNodelet::depthCb, this, _1));
  }
}

void MedianFilterNodelet::depthCb(const sensor_msgs::ImageConstPtr& depth_msg)
{
  cv_bridge::CvImagePtr cv_ptr;

  try
  {
    cv_ptr = cv_bridge::toCvCopy(*depth_msg, depth_msg->encoding);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }

  // Check if opencl is available
  if (cv::ocl::haveOpenCL())
  {
    // Apply a median filter using the OpenCL libraries of OpenCV
    cv::medianBlur(cv_ptr->image.getUMat(cv::ACCESS_READ),
                   cv_ptr->image.getUMat(cv::ACCESS_WRITE), kernel_size_);
  }

  else // CPU variant
  {
    // Apply a median filter using OpenCV
    cv::medianBlur(cv_ptr->image, cv_ptr->image, kernel_size_);
  }

  // Convert the data back to a ROS Image and store it in procImg
  cv_bridge::CvImage cvi;
  cvi.header = depth_msg->header;
  cvi.encoding = depth_msg->encoding;
  cvi.image = cv_ptr->image;
  cvi.toImageMsg(*current_frame_);

  pub_filtered_image_.publish(current_frame_);
}

} // namespace pses_kinect_utilities
