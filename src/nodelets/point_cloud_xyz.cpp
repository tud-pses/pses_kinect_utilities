#include <pses_kinect_utilities/point_cloud_xyz.h>
#include <image_transport/image_transport.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/point_cloud2_iterator.h>

// Register as nodelet
PLUGINLIB_EXPORT_CLASS(pses_kinect_utilities::PointCloudXYZNodelet,
                       nodelet::Nodelet);

namespace enc = sensor_msgs::image_encodings;

namespace pses_kinect_utilities
{

void PointCloudXYZNodelet::onInit()
{
  NODELET_DEBUG("Initializing nodelet...");

  ros::NodeHandle& private_nh = getPrivateNodeHandle();
  current_cloud_.reset(new PointCloudMsg);
  pcl_conversion_.reset(new DepthImageToPCL);

  // Read parameters
  private_nh.param("queue_size", queue_size_, 1);
  private_nh.param("kernel_size", kernel_size_, 5);
  private_nh.param<std::string>("tf_frame", tf_frame_, "kinect2_link");
  private_nh.param<std::string>("output_topic", output_topic_,
                                "/kinect_utilies/points");
  private_nh.param<std::string>("camera_info_topic", camera_info_topic_,
                                "/kinect2/sd/camera_info");
  private_nh.param<std::string>("depth_image_topic", depth_image_topic_,
                                "/kinect_utilities/depth_image");

  // Monitor whether anyone is subscribed to the output
  ros::SubscriberStatusCallback connect_cb =
      boost::bind(&PointCloudXYZNodelet::connectCb, this);
  // Make sure we don't enter connectCb() between advertising and assigning to
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  pub_cloud_ =
      nh.advertise<PointCloud>(output_topic_, 1, connect_cb, connect_cb);
}

void PointCloudXYZNodelet::connectCb()
{
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  if (pub_cloud_.getNumSubscribers() == 0)
  {
    ROS_INFO("unsuscribed!");
    sub_depth_.shutdown();
  }
  else if (!sub_depth_)
  {
    ROS_INFO("Suscribed!");
    sub_depth_ = nh.subscribe<sensor_msgs::Image>(
        depth_image_topic_, 1,
        boost::bind(&PointCloudXYZNodelet::depthCb, this, _1));
  }
}

void PointCloudXYZNodelet::depthCb(const sensor_msgs::ImageConstPtr& depth_msg)
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
  // if (cv::ocl::haveOpenCL())
  //{
  current_cloud_->header = depth_msg->header;
  current_cloud_->is_dense = false;
  current_cloud_->is_bigendian = false;
  current_cloud_->height = depth_msg->height;
  current_cloud_->width = depth_msg->width;
  sensor_msgs::PointCloud2Modifier pcd_mod(*current_cloud_);
  pcd_mod.setPointCloud2FieldsByString(1, "xyz");

  try
  {
    PointCloudPtr pc = pcl_conversion_->convert_to_pcl(depth_msg);
    pcl::toROSMsg(*pc, *current_cloud_);
  }
  catch (std::exception& e)
  {
    ROS_ERROR_STREAM("An error occured during depth to pcl conversion! "
                     << e.what());
  }

  current_cloud_->header.frame_id = tf_frame_;
  current_cloud_->is_dense = false;
  //}

  // else // CPU variant
  //{
  //
  // TODO
  //}
}

} // namespace pses_kinect_utilities
