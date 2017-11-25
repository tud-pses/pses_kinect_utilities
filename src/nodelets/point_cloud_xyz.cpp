#include <pses_kinect_utilities/point_cloud_xyz.h>
#include <image_transport/image_transport.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <opencv2/core/ocl.hpp>
#include <ros/package.h>

// Register as nodelet
PLUGINLIB_EXPORT_CLASS(pses_kinect_utilities::PointCloudXYZNodelet,
                       nodelet::Nodelet);

namespace pses_kinect_utilities
{

void PointCloudXYZNodelet::onInit()
{
  NODELET_DEBUG("Initializing pointcloud XYZ nodelet...");

  ros::NodeHandle& private_nh = getPrivateNodeHandle();
  ros::NodeHandle& nh = getNodeHandle();

  current_cloud_.reset(new PointCloudMsg);
  pcl_conversion_.reset(new DepthImageToPCL);
  it_.reset(new image_transport::ImageTransport(nh));
  kernel_ready_ = false;

  // Read parameters
  private_nh.param("queue_size", queue_size_, 1);
  private_nh.param<std::string>("tf_frame", tf_frame_, "kinect2_link");
  private_nh.param<std::string>("output_topic", output_topic_,
                                "/kinect_utilities/points");
  private_nh.param<std::string>("camera_info_topic", camera_info_topic_,
                                "/kinect_utilities/camera_info");
  private_nh.param<std::string>("depth_image_topic", depth_image_topic_,
                                "/kinect_utilities/depth_image");
  private_nh.param<std::string>("cl_file_path", cl_file_path_,
                                ros::package::getPath("pses_kinect_utilities") +
                                    "/ocl_kernel/ocl_kernel.cl");

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
    NODELET_INFO("Stopping conversion from depth image to XYZ pointcloud...");
    sub_depth_.shutdown();
  }
  else if (!sub_depth_)
  {
    NODELET_INFO("Running conversion from depth image to XYZ pointcloud...");
    image_transport::TransportHints hints("raw", ros::TransportHints(),
                                          getPrivateNodeHandle());
    sub_depth_ =
        it_->subscribeCamera(depth_image_topic_, queue_size_,
                             &PointCloudXYZNodelet::depthCb, this, hints);
  }
}

void PointCloudXYZNodelet::depthCb(
    const sensor_msgs::ImageConstPtr& depth_msg,
    const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  cv_bridge::CvImagePtr cv_ptr;

  try
  {
    cv_ptr = cv_bridge::toCvCopy(*depth_msg, depth_msg->encoding);
  }
  catch (cv_bridge::Exception& e)
  {
    NODELET_ERROR("cv_bridge exception: %s", e.what());
  }

  // Check if opencl is available
  if (cv::ocl::haveOpenCL())
  {

    if (!kernel_ready_)
    {
      kernel_ready_ = true;
      camera_model_.fromCameraInfo(info_msg);
      MetaData md;
      md.width = (cl_uint)depth_msg->width;
      md.height = (cl_uint)depth_msg->height;
      md.n_pixels = (cl_uint)depth_msg->width * depth_msg->height;
      md.depth_scaling = (cl_float)0.001f;
      md.invalid_depth = (cl_uint)0;
      md.max_depth = (cl_float)0.0f;
      md.NaN = std::numeric_limits<float>::quiet_NaN();
      Transform tf;
      tf.cx = (cl_float)camera_model_.cx();
      tf.cy = (cl_float)camera_model_.cy();
      tf.fx = (cl_float)camera_model_.fx();
      tf.fy = (cl_float)camera_model_.fy();
      pcl_conversion_->setMetaData(md);
      pcl_conversion_->setTFData(tf);
      pcl_conversion_->initCloud();
      try
      {
        pcl_conversion_->init_CL(cl_file_path_);
        NODELET_INFO_STREAM("Loading opencl kernel from path: " << cl_file_path_);
        pcl_conversion_->program_kernel("depth_to_pcl");
        pcl_conversion_->init_buffers();
      }
      catch (std::exception& e)
      {
        NODELET_ERROR_STREAM("An error occured during OCL setup! " << e.what());
        ros::shutdown();
      }
    }

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
      NODELET_ERROR_STREAM("An error occured during depth to pcl conversion! "
                           << e.what());
    }

    current_cloud_->header.frame_id = tf_frame_;
    current_cloud_->is_dense = false;
    pub_cloud_.publish(current_cloud_);
  }

  // else // CPU variant
  //{
  //
  // TODO
  //}
}

} // namespace pses_kinect_utilities
