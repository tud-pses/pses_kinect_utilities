/**
 * @file "pses_utilities/depth_image_to_pcl.h"
 * @brief Contains help functions, the DepthImageToPCL class and some structures that make
 * the implementation of the PointCloudXYZNodelet nodelet cleaner.
 *
*/

#ifndef DEPTH_IMAGE_TO_PCL_H
#define DEPTH_IMAGE_TO_PCL_H

#include <pses_kinect_utilities/ocl_library_wrapper.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/Image.h>

/**
 * @typedef pcl::PointCloud<pcl::PointXYZ> PointCloud
 * @brief Shortcut for a XYZ Point cloud of the pcl library.
*/
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

/**
 * @typedef PointCloud::Ptr PointCloudPtr
 * @brief Shortcut for a pointer of a XYZ Point cloud of the pcl library.
*/
typedef PointCloud::Ptr PointCloudPtr;

/**
 * @namespace pses_kinect_utilities
 * @brief This namespace is used by the nodelets inside our package pses_kinect_utilities.
 *
*/
namespace pses_kinect_utilities
{

/**
 * @struct MetaData depth_image_to_pcl.h
 * @brief Contains some parameters and metada data of the camera.
 *
*/
typedef struct
{
  cl_uint width;
  cl_uint height;
  cl_uint n_pixels;
  cl_float depth_scaling;
  cl_uint invalid_depth;
  cl_float max_depth;
  cl_float NaN;
} MetaData;

/**
 * @struct Transform depth_image_to_pcl.h
 * @brief Contains the camera calibration.
 *
*/
typedef struct
{
  cl_float cx; /**< X coordinate of the principal point. */
  cl_float cy; /**< y coordinate of the principal point. */
  cl_float fx; /**< X coordinate of the focal length. */
  cl_float fy; /**< Y coordinate of the focal length. */
} Transform;


/**
 * @class DepthImageToPCL depth_image_to_pcl.h
 * @brief Class that make
 * the implementation of the PointCloudXYZNodelet nodelet cleaner.
 *
*/
class DepthImageToPCL
{
public:
  DepthImageToPCL();
  DepthImageToPCL(const MetaData& md, const Transform& tf);
  void setMetaData(const MetaData& md);
  void setTFData(const Transform& tf);
  void initCloud();
  void init_CL(const std::string& kernel_file);
  void program_kernel(const std::string& kernel_function);
  void init_buffers();
  PointCloudPtr convert_to_pcl(const sensor_msgs::Image::ConstPtr rawImgPtr);

private:
  DevicePtr device;
  ContextPtr context;
  ProgramPtr program;
  StringPtr kernel_definition;
  QueuePtr queue;
  KernelPtr kernel;
  std::vector<BufferPtr> buffers;

  PointCloud cloud;

  bool cl_init;
  bool kernel_init;
  bool buffer_init;

  MetaData md;
  Transform tf;

  void fill_buffer(const unsigned char* image);
  void read_buffer();
};

} // namespace pses_kinect_utilities

#endif // DEPTH_IMAGE_TO_PCL_H
