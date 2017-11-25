#ifndef DEPTH_IMAGE_TO_PCL_H
#define DEPTH_IMAGE_TO_PCL_H

#include <pses_kinect_utilities/ocl_library_wrapper.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/Image.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;

namespace pses_kinect_utilities
{

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

typedef struct
{
  cl_float cx;
  cl_float cy;
  cl_float fx;
  cl_float fy;
} Transform;

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
  device_ptr device;
  context_ptr context;
  program_ptr program;
  string_ptr kernel_definition;
  queue_ptr queue;
  kernel_ptr kernel;
  std::vector<buffer_ptr> buffers;

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
