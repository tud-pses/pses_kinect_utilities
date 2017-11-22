#ifndef DEPTH_IMAGE_TO_PCL_H
#define DEPTH_IMAGE_TO_PCL_H

#include <pses_kinect_filter/ocl_library_wrapper.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/Image.h>

typedef pcl::PointCloud<pcl::PointXYZ> point_cloud;
typedef std::shared_ptr<point_cloud> point_cloud_ptr;

typedef struct{
  cl_uint width;
  cl_uint height;
  cl_uint n_pixels;
  cl_float depth_scaling;
}meta_data;

typedef struct{
  cl_float cx;
  cl_float cy;
  cl_float fx;
  cl_float fy;
}transform;

class DepthImageToPCL{
public:
  DepthImageToPCL();
  DepthImageToPCL(const meta_data& md, const transform& tf);
  void setMetaData(const meta_data& md);
  void setTFData(const transform& tf);
  void initCloud();
  void init_CL(const std::string& kernel_file);
  void program_kernel(const std::string& kernel_function);
  void init_buffers();
  point_cloud_ptr convert_to_pcl(const sensor_msgs::Image::ConstPtr rawImgPtr);

private:
  device_ptr device;
  context_ptr context;
  program_ptr program;
  string_ptr kernel_definition;
  queue_ptr queue;
  kernel_ptr kernel;
  std::vector<buffer_ptr> buffers;

  point_cloud cloud;

  bool cl_init;
  bool kernel_init;
  bool buffer_init;

  meta_data md;
  transform tf;

  void fill_buffer(const unsigned char* image);
  void read_buffer();

};


#endif // DEPTH_IMAGE_TO_PCL_H
