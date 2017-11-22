#include <pses_kinect_filter/depth_image_to_pcl.h>

DepthImageToPCL::DepthImageToPCL() {
  cl_init = false;
  kernel_init = false;
  buffer_init = false;
}

DepthImageToPCL::DepthImageToPCL(const meta_data& md, const transform& tf)
    : md(md), tf(tf)
{
  cl_init = false;
  kernel_init = false;
  buffer_init = false;
  cloud = point_cloud(md.width, md.height);
}

void DepthImageToPCL::setMetaData(const meta_data& md){
  this->md = md;
}
void DepthImageToPCL::setTFData(const transform& tf) {
  this->tf = tf;
}

void DepthImageToPCL::initCloud(){
  cloud = point_cloud(md.width, md.height);
}

void DepthImageToPCL::init_CL(const std::string& kernel_file)
{
  device = get_ocl_default_device();
  context = get_ocl_context(device);
  kernel_definition = load_kernel_definition(kernel_file);
  program = build_ocl_program(device, context, kernel_definition);
  queue = create_ocl_command_queue(context, device);
  buffers = std::vector<buffer_ptr>();
  cl_init = true;
}

void DepthImageToPCL::program_kernel(const std::string& kernel_function)
{
  if (!cl_init)
    throw std::runtime_error("Cl components have not yet been initilized!");
  kernel = create_ocl_kernel(program, kernel_function);
  kernel_init = true;
}

void DepthImageToPCL::init_buffers()
{
  if (!cl_init || !kernel_init)
    throw std::runtime_error(
        "Cl components or program kernel have not yet been initilized!");
  buffers.push_back(
      create_ocl_buffer<unsigned short>(context, md.n_pixels, R_ACCESS));
  buffers.push_back(create_ocl_buffer<float>(context, md.n_pixels * 3, RW_ACCESS));
  kernel->setArg(0,*buffers[0]);
  kernel->setArg(1,*buffers[1]);
  kernel->setArg(2,md);
  kernel->setArg(3,tf);
  buffer_init = true;
}

point_cloud_ptr
DepthImageToPCL::convert_to_pcl(const sensor_msgs::Image::ConstPtr img_in)
{
  if (!cl_init || !kernel_init || !buffer_init)
    throw std::runtime_error("Cl components, program kernel or buffers have "
                             "not yet been initilized!");
  fill_buffer(img_in->data.data());
  queue->enqueueNDRangeKernel(*kernel, cl::NullRange, cl::NDRange(md.n_pixels/100),
                             cl::NullRange);
  queue->finish();
  read_buffer();

  return std::make_shared<point_cloud>(cloud);
}

void DepthImageToPCL::fill_buffer(const unsigned char* image)
{
  write_ocl_buffer(queue, buffers[0], md.n_pixels * 2, image);
}

void DepthImageToPCL::read_buffer(){
  read_ocl_buffer(queue, buffers[1], md.n_pixels, cloud.points.data());
}
