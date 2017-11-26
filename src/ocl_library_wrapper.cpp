/**
 * @file "ocl_library_wrapper.cpp"
 * @brief Nodelet implementation of a depth image to XYZ point cloud conversion, containing the callback functions.
 *
*/

#include <pses_kinect_utilities/ocl_library_wrapper.h>

namespace pses_kinect_utilities
{

DevicePtr get_ocl_default_device()
{
  // get all platforms (drivers), e.g. NVIDIA
  std::vector<cl::Platform> all_platforms;
  cl::Platform::get(&all_platforms);

  if (all_platforms.size() == 0)
  {
    throw std::runtime_error("No platforms found. Check OpenCL installation!");
  }
  cl::Platform default_platform = all_platforms[0];
  std::cout << "Using platform: "
            << default_platform.getInfo<CL_PLATFORM_NAME>() << "\n";
  // get default device (CPUs, GPUs) of the default platform
  std::vector<cl::Device> all_devices;
  default_platform.getDevices(CL_DEVICE_TYPE_ALL, &all_devices);
  if (all_devices.size() == 0)
  {
    throw std::runtime_error("No devices found. Check OpenCL installation!");
  }
  else
  {
    for (cl::Device dev : all_devices)
    {
      std::cout << "Dev found: " << dev.getInfo<CL_DEVICE_TYPE>() << " "
                << dev.getInfo<CL_DEVICE_NAME>() << std::endl;
    }
  }

  // use device[1] because that's a GPU; device[0] is the CPU
  cl::Device default_device = all_devices[0];
  std::cout << "Using device: " << default_device.getInfo<CL_DEVICE_NAME>()
            << "\n";
  return std::make_shared<cl::Device>(default_device);
}

ContextPtr get_ocl_context(DevicePtr device)
{
  // a context is like a "runtime link" to the device and platform;
  // i.e. communication is possible
  cl::Context context({*device});
  return std::make_shared<cl::Context>(context);
}

StringPtr load_kernel_definition(const std::string& path)
{
  std::ifstream file(path);
  std::stringstream ss = std::stringstream();
  char buffer[65500];
  while (file.getline(buffer, 65500))
  {
    ss << buffer << "\n";
  }

  return std::make_shared<std::string>(ss.str());
}

ProgramPtr build_ocl_program(DevicePtr device, ContextPtr context,
                              StringPtr kernel)
{
  // create the program that we want to execute on the device
  cl::Program::Sources sources;
  sources.push_back({kernel->c_str(), kernel->length()});

  cl::Program program(*context, sources);
  if (program.build({*device}) != CL_SUCCESS)
  {
    throw std::runtime_error(
        "Error building: " +
        program.getBuildInfo<CL_PROGRAM_BUILD_LOG>(*device));
  }
  return std::make_shared<cl::Program>(program);
}

QueuePtr create_ocl_command_queue(ContextPtr context, DevicePtr device)
{
  cl::CommandQueue queue(*context, *device);
  return std::make_shared<cl::CommandQueue>(queue);
}

KernelPtr create_ocl_kernel(ProgramPtr program,
                             const std::string& program_name)
{
  cl::Kernel kernel = cl::Kernel(*program, program_name.c_str());
  return std::make_shared<cl::Kernel>(kernel);
}
} // namespace pses_kinect_utilities
