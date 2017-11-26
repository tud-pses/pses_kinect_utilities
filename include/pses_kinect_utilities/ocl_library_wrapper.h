/**
 * @file "pses_utilities/ocl_library_wrapper.h"
 * @brief Wrapper for the opencl library that contains help functions and typedefs
 * that make the opencl library easier to use.
 *
*/

#ifndef OCL_LIBRARY_WRAPPER_H
#define OCL_LIBRARY_WRAPPER_H

#define CL_HPP_MINIMUM_OPENCL_VERSION 110
#define CL_HPP_TARGET_OPENCL_VERSION 110

#include <iostream>
#include <CL/cl2.hpp>
#include <vector>
#include <fstream>
#include <boost/shared_ptr.hpp>
#include <string>
#include <sstream>

namespace pses_kinect_utilities
{

typedef std::shared_ptr<cl::Device> DevicePtr;
typedef std::shared_ptr<cl::Context> ContextPtr;
typedef std::shared_ptr<cl::Program> ProgramPtr;
typedef std::shared_ptr<std::string> StringPtr;
typedef std::shared_ptr<cl::CommandQueue> QueuePtr;
typedef std::shared_ptr<cl::Kernel> KernelPtr;
typedef std::shared_ptr<cl::Buffer> BufferPtr;

static const int W_ACCESS = 0;
static const int R_ACCESS = 1;
static const int RW_ACCESS = 2;

DevicePtr get_ocl_default_device();

ContextPtr get_ocl_context(DevicePtr device);

StringPtr load_kernel_definition(const std::string& path);

ProgramPtr build_ocl_program(DevicePtr device, ContextPtr context,
                              StringPtr kernel);

QueuePtr create_ocl_command_queue(ContextPtr context, DevicePtr device);

KernelPtr create_ocl_kernel(ProgramPtr program,
                             const std::string& program_name);

template <typename T>
BufferPtr create_ocl_buffer(ContextPtr context, unsigned int n_elements,
                             int access_type)
{
  cl::Buffer buffer;
  switch (access_type)
  {
  case W_ACCESS:
  {
    buffer = cl::Buffer(*context, CL_MEM_WRITE_ONLY, sizeof(T) * n_elements);
    break;
  }
  case R_ACCESS:
  {
    buffer = cl::Buffer(*context, CL_MEM_READ_ONLY, sizeof(T) * n_elements);
    break;
  }
  case RW_ACCESS:
  {
    buffer = cl::Buffer(*context, CL_MEM_READ_WRITE, sizeof(T) * n_elements);
    break;
  }
  default:
  {
    throw std::runtime_error("Unknown access type!");
  }
  }
  return std::make_shared<cl::Buffer>(buffer);
}

template <typename T>
void write_ocl_buffer(QueuePtr queue, BufferPtr buffer, std::vector<T>& array)
{
  queue->enqueueWriteBuffer(*buffer, CL_TRUE, 0, sizeof(T) * array.size(),
                            array.data());
}

template <typename T>
void write_ocl_buffer(QueuePtr queue, BufferPtr buffer,
                      const unsigned int size, const T* array)
{
  queue->enqueueWriteBuffer(*buffer, CL_TRUE, 0, sizeof(T) * size, array);
}

template <typename T>
void read_ocl_buffer(QueuePtr queue, BufferPtr buffer, std::vector<T>& array)
{
  queue->enqueueReadBuffer(*buffer, CL_TRUE, 0, sizeof(T) * array.size(),
                           array.data());
}

template <typename T>
void read_ocl_buffer(QueuePtr queue, BufferPtr buffer,
                     const unsigned int size, T* array)
{
  queue->enqueueReadBuffer(*buffer, CL_TRUE, 0, sizeof(T) * size, array);
}

} // namespace pses_kinect_utilities

#endif // OCL_LIBRARY_WRAPPER_H
