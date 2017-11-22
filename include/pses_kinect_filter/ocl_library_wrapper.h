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

typedef std::shared_ptr<cl::Device> device_ptr;
typedef std::shared_ptr<cl::Context> context_ptr;
typedef std::shared_ptr<cl::Program> program_ptr;
typedef std::shared_ptr<std::string> string_ptr;
typedef std::shared_ptr<cl::CommandQueue> queue_ptr;
typedef std::shared_ptr<cl::Kernel> kernel_ptr;
typedef std::shared_ptr<cl::Buffer> buffer_ptr;

static const int W_ACCESS = 0;
static const int R_ACCESS = 1;
static const int RW_ACCESS = 2;

device_ptr get_ocl_default_device();

context_ptr get_ocl_context(device_ptr device);

string_ptr load_kernel_definition(const std::string& path);

program_ptr build_ocl_program(device_ptr device, context_ptr context,
                              string_ptr kernel);

queue_ptr create_ocl_command_queue(context_ptr context, device_ptr device);

kernel_ptr create_ocl_kernel(program_ptr program,
                             const std::string& program_name);

template <typename T>
buffer_ptr create_ocl_buffer(context_ptr context, unsigned int n_elements,
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
void write_ocl_buffer(queue_ptr queue, buffer_ptr buffer, std::vector<T>& array){
  queue->enqueueWriteBuffer(*buffer, CL_TRUE, 0, sizeof(T)*array.size(), array.data());
}

template <typename T>
void write_ocl_buffer(queue_ptr queue, buffer_ptr buffer, const unsigned int size, const T* array){
  queue->enqueueWriteBuffer(*buffer, CL_TRUE, 0, sizeof(T)*size, array);
}

template <typename T>
void read_ocl_buffer(queue_ptr queue, buffer_ptr buffer, std::vector<T>& array){
  queue->enqueueReadBuffer(*buffer, CL_TRUE, 0, sizeof(T)*array.size(), array.data());
}

template <typename T>
void read_ocl_buffer(queue_ptr queue, buffer_ptr buffer, const unsigned int size, T* array){
  queue->enqueueReadBuffer(*buffer, CL_TRUE, 0, sizeof(T)*size, array);
}

#endif // OCL_LIBRARY_WRAPPER_H
