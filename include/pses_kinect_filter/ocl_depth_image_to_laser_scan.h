#ifndef OCL_DEPTH_IMAGE_TO_LASER_SCAN_H
#define OCL_DEPTH_IMAGE_TO_LASER_SCAN_H

#define CL_HPP_MINIMUM_OPENCL_VERSION 110
#define CL_HPP_TARGET_OPENCL_VERSION 110

#include <iostream>
#include <CL/cl2.hpp>
#include <ros/ros.h>

typedef std::shared_ptr<cl::Device> device_ptr;
typedef std::shared_ptr<cl::Context> context_ptr;
typedef std::shared_ptr<cl::Program> program_ptr;

device_ptr get_ocl_default_device(){
  // get all platforms (drivers), e.g. NVIDIA
  std::vector<cl::Platform> all_platforms;
  cl::Platform::get(&all_platforms);

  if (all_platforms.size()==0) {
      throw std::runtime_error("No platforms found. Check OpenCL installation!");
  }
  cl::Platform default_platform=all_platforms[0];
  std::cout << "Using platform: "<<default_platform.getInfo<CL_PLATFORM_NAME>()<<"\n";
  // get default device (CPUs, GPUs) of the default platform
  std::vector<cl::Device> all_devices;
  default_platform.getDevices(CL_DEVICE_TYPE_ALL, &all_devices);
  if(all_devices.size()==0){
      throw std::runtime_error("No devices found. Check OpenCL installation!");
  }else{
    for(cl::Device dev : all_devices){
      std::cout<<"Dev found: "<<dev.getInfo<CL_DEVICE_TYPE>()<<" "<<dev.getInfo<CL_DEVICE_NAME>()<<std::endl;
    }
  }

  // use device[1] because that's a GPU; device[0] is the CPU
  cl::Device default_device=all_devices[0];
  std::cout<< "Using device: "<<default_device.getInfo<CL_DEVICE_NAME>()<<"\n";
  return std::make_shared<cl::Device>(default_device);
}

context_ptr get_ocl_context(device_ptr device){
  // a context is like a "runtime link" to the device and platform;
  // i.e. communication is possible
  cl::Context context({*device});
  return std::make_shared<cl::Context>(context);
}

program_ptr build_ocl_program(device_ptr device, context_ptr context, const std::string& kernel){
  // create the program that we want to execute on the device
  cl::Program::Sources sources;
  sources.push_back({kernel.c_str(), kernel.length()});

  cl::Program program(*context, sources);
  if (program.build({*device}) != CL_SUCCESS) {
      throw std::runtime_error("Error building: "+program.getBuildInfo<CL_PROGRAM_BUILD_LOG>(*device));
  }
  return std::make_shared<cl::Program>(program);
}

void ocl_test(int n){
  std::string kernel =
      "   void kernel vector_sum(global const int* A, global const int* B, global int* C,"
      "                          const int n) {"
      "       int ID, Nthreads, ratio, start, stop;"
      ""
      "       ID = get_global_id(0);"
      "       Nthreads = get_global_size(0);"
      ""
      "       ratio = (n / Nthreads);"  // number of elements for each thread
      "       start = ratio * ID;"
      "       stop  = ratio * (ID + 1);"
      ""
      "       for (int i=start; i<stop; i++)"
      "          C[i] = A[i] + B[i] + i;"
      "   }";
  device_ptr dev = get_ocl_default_device();
  context_ptr cont = get_ocl_context(dev);
  program_ptr prog = build_ocl_program(dev, cont, kernel);

  // create buffers on device (allocate space on GPU)
  cl::Buffer buffer_A(*cont, CL_MEM_READ_WRITE, sizeof(int) * n);
  cl::Buffer buffer_B(*cont, CL_MEM_READ_WRITE, sizeof(int) * n);
  cl::Buffer buffer_C(*cont, CL_MEM_READ_WRITE, sizeof(int) * n);
  //cl::Buffer buffer_N(*cont, CL_MEM_READ_ONLY,  sizeof(int));
  // create things on here (CPU)
  int A[n], B[n];
  for (int i=0; i<n; i++) {
      A[i] = 1;
      B[i] = 1;
  }
  // create a queue (a queue of commands that the GPU will execute)
  cl::CommandQueue queue(*cont, *dev);
  // push write commands to queue
  queue.enqueueWriteBuffer(buffer_A, CL_TRUE, 0, sizeof(int)*n, A);
  queue.enqueueWriteBuffer(buffer_B, CL_TRUE, 0, sizeof(int)*n, B);
  //queue.enqueueWriteBuffer(buffer_N, CL_TRUE, 0, sizeof(int), &n);
  // RUN ZE KERNEL
  cl::Kernel kernel_add=cl::Kernel(*prog,"vector_sum");
  kernel_add.setArg(0,buffer_A);
  kernel_add.setArg(1,buffer_B);
  kernel_add.setArg(2,buffer_C);
  kernel_add.setArg(3,n);
  ros::Time t = ros::Time::now();
  queue.enqueueNDRangeKernel(kernel_add,cl::NDRange(50),cl::NDRange(50),cl::NDRange(50));
  queue.finish();
  ROS_INFO_STREAM("GPU Kernel execution took: " <<(ros::Time::now()-t).toSec());

  int C[n];
  // read result from GPU to here
  queue.enqueueReadBuffer(buffer_C, CL_TRUE, 0, sizeof(int)*n, C);
  std::cout<<"Result from GPU: "<<C[n-1]<<std::endl;
}

void cpu_test(int n){
  int A[n], B[n], C[n];
  for (int i=0; i<n; i++) {
      A[i] = 1;
      B[i] = 1;
  }
  ros::Time t = ros::Time::now();
  for (int i=0; i<n; i++) {
      C[i] = A[i]+B[i]+i;
  }
  ROS_INFO_STREAM("CPU Kernel execution took: " <<(ros::Time::now()-t).toSec());

  std::cout<<"Result from CPU: "<<C[n-1]<<std::endl;
}



#endif // OCL_DEPTH_IMAGE_TO_LASER_SCAN_H
