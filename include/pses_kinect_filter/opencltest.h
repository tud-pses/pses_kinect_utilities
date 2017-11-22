#ifndef OPENCLTEST_H
#define OPENCLTEST_H

#define CL_HPP_MINIMUM_OPENCL_VERSION 110
#define CL_HPP_TARGET_OPENCL_VERSION 110

#include <iostream>
#include <CL/cl2.hpp>
#include <ros/ros.h>
#include <pses_kinect_filter/ocl_library_wrapper.h>

void ocl_test(int n, const std::string& path)
{
  //Device and Kernel setup
  device_ptr device;
  context_ptr context;
  program_ptr program;
  queue_ptr queue;
  kernel_ptr k;
  string_ptr kernel_def;
  //kernel_def = std::make_shared<std::string>(kernel_code);
  try{
    device = get_ocl_default_device();
    context = get_ocl_context(device);
    kernel_def = load_kernel_definition(path);
    program =
        build_ocl_program(device, context, kernel_def);
    queue = create_ocl_command_queue(context, device);
    k = create_ocl_kernel(program, "test2");
  }catch(std::exception& e){
    std::cout<<"An Error occured during OCL set up: "<<e.what()<<std::endl;
    return;
  }

  // initalize buffers and kernel
  buffer_ptr a = create_ocl_buffer<int>(context, n, RW_ACCESS);
  buffer_ptr b = create_ocl_buffer<int>(context, n, RW_ACCESS);
  buffer_ptr c = create_ocl_buffer<int>(context, n, RW_ACCESS);

  k->setArg(0,*a);
  k->setArg(1,*b);
  k->setArg(2,*c);
  k->setArg(3,n);

  // set and push data to device
  std::vector<int> A(n), B(n), C(n);
  for (int i = 0; i < n; i++)
  {
    A[i] = 1;
    B[i] = 1;
  }
  write_ocl_buffer(queue, a, A);
  write_ocl_buffer(queue, b, B);

  ros::Time t = ros::Time::now();
  queue->enqueueNDRangeKernel(*k, cl::NullRange, cl::NDRange(n/10),
                             cl::NullRange);
  queue->finish();
  ROS_INFO_STREAM(
      "GPU Kernel execution took: " << (ros::Time::now() - t).toSec());

  // read result from GPU to here
  read_ocl_buffer(queue, c, C);

  std::cout << "Result from GPU: " << C[n - 1] << std::endl;

  // shut down
  queue->flush();
}

void cpu_test(int n)
{
  int A[n], B[n], C[n];
  for (int i = 0; i < n; i++)
  {
    A[i] = 1;
    B[i] = 1;
  }
  ros::Time t = ros::Time::now();
  for (int i = 0; i < n; i++)
  {
    C[i] = A[i] + B[i] + i;
  }
  ROS_INFO_STREAM(
      "CPU Kernel execution took: " << (ros::Time::now() - t).toSec());

  std::cout << "Result from CPU: " << C[n - 1] << std::endl;
}

int ocltest_sanity() {
    // get all platforms (drivers), e.g. NVIDIA
    std::vector<cl::Platform> all_platforms;
    cl::Platform::get(&all_platforms);

    if (all_platforms.size()==0) {
        std::cout<<" No platforms found. Check OpenCL installation!\n";
        exit(1);
    }
    cl::Platform default_platform=all_platforms[0];
    std::cout << "Using platform: "<<default_platform.getInfo<CL_PLATFORM_NAME>()<<"\n";
    // get default device (CPUs, GPUs) of the default platform
    std::vector<cl::Device> all_devices;
    default_platform.getDevices(CL_DEVICE_TYPE_ALL, &all_devices);
    if(all_devices.size()==0){
        std::cout<<" No devices found. Check OpenCL installation!\n";
        exit(1);
    }else{
      for(cl::Device dev : all_devices){
        std::cout<<"Dev found: "<<dev.getInfo<CL_DEVICE_TYPE>()<<" "<<dev.getInfo<CL_DEVICE_NAME>()<<std::endl;
      }
    }

    // use device[1] because that's a GPU; device[0] is the CPU
    cl::Device default_device=all_devices[0];
    std::cout<< "Using device: "<<default_device.getInfo<CL_DEVICE_NAME>()<<"\n";

    // a context is like a "runtime link" to the device and platform;
    // i.e. communication is possible
    cl::Context context({default_device});
    // create the program that we want to execute on the device
    cl::Program::Sources sources;
    // calculates for each element; C = A + B
    std::string kernel_code=
        "   void kernel simple_add(global const int* A, global const int* B, global int* C, "
        "                          global const int* N) {"
        "       int ID, Nthreads, n, ratio, start, stop;"
        ""
        "       ID = get_global_id(0);"
        "       Nthreads = get_global_size(0);"
        "       n = N[0];"
        ""
        "       ratio = (n / Nthreads);"  // number of elements for each thread
        "       start = ratio * ID;"
        "       stop  = ratio * (ID + 1);"
        ""
        "       for (int i=start; i<stop; i++)"
        "           C[i] = A[i] + B[i] + i;"
        "   }";
    sources.push_back({kernel_code.c_str(), kernel_code.length()});

    cl::Program program(context, sources);
    if (program.build({default_device}) != CL_SUCCESS) {
        std::cout << "Error building: " << program.getBuildInfo<CL_PROGRAM_BUILD_LOG>(default_device) << std::endl;
        exit(1);
    }

    // apparently OpenCL only likes arrays ...
    // N holds the number of elements in the vectors we want to add
    int N[1] = {500000};
    int n = N[0];

    // create buffers on device (allocate space on GPU)
    cl::Buffer buffer_A(context, CL_MEM_READ_WRITE, sizeof(int) * n);
    cl::Buffer buffer_B(context, CL_MEM_READ_WRITE, sizeof(int) * n);
    cl::Buffer buffer_C(context, CL_MEM_READ_WRITE, sizeof(int) * n);
    cl::Buffer buffer_N(context, CL_MEM_READ_ONLY,  sizeof(int));
    // create things on here (CPU)
    int A[n], B[n];
    for (int i=0; i<n; i++) {
        A[i] = 1;
        B[i] = 1;
    }
    // create a queue (a queue of commands that the GPU will execute)
    cl::CommandQueue queue(context, default_device);
    // push write commands to queue
    queue.enqueueWriteBuffer(buffer_A, CL_TRUE, 0, sizeof(int)*n, A);
    queue.enqueueWriteBuffer(buffer_B, CL_TRUE, 0, sizeof(int)*n, B);
    queue.enqueueWriteBuffer(buffer_N, CL_TRUE, 0, sizeof(int),   N);
    // RUN ZE KERNEL
    //cl::KernelFunctor simple_add(cl::Kernel(program, "simple_add"), queue, cl::NullRange, cl::NDRange(10), cl::NullRange);
    //simple_add(buffer_A, buffer_B, buffer_C, buffer_N);
    cl::Kernel kernel_add=cl::Kernel(program,"simple_add");
    kernel_add.setArg(0,buffer_A);
    kernel_add.setArg(1,buffer_B);
    kernel_add.setArg(2,buffer_C);
    kernel_add.setArg(3,buffer_N);
    queue.enqueueNDRangeKernel(kernel_add,cl::NullRange,cl::NDRange(10),cl::NullRange);
    queue.finish();

    int C[n];
    // read result from GPU to here
    queue.enqueueReadBuffer(buffer_C, CL_TRUE, 0, sizeof(int)*n, C);
    std::cout << "Result from GPU (sanity): " << C[n - 1] << std::endl;

    return 0;
}

#endif // OPENCLTEST_H
