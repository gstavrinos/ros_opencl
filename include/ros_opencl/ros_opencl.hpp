#pragma once

#include <vector>
#include <string>
#include <fstream>
#include <iterator>
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#ifdef __APPLE__
    #include "OpenCL/opencl.h"
#else
    #include "CL/cl.h"
#endif

namespace ros_opencl {

class ROS_OpenCL{

    protected:
        cl_kernel kernel;
        cl_context context;
        cl_program program;
        std::vector<cl_device_id> deviceIds;

    private:
        void checkError (const cl_int error);
        std::string LoadKernel (const char* name);
        std::string getDeviceName (const cl_device_id id);
        std::string getPlatformName (const cl_platform_id id);
        cl_program createProgram (const std::string& source, const cl_context context);

    public:
        ROS_OpenCL(){}
        ROS_OpenCL(const std::string full_kernel_path, const std::string kernel_function);
        virtual ~ROS_OpenCL(){}

        sensor_msgs::PointCloud2 process(const sensor_msgs::PointCloud2& msg);
        void process(sensor_msgs::PointCloud2::Ptr msg);
        void clean();
};

}
