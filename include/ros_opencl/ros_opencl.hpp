#pragma once

#include <vector>
#include <string>
#include <fstream>
#include <iterator>
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

#ifdef __APPLE__
    #include "OpenCL/opencl.h"
#else
    #include "CL/cl.h"
#endif

namespace ros_opencl {

class ROS_OpenCL{

    private:
        cl_kernel kernel;
        cl_context context;
        cl_program program;
        std::vector<cl_device_id> deviceIds;

        void clean();
        void checkError (const cl_int error);
        std::string LoadKernel (const char* name);
        std::string getDeviceName (const cl_device_id id);
        std::string getPlatformName (const cl_platform_id id);
        cl_program createProgram (const std::string& source, const cl_context context);

    public:
        ROS_OpenCL(){}
        ROS_OpenCL(const std::string full_kernel_path, const std::string kernel_function);
        ROS_OpenCL(const ROS_OpenCL& s){}
        ~ROS_OpenCL();
        ROS_OpenCL operator=(ROS_OpenCL* s);


        sensor_msgs::PointCloud2 process(const sensor_msgs::PointCloud2& msg);
        void process(sensor_msgs::PointCloud2::Ptr msg);

        sensor_msgs::LaserScan process(const sensor_msgs::LaserScan& msg);
        void process(sensor_msgs::LaserScan::Ptr msg);

        sensor_msgs::Image process(const sensor_msgs::Image& msg);
        void process(sensor_msgs::Image::Ptr msg);

        std::vector<float> process(const std::vector<float> v);
        void process(std::vector<float>* v);

        std::vector<double> process(const std::vector<double> v);
        void process(std::vector<double>* v);

        std::vector<int> process(const std::vector<int> v);
        void process(std::vector<int>* v);

        std::vector<char> process(const std::vector<char> v);
        void process(std::vector<char>* v);

        std::vector<char> process(const std::vector<char> v, const std::vector<char> v2, bool two_dimensional=false);
        void process(std::vector<char>* v, const std::vector<char> v2, bool two_dimensional=false);
        void process(std::vector<char>* v, std::vector<char>* v2, bool two_dimensional=false);

        std::vector<char> process(const std::vector<char> v, const std::vector<int> v2, bool two_dimensional=false);
        void process(std::vector<char>* v, const std::vector<int> v2, bool two_dimensional=false);
        void process(std::vector<char>* v, std::vector<int>* v2, bool two_dimensional=false);

        std::vector<char> process(const std::vector<char> v, const std::vector<float> v2, bool two_dimensional=false);
        void process(std::vector<char>* v, const std::vector<float> v2, bool two_dimensional=false);
        void process(std::vector<char>* v, std::vector<float>* v2, bool two_dimensional=false);

        std::vector<char> process(const std::vector<char> v, const std::vector<double> v2, bool two_dimensional=false);
        void process(std::vector<char>* v, const std::vector<double> v2, bool two_dimensional=false);
        void process(std::vector<char>* v, std::vector<double>* v2, bool two_dimensional=false);

        std::vector<int> process(const std::vector<int> v, const std::vector<char> v2, bool two_dimensional=false);
        void process(std::vector<int>* v, const std::vector<char> v2, bool two_dimensional=false);
        void process(std::vector<int>* v, std::vector<char>* v2, bool two_dimensional=false);

        std::vector<int> process(const std::vector<int> v, const std::vector<int> v2, bool two_dimensional=false);
        void process(std::vector<int>* v, const std::vector<int> v2, bool two_dimensional=false);
        void process(std::vector<int>* v, std::vector<int>* v2, bool two_dimensional=false);

        // |                             |
        // |            TODO             |
        // |  -- Not yet implemented --  |
        // |                             |
        // v                             v

        std::vector<int> process(const std::vector<int> v, const std::vector<float> v2, bool two_dimensional=false);
        void process(std::vector<int>* v, const std::vector<float> v2, bool two_dimensional=false);
        void process(std::vector<int>* v, std::vector<float>* v2, bool two_dimensional=false);

        std::vector<int> process(const std::vector<int> v, const std::vector<double> v2, bool two_dimensional=false);
        void process(std::vector<int>* v, const std::vector<double> v2, bool two_dimensional=false);
        void process(std::vector<int>* v, std::vector<double>* v2, bool two_dimensional=false);

        std::vector<float> process(const std::vector<float> v, const std::vector<char> v2, bool two_dimensional=false);
        void process(std::vector<float>* v, const std::vector<char> v2, bool two_dimensional=false);
        void process(std::vector<float>* v, std::vector<char>* v2, bool two_dimensional=false);

        std::vector<float> process(const std::vector<float> v, const std::vector<int> v2, bool two_dimensional=false);
        void process(std::vector<float>* v, const std::vector<int> v2, bool two_dimensional=false);
        void process(std::vector<float>* v, std::vector<int>* v2, bool two_dimensional=false);

        std::vector<float> process(const std::vector<float> v, const std::vector<float> v2, bool two_dimensional=false);
        void process(std::vector<float>* v, const std::vector<float> v2, bool two_dimensional=false);
        void process(std::vector<float>* v, std::vector<float>* v2, bool two_dimensional=false);

        std::vector<float> process(const std::vector<float> v, const std::vector<double> v2, bool two_dimensional=false);
        void process(std::vector<float>* v, const std::vector<double> v2, bool two_dimensional=false);
        void process(std::vector<float>* v, std::vector<double>* v2, bool two_dimensional=false);

        std::vector<double> process(const std::vector<double> v, const std::vector<char> v2);
        void process(std::vector<double>* v, const std::vector<char> v2);
        void process(std::vector<double>* v, std::vector<char>* v2);

        std::vector<double> process(const std::vector<double> v, const std::vector<int> v2);
        void process(std::vector<double>* v, const std::vector<int> v2);
        void process(std::vector<double>* v, std::vector<int>* v2);

        std::vector<double> process(const std::vector<double> v, const std::vector<float> v2);
        void process(std::vector<double>* v, const std::vector<float> v2);
        void process(std::vector<double>* v, std::vector<float>* v2);

        std::vector<double> process(const std::vector<double> v, const std::vector<double> v2);
        void process(std::vector<double>* v, const std::vector<double> v2);
        void process(std::vector<double>* v, std::vector<double>* v2);

};

}
