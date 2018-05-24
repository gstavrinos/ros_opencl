#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include "ros_opencl/ros_opencl.hpp"

using namespace std;

ros_opencl::ROS_OpenCL roscl;

int main (int argc, char** argv){
    ros::init (argc, argv, "ros_opencl_int_vector_buffer_size_test1");
    ros::NodeHandle nh;
    string kernel_filename;

    nh.param("ros_opencl_int_vector_buffer_size_test1/kernel_filename", kernel_filename, string("test_kernels.cl"));

    string full_kernel_path = ros::package::getPath("ros_opencl") + "/tests/kernels/" + kernel_filename;

    roscl = new ros_opencl::ROS_OpenCL(full_kernel_path, "buffSizeTest");

    std::vector<int> v;
    v.push_back(1);
    v.push_back(2);
    v.push_back(3);

    std::vector<size_t> g_work_size;
    std::vector<size_t> buffer_size;
    g_work_size.push_back(2);
    buffer_size.push_back(3);

    v = roscl.process(v, g_work_size, buffer_size);


    ROS_INFO("The vector from [1, 2, 3] changed to [%d, %d, %d].", v[0], v[1], v[2]);
}